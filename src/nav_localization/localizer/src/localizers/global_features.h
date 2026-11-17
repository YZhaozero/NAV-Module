#pragma once

// Bounded coarse-proposal preparation only. Neither the registration map nor
// the full-resolution query is changed. Feature math uses PCL's own SPFH/FPFH
// routines; checkpoints split its whole-cloud work into cancellable batches.
#include "commons.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <omp.h>

namespace bounded_global {
using Checkpoint = std::function<void()>;
using Features = pcl::PointCloud<pcl::FPFHSignature33>;
inline constexpr std::size_t kPointBudget = 24000;
inline constexpr std::size_t kFeatureBatch = 128;

inline void check(const Checkpoint &checkpoint) { if (checkpoint) checkpoint(); }

struct Bounds {
    V3D minimum = V3D::Constant(std::numeric_limits<double>::infinity());
    V3D maximum = V3D::Constant(-std::numeric_limits<double>::infinity());
};

inline Bounds bounds(const CloudType::ConstPtr &cloud, const Checkpoint &checkpoint) {
    if (!cloud || cloud->empty()) throw std::invalid_argument("Empty global feature cloud");
    Bounds result;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
        if (i % 4096 == 0) check(checkpoint);
        const auto &point = cloud->points[i];
        const V3D xyz(point.x, point.y, point.z);
        if (!xyz.allFinite()) throw std::invalid_argument("Nonfinite global feature input");
        result.minimum = result.minimum.cwiseMin(xyz);
        result.maximum = result.maximum.cwiseMax(xyz);
    }
    return result;
}

inline bool gridFits(const Bounds &bounds, double leaf) {
    long double cells = 1.;
    for (int axis = 0; axis < 3; ++axis) {
        const long double low = std::floor(static_cast<long double>(bounds.minimum[axis]) / leaf);
        const long double high = std::floor(static_cast<long double>(bounds.maximum[axis]) / leaf);
        if (low < std::numeric_limits<int32_t>::min() || high > std::numeric_limits<int32_t>::max()) return false;
        cells *= high - low + 1.;
    }
    return cells < static_cast<long double>(std::numeric_limits<int32_t>::max()) * .99L;
}

inline CloudType::Ptr voxelize(const CloudType::ConstPtr &cloud, double leaf,
                              const Checkpoint &checkpoint) {
    check(checkpoint);
    pcl::VoxelGrid<PointType> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(leaf, leaf, leaf);
    CloudType::Ptr output(new CloudType);
    grid.filter(*output);
    check(checkpoint);
    return output;
}

struct ProposalClouds {
    CloudType::Ptr source, target;
    double voxel_size = 0.;
    double feature_radius = 0.;
};

inline ProposalClouds prepare(const CloudType::ConstPtr &source, const CloudType::ConstPtr &target,
                              double requested_leaf, double requested_radius,
                              const Checkpoint &checkpoint = {},
                              const CloudType::Ptr &cached_target = CloudType::Ptr(),
                              double cached_leaf = 0., std::size_t budget = kPointBudget) {
    if (!std::isfinite(requested_leaf) || requested_leaf <= 0. ||
        !std::isfinite(requested_radius) || requested_radius <= 0. || budget < 32)
        throw std::invalid_argument("Invalid global feature bounds");
    const auto source_bounds = bounds(source, checkpoint), target_bounds = bounds(target, checkpoint);
    double leaf = cached_target ? std::max(requested_leaf, cached_leaf) : requested_leaf;
    for (unsigned iteration = 0; iteration < 48; ++iteration) {
        check(checkpoint);
        // PCL otherwise silently returns the unfiltered input on 32-bit voxel
        // index overflow, defeating both the point budget and cancellation.
        if (!gridFits(source_bounds, leaf) || !gridFits(target_bounds, leaf)) {
            leaf *= 2.;
            continue;
        }
        auto sparse_source = voxelize(source, leaf, checkpoint);
        auto sparse_target = cached_target && leaf == cached_leaf ? cached_target : voxelize(target, leaf, checkpoint);
        const auto count = std::max(sparse_source->size(), sparse_target->size());
        if (count <= budget) {
            // Preserve the original configured descriptor for inputs already
            // within budget. Coarser proposals need a compatible neighborhood.
            const double radius = leaf > requested_leaf ? std::max(requested_radius, 3. * leaf) : requested_radius;
            return {sparse_source, sparse_target, leaf, radius};
        }
        // Conservative 3D density estimate; repeat for planar/linear maps.
        leaf *= std::max(1.2, std::min(2., std::cbrt(double(count) / budget) * 1.05));
    }
    throw std::runtime_error("Global feature spatial budget could not be met");
}

inline void compute(const CloudType::ConstPtr &cloud, Features &features, double radius,
                    const Checkpoint &checkpoint = {}) {
    if (!cloud || cloud->empty() || cloud->size() > kPointBudget ||
        !std::isfinite(radius) || radius <= 0.)
        throw std::invalid_argument("Invalid bounded feature input");
    check(checkpoint);
    const unsigned threads = std::max(1, std::min(4, omp_get_num_procs()));
    typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>(false));
    tree->setInputCloud(cloud);
    check(checkpoint);
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimator(threads);
    normal_estimator.setRadiusSearch(radius * .5);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(cloud->size());
    for (std::size_t begin = 0; begin < cloud->size(); begin += kFeatureBatch) {
        check(checkpoint);
        const auto end = std::min(cloud->size(), begin + kFeatureBatch);
        pcl::IndicesPtr indices(new pcl::Indices(end - begin));
        std::iota(indices->begin(), indices->end(), static_cast<int>(begin));
        normal_estimator.setIndices(indices);
        pcl::PointCloud<pcl::Normal> block;
        normal_estimator.compute(block);
        if (block.size() != end - begin) throw std::runtime_error("Incomplete feature normals");
        std::copy(block.begin(), block.end(), normals->begin() + begin);
    }
    check(checkpoint);
    const auto count = cloud->size();
    std::vector<unsigned char> valid(count, 0);
    for (std::size_t i = 0; i < count; ++i)
        valid[i] = std::isfinite(normals->points[i].normal_x) && std::isfinite(normals->points[i].normal_y) &&
                   std::isfinite(normals->points[i].normal_z);
    Eigen::MatrixXf hist1 = Eigen::MatrixXf::Zero(count, 11);
    Eigen::MatrixXf hist2 = Eigen::MatrixXf::Zero(count, 11);
    Eigen::MatrixXf hist3 = Eigen::MatrixXf::Zero(count, 11);
    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> estimator;
    for (std::size_t begin = 0; begin < count; begin += kFeatureBatch) {
        check(checkpoint);  // Only outside OpenMP: cancellation exceptions propagate safely.
        const auto end = std::min(count, begin + kFeatureBatch);
        #pragma omp parallel for num_threads(threads) schedule(static)
        for (std::ptrdiff_t i = begin; i < static_cast<std::ptrdiff_t>(end); ++i) {
            if (!valid[i]) continue;
            pcl::Indices neighbors;
            std::vector<float> distances;
            tree->radiusSearch(i, radius, neighbors, distances);
            neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                [&](int index) { return !valid[index]; }), neighbors.end());
            if (neighbors.size() > 1)
                estimator.computePointSPFHSignature(*cloud, *normals, i, i, neighbors, hist1, hist2, hist3);
        }
    }
    Features output;
    output.header = cloud->header;
    output.resize(count);
    output.is_dense = std::all_of(valid.begin(), valid.end(), [](unsigned char v) { return v != 0; });
    for (std::size_t begin = 0; begin < count; begin += kFeatureBatch) {
        check(checkpoint);
        const auto end = std::min(count, begin + kFeatureBatch);
        #pragma omp parallel for num_threads(threads) schedule(static)
        for (std::ptrdiff_t i = begin; i < static_cast<std::ptrdiff_t>(end); ++i) {
            if (!valid[i]) {
                std::fill_n(output[i].histogram, 33, std::numeric_limits<float>::quiet_NaN());
                continue;
            }
            pcl::Indices neighbors;
            std::vector<float> distances;
            tree->radiusSearch(i, radius, neighbors, distances);
            // Preserve the radius-search order; PCL's weighted sums then remain
            // numerically identical whenever its normals are all valid.
            std::size_t kept = 0;
            for (std::size_t j = 0; j < neighbors.size(); ++j)
                if (valid[neighbors[j]]) {
                    neighbors[kept] = neighbors[j]; distances[kept++] = distances[j];
                }
            neighbors.resize(kept); distances.resize(kept);
            Eigen::VectorXf histogram = Eigen::VectorXf::Zero(33);
            estimator.weightPointSPFHSignature(hist1, hist2, hist3, neighbors, distances, histogram);
            std::copy_n(histogram.data(), 33, output[i].histogram);
        }
    }
    check(checkpoint);
    features.swap(output);  // Cancelled work never publishes partial descriptors.
}

inline void retainFiniteFeatures(CloudType::Ptr &cloud, Features::Ptr &features) {
    if (features->size() != cloud->size()) throw std::runtime_error("Feature/point count mismatch");
    const auto valid = [](const auto &feature) {
        return std::all_of(feature.histogram, feature.histogram + 33, [](float bin) { return std::isfinite(bin); });
    };
    if (std::all_of(features->begin(), features->end(), valid)) return;
    CloudType::Ptr points(new CloudType);
    Features::Ptr finite(new Features);
    points->header = cloud->header;
    finite->header = features->header;
    for (std::size_t i = 0; i < cloud->size(); ++i)
        if (valid(features->points[i])) {
            points->push_back(cloud->points[i]); finite->push_back(features->points[i]);
        }
    cloud = points; features = finite;
}
}  // namespace bounded_global

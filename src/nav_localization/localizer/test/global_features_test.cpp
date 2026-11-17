#include "../src/localizers/global_features.h"
#include <pcl/features/fpfh_omp.h>
#include <cassert>
#include <chrono>
#include <iostream>

CloudType::Ptr surface(int nx, int ny, int layers = 1) {
    CloudType::Ptr cloud(new CloudType);
    for (int layer = 0; layer < layers; ++layer)
        for (int x = 0; x < nx; ++x)
            for (int y = 0; y < ny; ++y) {
                PointType point;
                point.x = x * .21f; point.y = y * .21f;
                point.z = 3.f + layer * 3.f + .2f * std::sin(point.x) + .1f * std::cos(point.y);
                point.intensity = float(x * ny + y);
                cloud->push_back(point);
            }
    cloud->is_dense = true;
    return cloud;
}

bounded_global::Features legacy(const CloudType::Ptr &cloud, double radius) {
    pcl::NormalEstimationOMP<PointType, pcl::Normal> nest(2);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    nest.setRadiusSearch(radius * .5);
    nest.setInputCloud(cloud);
    nest.compute(*normals);
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fest(2);
    fest.setRadiusSearch(radius);
    fest.setInputCloud(cloud);
    fest.setInputNormals(normals);
    bounded_global::Features features;
    fest.compute(features);
    return features;
}

void unbudgeted_formula_and_scale_unchanged() {
    const auto map = surface(35, 30, 2), query = surface(20, 20, 2);
    auto proposal = bounded_global::prepare(query, map, .2, 1.);
    assert(proposal.voxel_size == .2 && proposal.feature_radius == 1.);
    const auto expected = legacy(proposal.target, proposal.feature_radius);
    bounded_global::Features actual;
    bounded_global::compute(proposal.target, actual, proposal.feature_radius);
    assert(expected.size() == actual.size());
    double maximum_error = 0.;
    for (std::size_t i = 0; i < expected.size(); ++i)
        for (int bin = 0; bin < 33; ++bin) {
            assert(std::isfinite(expected[i].histogram[bin]) && std::isfinite(actual[i].histogram[bin]));
            maximum_error = std::max(maximum_error, double(std::abs(expected[i].histogram[bin] - actual[i].histogram[bin])));
        }
    std::cout << "FPFH original-PCL max bin error=" << maximum_error << '\n';
    assert(maximum_error <= 1e-5);
    const auto plain = bounded_global::voxelize(map, .2, {});
    assert(plain->size() == proposal.target->size());
    for (std::size_t i = 0; i < plain->size(); ++i)
        assert(plain->points[i].getVector3fMap() == proposal.target->points[i].getVector3fMap());
}

void proposal_budget_preserves_original_registration_clouds() {
    const auto map = surface(250, 200, 3), query = surface(60, 60, 2);
    const auto original_size = map->size();
    const PointType last = map->back();
    auto proposal = bounded_global::prepare(query, map, .2, 1.);
    assert(map->size() == original_size && map->back().getVector3fMap() == last.getVector3fMap());
    assert(query->size() == 7200);
    assert(proposal.target->size() <= bounded_global::kPointBudget);
    assert(proposal.source->size() <= bounded_global::kPointBudget);
    assert(proposal.voxel_size > .2 && proposal.feature_radius >= 3. * proposal.voxel_size);
    const auto source_expected = bounded_global::voxelize(query, proposal.voxel_size, {});
    assert(source_expected->size() == proposal.source->size());
    // A cached target is reused only at its actual scale; a radius change is
    // returned explicitly for the caller's (actual leaf, radius) feature key.
    const auto cached = bounded_global::prepare(query, map, .2, 1., {}, proposal.target, proposal.voxel_size);
    assert(cached.target == proposal.target && cached.voxel_size == proposal.voxel_size);
    const auto changed = bounded_global::prepare(query, map, .2, 8., {}, proposal.target, proposal.voxel_size);
    assert(changed.feature_radius == 8. && changed.feature_radius != proposal.feature_radius);
    assert(changed.voxel_size == proposal.voxel_size);
    std::cout << "Spatial budget " << original_size << " -> " << proposal.target->size()
              << ", leaf=" << proposal.voxel_size << ", radius=" << proposal.feature_radius << '\n';
    bounded_global::Features target_features;
    const auto started = std::chrono::steady_clock::now();
    bounded_global::compute(proposal.target, target_features, proposal.feature_radius);
    assert(target_features.size() == proposal.target->size());
    for (const auto &feature : target_features)
        for (float bin : feature.histogram) assert(std::isfinite(bin));
    std::cout << "Budgeted target FPFH seconds=" <<
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() << '\n';
}

void very_large_voxel_grid_cannot_silently_return_full_map() {
    auto cloud = surface(120, 120, 2);
    PointType remote = cloud->back(); remote.x = 4000.; remote.y = 4000.; remote.z = 1000.;
    cloud->push_back(remote);
    auto proposal = bounded_global::prepare(cloud, cloud, .01, 1.);
    assert(proposal.voxel_size > .01);
    assert(proposal.source->size() <= bounded_global::kPointBudget);
    assert(proposal.target->size() <= bounded_global::kPointBudget);
    assert(cloud->size() == 28801);
}

struct Cancelled {};
void cancellation_at_each_expensive_stage_preserves_result() {
    auto cloud = surface(32, 32);
    // 1024 points: normal checkpoints 3..10, SPFH 12..19, FPFH 20..27.
    for (int stop : {1, 5, 15, 23}) {
        bounded_global::Features output;
        output.resize(1); output[0].histogram[0] = 55.f;
        int checks = 0;
        bool cancelled = false;
        try {
            bounded_global::compute(cloud, output, 1., [&] { if (++checks == stop) throw Cancelled(); });
        } catch (const Cancelled &) { cancelled = true; }
        assert(cancelled && checks == stop);
        assert(output.size() == 1 && output[0].histogram[0] == 55.f);
    }
    int checks = 0;
    bool cancelled = false;
    try {
        bounded_global::prepare(cloud, cloud, .2, 1., [&] { if (++checks == 3) throw Cancelled(); });
    } catch (const Cancelled &) { cancelled = true; }
    assert(cancelled && checks == 3 && cloud->size() == 1024);
}

void invalid_normals_never_reach_matcher_as_nan_features() {
    auto cloud = surface(20, 20);
    PointType isolated; isolated.x = 100.; isolated.y = 100.; isolated.z = 100.; isolated.intensity = 999.;
    cloud->push_back(isolated);
    bounded_global::Features::Ptr features(new bounded_global::Features);
    bounded_global::compute(cloud, *features, 1.);
    assert(features->size() == cloud->size());
    bounded_global::retainFiniteFeatures(cloud, features);
    assert(cloud->size() == 400 && features->size() == 400);
    for (const auto &feature : *features)
        for (float bin : feature.histogram) assert(std::isfinite(bin));
}

int main() {
    unbudgeted_formula_and_scale_unchanged();
    proposal_budget_preserves_original_registration_clouds();
    very_large_voxel_grid_cannot_silently_return_full_map();
    cancellation_at_each_expensive_stage_preserves_result();
    invalid_normals_never_reach_matcher_as_nan_features();
    std::cout << "global_features: five test groups passed\n";
}

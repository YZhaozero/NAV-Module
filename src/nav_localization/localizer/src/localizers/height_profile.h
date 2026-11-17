#pragma once

#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace height_profile {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Band = std::array<float, 2>;
struct Peak {
  float z = 0; int support = 0;
  std::set<std::pair<int, int>> footprint;
  size_t family = 0;
};
struct Profile {
  std::vector<Peak> horizontal;
  bool has_floor = false;
  float floor = 0;
  float ceiling = 0;
  bool has_ceiling = false;
  bool complete = true;
  float upper = 0;
};

inline float quantile(std::vector<float> values, float q) {
  if (values.empty()) return 0;
  const size_t n = static_cast<size_t>(q * (values.size() - 1));
  std::nth_element(values.begin(), values.begin() + n, values.end());
  return values[n];
}

// The caller aligns gravity. Voxel representatives reduce dependence on raw
// scan rate and fused-map point density; map profiles are built once per load.
inline Profile describe(const Cloud& input) {
  Profile result;
  if (input.size() < 20) return result;
  Cloud::Ptr sample(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(input.makeShared());
  filter.setLeafSize(.3f, .3f, .3f);
  filter.filter(*sample);
  if (sample->size() < 20) return result;
  std::vector<float> heights;
  heights.reserve(sample->size());
  for (const auto& p : *sample) heights.push_back(p.z);
  result.upper = quantile(heights, .99f);
  const float low_limit = quantile(heights, .3f);
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(sample);
  std::vector<int> indices(20);
  std::vector<float> distances(20);
  std::map<int, std::vector<float>> bins;
  std::vector<Eigen::Vector3f> horizontal_points;
  for (const auto& p : *sample) {
    if (tree.nearestKSearch(p, 20, indices, distances) != 20 || distances.back() > 2.25f) continue;
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int i : indices) mean += (*sample)[i].getVector3fMap();
    mean /= 20;
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int i : indices) {
      const Eigen::Vector3f d = (*sample)[i].getVector3fMap() - mean;
      covariance.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1e-6f ||
        solver.eigenvalues()[0] / solver.eigenvalues()[1] >= .2f ||
        std::abs(solver.eigenvectors()(2, 0)) < .9f) continue;
    bins[static_cast<int>(std::floor(p.z / .1f))].push_back(p.z);
    horizontal_points.push_back(p.getVector3fMap());
  }
  std::vector<Peak> peaks;
  for (const auto& b : bins) {
    std::vector<float> nearby;
    for (int k = b.first - 1; k <= b.first + 1; ++k) {
      const auto it = bins.find(k);
      if (it != bins.end()) nearby.insert(nearby.end(), it->second.begin(), it->second.end());
    }
    if (nearby.size() >= 40) {
      Peak peak; peak.z=quantile(nearby,.5f);peak.support=static_cast<int>(nearby.size());
      peaks.push_back(std::move(peak));
    }
  }
  std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) {
    return a.support != b.support ? a.support > b.support : a.z < b.z;
  });
  for (const auto& p : peaks) {
    if (std::any_of(result.horizontal.begin(), result.horizontal.end(), [&](const Peak& old) {
          return std::abs(old.z - p.z) < .4f;
        })) continue;
    if (!peaks.empty() && p.support < .15f * peaks.front().support) continue;
    if (result.horizontal.size() < 8) result.horizontal.push_back(p);
    else result.complete = false;
  }
  // Uneven ground occupies adjacent XY patches at slightly different heights.
  // Stacked floors occupy the SAME XY footprint. Do not group the latter.
  for (size_t i=0;i<result.horizontal.size();++i) {
    auto& peak=result.horizontal[i];peak.family=i;
    // Use the profile's voxel scale. A metre-wide XY cell can cover two
    // different heights on a continuous slope and falsely imply stacked floors.
    for(const auto& p:horizontal_points) if(std::abs(p.z()-peak.z)<.15f)
      peak.footprint.emplace(static_cast<int>(std::floor(p.x()/.3f)),
                             static_cast<int>(std::floor(p.y()/.3f)));
    for(size_t family=0;family<i;++family) {
      bool compatible=true, found=false;
      for(size_t j=0;j<i;++j) if(result.horizontal[j].family==family) {
        found=true;const auto& other=result.horizontal[j];size_t overlap=0;
        for(const auto& cell:peak.footprint) overlap+=other.footprint.count(cell);
        const size_t smaller=std::min(peak.footprint.size(),other.footprint.size());
        if(std::abs(peak.z-other.z)>1.f || !smaller || float(overlap)/smaller>.2f) compatible=false;
      }
      if(found && compatible){peak.family=family;break;}
    }
  }
  for (const auto& p : result.horizontal) {
    if (p.z <= low_limit + .15f) {
      result.floor = p.z;
      result.has_floor = true;
      for (const auto& top : result.horizontal) {
        if (top.z - p.z >= 1.8f && top.z - p.z <= 6.f && top.support >= .3f * p.support) {
          result.ceiling = top.z;
          result.has_ceiling = true;
          break;
        }
      }
      break;
    }
  }
  return result;
}

inline std::vector<Band> layers(const Profile& p, bool normalized) {
  if (!normalized) return {{-.5f, 2.f}, {2.f, 8.f}, {8.f, 35.f}};
  if (p.has_ceiling) {
    const float step = (p.ceiling - p.floor - .6f) / 4.f;
    std::vector<Band> result;
    for (int i = 0; i < 4; ++i) result.push_back({.3f + i * step, .3f + (i + 1) * step});
    return result;
  }
  return {{.3f, 3.f}, {3.f, 10.f}, {10.f, std::clamp(p.upper - p.floor, 12.f, 45.f)}};
}
}  // namespace height_profile

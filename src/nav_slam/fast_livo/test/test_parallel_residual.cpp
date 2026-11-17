#include "voxel_map.h"
#include <chrono>
#include <iostream>
#include <stdexcept>

static void require(bool value) {
  if (!value) throw std::runtime_error("parallel residual result differs from serial reference");
}

int main() {
  VoxelMapManager manager;
  manager.config_setting_ = {};
  manager.config_setting_.max_layer_ = 0;
  manager.config_setting_.max_voxel_size_ = 10.;
  manager.config_setting_.sigma_num_ = 3.;
  auto tree = std::make_unique<VoxelOctoTree>(0, 0, 5, 100, .01);
  tree->quater_length_ = 2.5;
  for (auto &value : tree->voxel_center_) value = 5.;
  auto &plane = *tree->plane_ptr_;
  plane.is_plane_ = true;
  plane.normal_ = Eigen::Vector3d(0., 0., 1.);
  plane.center_ = Eigen::Vector3d(5., 5., 1.);
  plane.d_ = -1.;
  plane.radius_ = 20.;
  plane.plane_var_.setIdentity();
  plane.plane_var_ *= .001;
  manager.voxel_map_[VOXEL_LOCATION(0, 0, 0)] = tree.get();
  std::vector<pointWithVar> input(4097);
  std::vector<PointToPlane> reference;
  for (size_t i = 0; i < input.size(); ++i) {
    auto &p = input[i];
    p.point_b = p.point_w = Eigen::Vector3d(1. + (i % 80) * .1, 2., 1. + (i % 3) * .01);
    p.var = p.body_var = Eigen::Matrix3d::Identity() * .001;
    // Interleave successes, points with no voxel and failed plane matches,
    // including adjacent flags that vector<bool> stores in the same word.
    if (i % 7 == 0) p.point_b = p.point_w = Eigen::Vector3d(20., 20., 20.);
    else if (i % 11 == 0) p.point_w.z() = 8.;
    if (i % 7 == 0) continue;
    bool matched = false;
    double probability = 0.;
    PointToPlane result;
    manager.build_single_residual(p, tree.get(), 0, matched, probability, result);
    if (matched) reference.push_back(result);
  }
  std::vector<PointToPlane> actual;
  auto start = std::chrono::steady_clock::now();
  for (int repeat = 0; repeat < 100; ++repeat) {
    manager.BuildResidualListOMP(input, actual);
    require(actual.size() == reference.size());
    for (size_t i = 0; i < actual.size(); ++i) {
      require(actual[i].point_w_.isApprox(reference[i].point_w_, 1e-12));
      require(actual[i].normal_.isApprox(reference[i].normal_, 1e-12));
      require(actual[i].plane_var_.isApprox(reference[i].plane_var_, 1e-12));
      require(actual[i].dis_to_plane_ == reference[i].dis_to_plane_);
    }
  }
  std::cout << "100 passes, " << actual.size() << " ordered matches, seconds="
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << '\n';
  // A single plane is weak geometry. Adaptive queries must fall back before
  // updating the filter and retain full coverage for five wall-clock seconds.
  const auto original_input = input;
  manager.adaptive_query_fraction_ = .5;
  manager.BuildAdaptiveResidualList(input, actual, 10.);
  require(actual.size() == reference.size());
  require(manager.last_match_query_count_ == input.size());
  require(manager.adaptive_full_query_fallbacks_ == 1);
  manager.BuildAdaptiveResidualList(input, actual, 14.9);
  require(actual.size() == reference.size());
  require(manager.adaptive_full_query_fallbacks_ == 1);
  manager.BuildAdaptiveResidualList(input, actual, 15.1);
  require(actual.size() == reference.size());
  require(manager.adaptive_full_query_fallbacks_ == 2);
  require(input.size() == original_input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    require(input[i].point_b == original_input[i].point_b);
    require(input[i].point_w == original_input[i].point_w);
    require(input[i].var == original_input[i].var);
  }
  input.clear();
  manager.BuildResidualListOMP(input, actual);
  require(actual.empty());
  // Reuse retained scratch after an empty scan, and after a failed match.
  // No validity bit or successful residual from an earlier scan may survive.
  input.resize(3);
  for (auto &p : input) {
    p.point_b = p.point_w = Eigen::Vector3d(2., 2., 1.);
    p.var = p.body_var = Eigen::Matrix3d::Identity() * .001;
  }
  manager.BuildResidualListOMP(input, actual);
  require(actual.size() == 3);
  for (auto &p : input) p.point_w = Eigen::Vector3d(20., 20., 20.);
  manager.BuildResidualListOMP(input, actual);
  require(actual.empty());

  // A failed root match near the positive edge must try the root on its
  // positive side. At voxel size 10, comparing index 1.8 to center 15 metres
  // previously selected the negative neighbour on all three axes instead.
  manager.voxel_map_.clear();
  auto source = std::make_unique<VoxelOctoTree>(0, 0, 5, 100, .01);
  auto neighbour = std::make_unique<VoxelOctoTree>(0, 0, 5, 100, .01);
  source->quater_length_ = 2.5;
  source->voxel_center_[0] = source->voxel_center_[1] = 15.;
  source->voxel_center_[2] = 5.;
  auto &neighbour_plane = *neighbour->plane_ptr_;
  neighbour_plane.is_plane_ = true;
  neighbour_plane.normal_ = Eigen::Vector3d(0., 0., 1.);
  neighbour_plane.center_ = Eigen::Vector3d(18., 15., 5.);
  neighbour_plane.d_ = -5.;
  neighbour_plane.radius_ = 20.;
  neighbour_plane.plane_var_ = Eigen::Matrix<double, 6, 6>::Identity() * .001;
  manager.voxel_map_[VOXEL_LOCATION(1, 1, 0)] = source.get();
  manager.voxel_map_[VOXEL_LOCATION(2, 1, 0)] = neighbour.get();
  input.resize(1);
  input[0].point_b = input[0].point_w = Eigen::Vector3d(18., 15., 5.);
  input[0].var = input[0].body_var = Eigen::Matrix3d::Identity() * .001;
  manager.BuildResidualListOMP(input, actual);
  require(actual.size() == 1);
  require(actual[0].point_w_.isApprox(input[0].point_w, 1e-12));

  // Lookup must use the same floor convention as insertion at -10 exactly.
  manager.voxel_map_.clear();
  manager.voxel_map_[VOXEL_LOCATION(-1, 1, 0)] = neighbour.get();
  input[0].point_b = input[0].point_w = Eigen::Vector3d(-10., 15., 5.);
  manager.BuildResidualListOMP(input, actual);
  require(actual.size() == 1);
}

#include "voxel_map.h"
#include "voxel_index.h"
#include "parallelism.h"
#include <chrono>
#include <iostream>
#include <set>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

static VoxelMapConfig config() {
  VoxelMapConfig result{};
  result.max_voxel_size_ = 2.;
  result.max_layer_ = 2;
  result.max_points_num_ = 150;
  result.layer_init_num_ = {5, 5, 5};
  result.planner_threshold_ = .001;
  result.sigma_num_ = 3.;
  result.dept_err_ = .05;
  result.beam_err_ = .02;
  return result;
}

// Independent original per-point insertion path, with the corrected shared
// floor rule. It does not use the production grouping or update scratch.
static void serialReference(VoxelMapManager &manager, const std::vector<pointWithVar> &points) {
  const auto &cfg = manager.config_setting_;
  for (const auto &point : points) {
    const VOXEL_LOCATION location(
        fast_livo_voxel::voxelIndex(point.point_w.x(), cfg.max_voxel_size_),
        fast_livo_voxel::voxelIndex(point.point_w.y(), cfg.max_voxel_size_),
        fast_livo_voxel::voxelIndex(point.point_w.z(), cfg.max_voxel_size_));
    auto entry = manager.voxel_map_.find(location);
    if (entry == manager.voxel_map_.end()) {
      auto *tree = new VoxelOctoTree(cfg.max_layer_, 0, cfg.layer_init_num_[0],
                                   cfg.max_points_num_, cfg.planner_threshold_);
      tree->layer_init_num_ = convertToIntVectorSafe(cfg.layer_init_num_);
      tree->quater_length_ = cfg.max_voxel_size_ / 4.;
      tree->voxel_center_[0] = (location.x + .5) * cfg.max_voxel_size_;
      tree->voxel_center_[1] = (location.y + .5) * cfg.max_voxel_size_;
      tree->voxel_center_[2] = (location.z + .5) * cfg.max_voxel_size_;
      entry = manager.voxel_map_.emplace(location, tree).first;
    }
    entry->second->UpdateOctoTree(point);
  }
}

static void compareTree(const VoxelOctoTree &a, const VoxelOctoTree &b, std::set<int> &ids) {
  require(a.init_octo_ == b.init_octo_, "tree initialization differs");
  require(a.octo_state_ == b.octo_state_ && a.new_points_ == b.new_points_, "tree counters differ");
  require(a.update_enable_ == b.update_enable_, "tree saturation differs");
  require(a.temp_points_.size() == b.temp_points_.size(), "point count differs");
  for (size_t i = 0; i < a.temp_points_.size(); ++i) {
    require(a.temp_points_[i].point_w.isApprox(b.temp_points_[i].point_w, 1e-12), "point order differs");
    require(a.temp_points_[i].var.isApprox(b.temp_points_[i].var, 1e-12), "point covariance differs");
  }
  const auto &pa = *a.plane_ptr_;
  const auto &pb = *b.plane_ptr_;
  require(pa.is_plane_ == pb.is_plane_ && pa.is_init_ == pb.is_init_, "plane classification differs");
  require(pa.points_size_ == pb.points_size_, "plane support differs");
  require(pa.center_.isApprox(pb.center_, 1e-12), "plane center differs");
  require(pa.covariance_.isApprox(pb.covariance_, 1e-12), "plane covariance differs");
  require(pa.plane_var_.isApprox(pb.plane_var_, 1e-12), "plane uncertainty differs");
  require(pa.normal_.isApprox(pb.normal_, 1e-12), "plane normal differs");
  if (pb.is_init_) require(ids.insert(pb.id_).second, "parallel plane marker ids collide");
  if (pa.is_plane_) {
    require(pa.d_ == pb.d_ && pa.radius_ == pb.radius_, "plane distance or radius differs");
    require(pa.min_eigen_value_ == pb.min_eigen_value_ &&
            pa.mid_eigen_value_ == pb.mid_eigen_value_ &&
            pa.max_eigen_value_ == pb.max_eigen_value_, "plane eigenvalues differ");
  }
  for (int i = 0; i < 8; ++i) {
    require(bool(a.leaves_[i]) == bool(b.leaves_[i]), "octree topology differs");
    if (a.leaves_[i]) compareTree(*a.leaves_[i], *b.leaves_[i], ids);
  }
}

static void compareMaps(const VoxelMapManager &a, const VoxelMapManager &b) {
  require(a.voxel_map_.size() == b.voxel_map_.size(), "root voxel count differs");
  std::set<int> ids;
  for (const auto &root : a.voxel_map_) {
    const auto other = b.voxel_map_.find(root.first);
    require(other != b.voxel_map_.end(), "root voxel coordinate differs");
    compareTree(*root.second, *other->second, ids);
  }
  require(ids.size() > 10, "test did not initialize enough planes");
}

static void clearMap(VoxelMapManager &manager) {
  for (const auto &entry : manager.voxel_map_) delete entry.second;
  manager.voxel_map_.clear();
}

int main(int argc, char **argv) {
  const int threads = argc > 1 ? std::atoi(argv[1]) : 4;
  require(threads >= 1 && threads <= 4, "expected a thread count from 1 to 4");
  setenv("OMP_NUM_THREADS", std::to_string(threads).c_str(), 1);
  const int worker_budget = fast_livo_parallel::threadCount();
  require(worker_budget >= 1 && worker_budget <= threads, "thread budget not honored");
  omp_set_dynamic(0);
  {
    VoxelMapManager reference, actual;
    reference.config_setting_ = actual.config_setting_ = config();
    const auto start = std::chrono::steady_clock::now();
    for (int batch = 0; batch < 6; ++batch) {
      std::vector<pointWithVar> points;
      // Interleaving roots detects reordering within each root. Half the roots
      // are non-planar and split, exercising descendant creation in workers.
      for (int sample = 0; sample < 42; ++sample) {
        for (int root = 0; root < 24; ++root) {
          const int k = sample + batch * 42;
          const double x = .15 + (k % 13) * .12;
          const double y = .16 + ((k * 5) % 11) * .15;
          const double z = root % 2 == 0 ? .2 + .015 * x + .02 * y : .25 + (k % 2) * 1.3 + .01 * x;
          pointWithVar point;
          point.point_b = point.point_w = V3D((root % 6 - 3) * 2. + x, (root / 6 - 2) * 2. + y, z);
          point.var = point.body_var = M3D::Identity() * (.001 + (k % 5) * .0001);
          points.push_back(point);
        }
      }
      serialReference(reference, points);
      actual.UpdateVoxelMap(points);
      compareMaps(reference, actual);
      for (const auto &root : actual.voxel_map_)
        require(root.second->update_group_index_ == std::numeric_limits<size_t>::max(),
                "temporary root grouping index was not reset");
      // Empty scans must not replay a previous group's retained links.
      actual.UpdateVoxelMap({});
      compareMaps(reference, actual);
    }
    std::cout << worker_budget << " workers (requested " << threads
              << "): exact ordered-tree reference passed, seconds="
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << '\n';
    clearMap(reference);
    clearMap(actual);
  }

  // The exact negative boundary belongs to voxel -1, not -2, on insertion.
  VoxelMapManager boundary;
  boundary.config_setting_ = config();
  pointWithVar point;
  point.point_b = point.point_w = V3D(-2., -4., .5);
  point.var = point.body_var = M3D::Identity() * .001;
  boundary.UpdateVoxelMap({point});
  require(boundary.voxel_map_.count(VOXEL_LOCATION(-1, -2, 0)) == 1, "negative boundary assigned to wrong root");
  clearMap(boundary);
}

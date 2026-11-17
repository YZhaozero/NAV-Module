#include "voxel_map.h"
#include "adaptive_load.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

static void check(bool ok) {
  if (!ok) throw std::runtime_error("bounded query coverage/pose regression");
}

int main() {
  VoxelMapManager manager;
  manager.config_setting_ = {};
  manager.config_setting_.max_layer_ = 1;
  manager.config_setting_.max_voxel_size_ = 10.;
  manager.config_setting_.max_iterations_ = 5;
  manager.config_setting_.dept_err_ = .02;
  manager.config_setting_.beam_err_ = .05;
  manager.config_setting_.sigma_num_ = 3.;
  manager.extR_.setIdentity(); manager.extT_.setZero();
  auto tree = std::make_unique<VoxelOctoTree>(1, 0, 5, 100, .01);
  tree->quater_length_ = 2.5;
  for (auto &v : tree->voxel_center_) v = 5.;
  for (int axis=0; axis<3; ++axis) {
    tree->leaves_[axis] = new VoxelOctoTree(1, 1, 5, 100, .01);
    auto &plane = *tree->leaves_[axis]->plane_ptr_;
    plane.is_plane_ = true; plane.normal_ = Eigen::Vector3d::Unit(axis);
    plane.center_ = Eigen::Vector3d::Constant(5.); plane.d_ = -5.;
    plane.radius_ = 20.; plane.plane_var_ = Eigen::Matrix<double,6,6>::Identity()*.0001;
  }
  manager.voxel_map_[VOXEL_LOCATION(0,0,0)] = tree.get();
  manager.feats_down_body_.reset(new PointCloudXYZI());
  manager.feats_undistort_ = manager.feats_down_body_;
  std::cout << "points,budget,fraction,queries,median_ms,max_ms,position_difference_m,rotation_difference_rad\n";
  // Same geometry at increasing density, then return to the sparse scene to
  // exercise scratch reuse. Timings are reported, not flaky CI speed gates.
  for (const int count : {3000, 9000, 18000, 36000, 3000}) {
    manager.feats_down_body_->clear();
    for (int i=0; i<count; ++i) {
      pcl::PointXYZINormal p{};
      p.x=1.+(i%79)*.1; p.y=1.2+(i%67)*.1; p.z=1.4+(i%61)*.1;
      p.data[(i/5)%3]=5.;
      if (i%17==0) p.x=p.y=p.z=20.;
      manager.feats_down_body_->push_back(p);
    }
    manager.feats_down_size_ = count;
    StatesGroup reference;
    for (const int budget : {0, 4000}) {
      for (const double fraction : {1., .25}) {
        manager.adaptive_query_budget_ = budget;
        manager.adaptive_query_fraction_ = fraction;
        manager.adaptive_full_query_until_ = -1.;
        manager.adaptive_full_query_fallbacks_ = 0;
        std::vector<double> times;
        for (int repeat=0; repeat<9; ++repeat) {
          StatesGroup prior;
          prior.pos_end = Eigen::Vector3d(.02,-.03,.01);
          prior.rot_end = Eigen::AngleAxisd(.003,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
          manager.state_ = prior;
          const auto begin=std::chrono::steady_clock::now();
          manager.StateEstimation(prior);
          if (repeat) times.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
          check(manager.state_.cov.allFinite());
          check(manager.pv_list_.size()==static_cast<std::size_t>(count));
          check(manager.adaptive_full_query_fallbacks_==0);
          check(manager.last_match_query_count_==fast_livo_schedule::matchQueryCount(count,fraction,budget));
          for (int i=0; i<count; ++i) {
            check(manager.pv_list_[i].var.allFinite());
            check(manager.pv_list_[i].point_w.allFinite());
            check(manager.pv_list_[i].point_b.x()==manager.feats_down_body_->points[i].x);
          }
        }
        if (budget==0 && fraction==1.) reference=manager.state_;
        const double position_error=(manager.state_.pos_end-reference.pos_end).norm();
        const double rotation_error=Eigen::AngleAxisd(reference.rot_end.transpose()*manager.state_.rot_end).angle();
        check(position_error<.01 && rotation_error<.005);
        std::sort(times.begin(),times.end());
        std::cout << count << ',' << budget << ',' << fraction << ',' << manager.last_match_query_count_
                  << ',' << (times[3]+times[4])*.5 << ',' << times.back()
                  << ',' << position_error << ',' << rotation_error << '\n';
      }
    }
  }
}

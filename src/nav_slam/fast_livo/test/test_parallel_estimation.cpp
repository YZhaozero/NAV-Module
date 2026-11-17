#include "voxel_map.h"
#include "adaptive_load.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>

static void require(bool value) {
  if (!value) throw std::runtime_error("point-parallel state estimation regression");
}

int main(int argc, char **argv) {
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
  for (auto &value : tree->voxel_center_) value = 5.;
  for (int axis=0; axis<3; ++axis) {
    tree->leaves_[axis] = new VoxelOctoTree(1, 1, 5, 100, .01);
    auto &plane = *tree->leaves_[axis]->plane_ptr_;
    plane.is_plane_ = true; plane.normal_ = Eigen::Vector3d::Unit(axis);
    plane.center_ = Eigen::Vector3d::Constant(5.); plane.d_ = -5.;
    plane.radius_ = 20.; plane.plane_var_.setIdentity(); plane.plane_var_ *= .0001;
  }
  manager.voxel_map_[VOXEL_LOCATION(0,0,0)] = tree.get();
  manager.feats_down_body_.reset(new PointCloudXYZI());
  for (int i=0; i<6003; ++i) {
    pcl::PointXYZINormal p{};
    p.x = 1. + (i%79)*.1; p.y = 1.2 + (i%67)*.1; p.z = 1.4 + (i%61)*.1;
    // Five-point blocks retain all three normals under 1/3 and 1/4 sampling.
    p.data[(i/5)%3] = 5.;
    if (i%17 == 0) p.x = p.y = p.z = 20.; // no matching voxel
    manager.feats_down_body_->push_back(p);
  }
  manager.feats_undistort_ = manager.feats_down_body_;
  manager.feats_down_size_ = manager.feats_down_body_->size();
  StatesGroup first;
  for (int pass=0; pass<20; ++pass) {
    StatesGroup propagated;
    propagated.pos_end = Eigen::Vector3d(.02,-.03,.01);
    propagated.rot_end = Eigen::AngleAxisd(.003,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
    manager.state_ = propagated;
    manager.StateEstimation(propagated);
    require(manager.state_.cov.allFinite() && manager.state_.pos_end.allFinite());
    require(manager.effct_feat_num_ > 2000);
    require(manager.state_.pos_end.norm() < 2.*propagated.pos_end.norm());
    if (pass == 0) first=manager.state_;
    else {
      require((manager.state_-first).norm() < 1e-12);
      require((manager.state_.cov-first.cov).norm() < 1e-12);
    }
  }
  if (argc == 2) {
    std::ofstream out(argv[1]); out << std::setprecision(17);
    out << manager.state_.rot_end << '\n' << manager.state_.pos_end << '\n' << manager.state_.cov << '\n';
    for (const auto &p : manager.pv_list_) out << p.point_w.transpose() << ' ' << p.var << '\n';
    require(bool(out));
  }
  // Reducing point-to-plane queries must not reduce the cloud used for map
  // insertion and geometric archive output, including the sparse-map path.
  const auto original_cloud = *manager.feats_down_body_;
  for (double fraction : {.75, .5, 1. / 3., .25}) {
  manager.adaptive_query_fraction_ = fraction;
  StatesGroup propagated;
  propagated.pos_end = Eigen::Vector3d(.02,-.03,.01);
  propagated.rot_end = Eigen::AngleAxisd(.003,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
  manager.state_ = propagated;
  manager.StateEstimation(propagated);
  const auto expected_queries = fast_livo_schedule::matchQueryCount(original_cloud.size(), fraction);
  require(manager.last_match_query_count_ == expected_queries);
  require(expected_queries > 1000 && expected_queries < original_cloud.size());
  require(manager.adaptive_full_query_fallbacks_ == 0);
  require(manager.effct_feat_num_ >= 800);
  require(manager.pv_list_.size() == original_cloud.size());
  require(manager.cross_mat_list_.size() == original_cloud.size());
  require(manager.body_cov_list_.size() == original_cloud.size());
  require(manager.feats_down_body_->size() == original_cloud.size());
  require(manager.state_.cov.allFinite() && manager.state_.pos_end.allFinite());
  require((manager.state_.pos_end - first.pos_end).norm() < .01);
  for (size_t i = 0; i < original_cloud.size(); ++i) {
    require(manager.pv_list_[i].point_b.x() == original_cloud[i].x);
    require(manager.pv_list_[i].point_b.y() == original_cloud[i].y);
    require(manager.pv_list_[i].point_b.z() == original_cloud[i].z);
    require(manager.pv_list_[i].var.allFinite());
    require(manager.feats_down_body_->points[i].x == original_cloud[i].x);
  }
  }
  manager.adaptive_query_fraction_ = .5;
  // Sampling indices may correlate with spatial/normal ordering. Here all
  // even points constrain Z and all odd points alternate X/Y. Half sampling
  // selects odd points: it must not silently remove the sole Z constraints.
  std::vector<pointWithVar> correlated(2400);
  for (size_t i = 0; i < correlated.size(); ++i) {
    auto &p = correlated[i];
    p.point_b = p.point_w = Eigen::Vector3d(2., 2., 2.);
    const int axis = i % 2 == 0 ? 2 : ((i / 2) % 2);
    p.point_b[axis] = p.point_w[axis] = 5.;
    p.var = p.body_var = Eigen::Matrix3d::Identity() * .001;
  }
  std::vector<PointToPlane> constrained;
  manager.BuildAdaptiveResidualList(correlated, constrained, 100.);
  require(manager.last_match_query_count_ == correlated.size());
  require(manager.adaptive_full_query_fallbacks_ == 1);
  require(constrained.size() == correlated.size());
  Eigen::Matrix3d normals = Eigen::Matrix3d::Zero();
  for (const auto &p : constrained) normals.noalias() += p.normal_ * p.normal_.transpose();
  require(normals(0,0) == 600. && normals(1,1) == 600. && normals(2,2) == 1200.);
  // The root recursively owns its child trees.
}

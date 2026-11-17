#include "bounded_buffers.h"
#include "vio.h"
#include "preprocess.h"
#include <deque>
#include <numeric>
#include <stdexcept>

static void require(bool okay) { if (!okay) throw std::runtime_error("memory regression failed"); }

static Feature *observation(VisualPoint *point, int frame, const cv::Mat &image) {
  auto *feature = new Feature(point, new float[64](), V2D(32,32), V3D(0,0,1), SE3<double>(), 0);
  feature->id_ = frame;
  feature->img_ = image;
  point->addFrameRef(feature);
  return feature;
}

static void test_buffers() {
  std::deque<int> values, times;
  for (int i=0; i<10000; ++i) {
    values.push_back(i); times.push_back(i*2);
    fast_livo_memory::trimPaired(values,times,8);
    require(values.size()<=8 && values.size()==times.size());
    require(values.front()*2==times.front());
  }
  // The frame chosen for LIO stays alive when the next callback trims its queue.
  std::deque<cv::Mat> images;
  std::deque<double> stamps;
  images.emplace_back(8,8,CV_8U,cv::Scalar(42)); stamps.push_back(1.0);
  cv::Mat paired=images.front(); const double paired_time=stamps.front();
  images.pop_front(); stamps.pop_front();
  for (int i=0;i<30;++i) {
    images.emplace_back(8,8,CV_8U,cv::Scalar(i)); stamps.push_back(i+2.0);
    fast_livo_memory::trimPaired(images,stamps,8);
  }
  require(paired_time==1.0 && paired.at<unsigned char>(0,0)==42);

  std::vector<int> buffer, input(2505), persisted;
  std::iota(input.begin(),input.end(),0);
  int chunks=0;
  auto sink=[&](const auto &chunk) {
    require(chunk.size()<=1000); ++chunks;
    persisted.insert(persisted.end(),chunk.begin(),chunk.end());
  };
  fast_livo_memory::appendCloud(buffer,input,1000,sink);
  require(buffer.size()==505 && chunks==2);
  sink(buffer); std::vector<int>().swap(buffer);
  require(persisted==input && buffer.capacity()==0);
  bool failed=false;
  try {
    fast_livo_memory::appendCloud(buffer,input,1000,[](const auto &) { throw std::runtime_error("disk full"); });
  } catch (const std::runtime_error &) { failed=true; }
  require(failed && buffer.size()==1000); // never discard a failed write
}

static size_t visual_age_case(int age) {
  VIOManager manager;
  manager.max_visual_frames=age;
  for (int frame=0;frame<1000;++frame) {
    for(auto *point:manager.points_pending_delete) delete point;
    manager.points_pending_delete.clear();
    auto *point=new VisualPoint(V3D(frame,0,5));
    cv::Mat image(64,64,CV_8U,cv::Scalar(42));
    auto *feature=observation(point,frame,image);
    point->ref_patch=feature; point->has_ref_patch_=true;
    manager.insertPointIntoVoxelMap(point);
    manager.evictStaleVisualPoints(frame);
    require(manager.feat_map.size()<=(age ? 11 : 1000));
    require(manager.points_pending_delete.size()<=1);
  }
  const auto retained = manager.feat_map.size();
  manager.evictStaleVisualPoints(2000);
  require(age ? manager.feat_map.empty() : retained == 1000);
  // Pending deletions are intentionally left for the manager destructor.
  return retained;
}

static void test_visual_age() {
  const auto disabled = visual_age_case(0);
  const auto enabled = visual_age_case(10);
  require(disabled == 1000 && enabled == 11);
  std::cout << "VISUAL_AGE_ABLATION input_frames=1000 disabled_retained=" << disabled
            << " age10_retained=" << enabled << " deferred_retained_at_most=1" << std::endl;
}

static void test_stationary_patch_ownership() {
  vk::PinholeCamera camera(64,64,1,20,20,32,32);
  StatesGroup state;
  cv::Mat image(64,64,CV_8U,cv::Scalar(42));
  VIOManager manager;
  manager.cam=&camera; manager.state=&state;
  manager.width=64; manager.height=64;
  manager.patch_size=8; manager.patch_size_total=64; manager.patch_size_half=4;
  manager.new_frame_.reset(new Frame(&camera,image));
  manager.visual_submap=new SubSparseMap;
  auto *point=new VisualPoint(V3D(0,0,5));
  observation(point,0,image);
  manager.insertPointIntoVoxelMap(point);
  manager.visual_submap->voxel_points.push_back(point);
  manager.visual_submap->search_levels.push_back(0);
  manager.total_points=1; manager.update_flag.resize(1);
  for (int i=0;i<1000;++i) manager.updateVisualMapPoints(image);
  require(point->obs_.size()==1);
  // A moving observation must still be retained and released with the point.
  manager.new_frame_->T_f_w_=SE3<double>(M3D::Identity(),V3D(0,0,1));
  manager.updateVisualMapPoints(image);
  require(point->obs_.size()==2);
}

static void test_blind_range() {
  Preprocess pre;
  require(std::abs(pre.blind_sqr - 0.0001) < 1e-9);
  // Mirror ROS parameter loading: the public blind field changes without set().
  pre.blind = 0.5;
  auto message = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  for (float distance : {0.0f, 0.2f, 0.6f, 1.0f}) {
    livox_ros_driver2::msg::CustomPoint point;
    point.x = distance; point.line = 0; point.tag = 0x10;
    message->points.push_back(point);
  }
  message->point_num = message->points.size();
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  pre.process(message, output);
  require(output->size() == 2 && output->front().x == 0.6f);
  // The standard PointCloud2 path must obey a subsequently changed threshold.
  pre.lidar_type = L515;
  pre.blind = 0.8;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (float distance : {0.0f, 0.6f, 1.0f}) {
    pcl::PointXYZRGB point; point.x = distance; point.y = point.z = 0;
    cloud.push_back(point);
  }
  auto standard = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *standard);
  pre.process(standard, output);
  require(output->size() == 1 && output->front().x == 1.0f);
}

int main() {
  test_blind_range();
  test_buffers(); test_visual_age(); test_stationary_patch_ownership();
  return 0;
}

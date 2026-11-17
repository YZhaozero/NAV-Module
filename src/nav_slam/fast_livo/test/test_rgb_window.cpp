#include "LIVMapper.h"
#include <stdexcept>
#include <thread>
#include <atomic>

volatile std::sig_atomic_t g_stop_requested = 0;

int main() {
  const char *arguments[] = {"rgb_window_regression", "--ros-args", "--params-file",
      ROOT_DIR "config/mid360_live.yaml", "-p", "common.img_en:=0",
      "-p", "pcd_save.pcd_save_en:=false"};
  rclcpp::init(8, arguments);
  auto calibration = std::make_shared<rclcpp::Node>("parameter_blackboard");
  calibration->declare_parameter("cam_model", "Pinhole");
  calibration->declare_parameter("cam_width", 64);
  calibration->declare_parameter("cam_height", 64);
  calibration->declare_parameter("scale", 1.0);
  for (const auto *name : {"cam_fx", "cam_fy"}) calibration->declare_parameter(name, 40.0);
  for (const auto *name : {"cam_cx", "cam_cy"}) calibration->declare_parameter(name, 32.0);
  for (const auto *name : {"cam_d0", "cam_d1", "cam_d2", "cam_d3"}) calibration->declare_parameter(name, 0.0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(calibration);
  std::thread serving([&]() { executor.spin(); });
  {
    auto node = std::make_shared<rclcpp::Node>("rgb_test_owner");
    LIVMapper mapper(node, "rgb_window_test");
    std::unique_ptr<vk::AbstractCamera> camera(mapper.vio_manager->cam);
    mapper.slam_mode_ = LIVO;
    mapper.LidarMeasures.lio_vio_flg = VIO;
    mapper.LidarMeasures.measures.emplace_back();
    mapper.LidarMeasures.last_lio_update_time = 1.0;
    mapper.pcd_save_en = mapper.img_save_en = false;
    mapper.pub_scan_num = 1;
    mapper.vio_manager->img_rgb = cv::Mat(64, 64, CV_8UC3, cv::Scalar(20, 40, 60));
    mapper.vio_manager->new_frame_.reset(new Frame(camera.get(), cv::Mat(64, 64, CV_8UC1, cv::Scalar(40))));
    mapper.vio_manager->new_frame_->T_f_w_ = SE3<double>();
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rgb_window_test", 1);
    std::atomic<size_t> visible_count{0};
    auto subscriber=node->create_subscription<sensor_msgs::msg::PointCloud2>("/rgb_window_test",10,
        [&](const sensor_msgs::msg::PointCloud2 &msg){if(msg.width)visible_count=msg.width;});
    executor.add_node(node);
    // All 100,000 measurements are behind the camera: no RGB output, but the
    // full-field geometry is handled independently by NavigationOutput.
    for (int frame = 0; frame < 1000; ++frame) {
      PointType point; point.x = point.y = 0; point.z = -5;
      mapper.pcl_w_wait_pub->assign(100, point);
      mapper.publish_frame_world(publisher, mapper.vio_manager);
      if (!mapper.pcl_wait_pub->empty() || !mapper.pcl_w_wait_pub->empty())
        throw std::runtime_error("RGB window retained invisible scans");
    }
    // Visible points still pass the real projection/publication path.
    PointType point; point.x = point.y = 0; point.z = 5;
    mapper.pcl_w_wait_pub->push_back(point);
    mapper.publish_frame_world(publisher, mapper.vio_manager);
    if (!mapper.pcl_wait_pub->empty()) throw std::runtime_error("RGB window not consumed");
    // Three LIO updates before one VIO update: all three must be coloured.
    for(int frame=0;frame<3;++frame) {
      mapper.LidarMeasures.lio_vio_flg=LIO;
      mapper.LidarMeasures.last_lio_update_time=2.+frame*.1;
      point.x=(frame-1)*.1;point.z=5;
      mapper.pcl_w_wait_pub->assign(1,point);
      mapper.publish_frame_world(publisher,mapper.vio_manager);
    }
    mapper.LidarMeasures.lio_vio_flg=VIO;
    // A display/storage worker may still own this immutable LIO snapshot when
    // the visual window is consumed. Releasing our pointer must not clear it.
    PointCloudXYZI::ConstPtr published_snapshot = mapper.pcl_w_wait_pub;
    const auto published_size = published_snapshot->size();
    mapper.publish_frame_world(publisher,mapper.vio_manager);
    if (published_snapshot->size() != published_size || mapper.pcl_w_wait_pub == published_snapshot)
      throw std::runtime_error("Colour window mutated an asynchronously published scan");
    for(int n=0;n<100&&visible_count!=3;++n)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if(visible_count!=3)throw std::runtime_error("Visual cadence discarded intermediate LIO slices");
    executor.remove_node(node);
  }
  executor.cancel();
  serving.join();
  rclcpp::shutdown();
  return 0;
}

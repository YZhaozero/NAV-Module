#include "LIVMapper.h"
#include <atomic>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

volatile std::sig_atomic_t g_stop_requested = 0;

static void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

template <class Predicate> static void waitFor(Predicate predicate, const char *message) {
  for (int n = 0; n < 500; ++n) {
    if (predicate()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error(message);
}

int main() {
  const char *arguments[] = {"async_rgb_mapper_regression", "--ros-args", "--params-file",
      ROOT_DIR "config/mid360_live.yaml", "-p", "common.img_en:=0",
      "-p", "pcd_save.pcd_save_en:=false", "-p", "navigation.enabled:=true"};
  rclcpp::init(10, arguments);
  auto calibration = std::make_shared<rclcpp::Node>("parameter_blackboard");
  calibration->declare_parameter("cam_model", "Pinhole");
  calibration->declare_parameter("cam_width", 64);
  calibration->declare_parameter("cam_height", 64);
  calibration->declare_parameter("scale", 1.0);
  for (const auto *name : {"cam_fx", "cam_fy"}) calibration->declare_parameter(name, 40.0);
  for (const auto *name : {"cam_cx", "cam_cy"}) calibration->declare_parameter(name, 32.0);
  for (const auto *name : {"cam_d0", "cam_d1", "cam_d2", "cam_d3"})
    calibration->declare_parameter(name, 0.0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(calibration);
  std::thread serving([&] { executor.spin(); });
  int result = 0;
  try {
    auto owner = std::make_shared<rclcpp::Node>("async_rgb_test_owner");
    LIVMapper mapper(owner, "async_rgb_mapper_test");
    std::unique_ptr<vk::AbstractCamera> camera(mapper.vio_manager->cam);
    mapper.slam_mode_ = LIVO;
    mapper.pcd_save_en = mapper.img_save_en = mapper.colmap_output_en = false;
    mapper.pub_scan_num = 1;
    mapper.blind_rgb_points = 0.;
    mapper.LidarMeasures.measures.emplace_back();
    mapper.vio_manager->img_rgb = cv::Mat(64, 64, CV_8UC3, cv::Scalar(20, 40, 60));
    mapper.vio_manager->img_cp = cv::Mat(64, 64, CV_8UC3, cv::Scalar(7, 8, 9));
    mapper.vio_manager->new_frame_.reset(
        new Frame(camera.get(), cv::Mat(64, 64, CV_8UC1, cv::Scalar(40))));
    mapper.vio_manager->new_frame_->T_f_w_ = SE3<double>();
    mapper.pubLaserCloudFullRes = owner->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/async_rgb_mapper/cloud", 1);
    image_transport::ImageTransport transport(owner);
    mapper.pubImage = transport.advertise("/async_rgb_mapper/image", 1);
    std::mutex received_mutex;
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::msg::Image image;
    auto cloud_sub = owner->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/async_rgb_mapper/cloud", 10, [&](const sensor_msgs::msg::PointCloud2 &message) {
          std::lock_guard<std::mutex> lock(received_mutex); cloud = message;
        });
    auto image_sub = owner->create_subscription<sensor_msgs::msg::Image>(
        "/async_rgb_mapper/image", 10, [&](const sensor_msgs::msg::Image &message) {
          std::lock_guard<std::mutex> lock(received_mutex); image = message;
        });
    executor.add_node(owner);
    mapper.initializeAsyncRgbPreview();
    require(mapper.asyncRgbEnabled(), "Real mapper did not enable asynchronous RGB preview");
    waitFor([&] { return mapper.pubLaserCloudFullRes->get_subscription_count() > 0 &&
                        mapper.pubImage.getNumSubscribers() > 0; }, "ROS RGB subscribers not discovered");
    for (int n = 0; n < 3; ++n) {
      mapper.LidarMeasures.lio_vio_flg = LIO;
      mapper.LidarMeasures.last_lio_update_time = 2. + n * .1;
      PointType point{}; point.x = (n - 1) * .125; point.z = 5.;
      mapper.pcl_w_wait_pub.reset(new PointCloudXYZI());
      mapper.pcl_w_wait_pub->push_back(point);
      mapper.publish_frame_world(mapper.pubLaserCloudFullRes, mapper.vio_manager);
    }
    auto retained_geometry = mapper.pcl_w_wait_pub;
    auto retained_image = mapper.vio_manager->img_cp;
    mapper.LidarMeasures.lio_vio_flg = VIO;
    mapper.LidarMeasures.measures.back().vio_time = 2.23456789;
    const auto expected_stamp = sec2Stamp(mapper.LidarMeasures.measures.back().vio_time);
    mapper.publish_frame_world(mapper.pubLaserCloudFullRes, mapper.vio_manager);
    // The producer immediately starts the next frame; worker snapshots must
    // retain the old pose/image/cloud independently of these mutable members.
    mapper.vio_manager->img_rgb = cv::Mat(64, 64, CV_8UC3, cv::Scalar(0, 0, 0));
    mapper.vio_manager->img_cp = cv::Mat(64, 64, CV_8UC3, cv::Scalar(0, 0, 0));
    mapper.vio_manager->new_frame_->T_f_w_.translation() = V3D(100, 100, 100);
    require(retained_geometry->size() == 1 && mapper.pcl_w_wait_pub != retained_geometry,
            "Real mapper mutated a cloud owned by the RGB worker");
    waitFor([&] {
      std::lock_guard<std::mutex> lock(received_mutex);
      return cloud.width == 3 && image.header.stamp == expected_stamp;
    }, "Real mapper did not publish all three LIO slices and the captured image");
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      require(cloud.header.stamp == expected_stamp, "RGB cloud used wall time instead of VIO time");
      require(cloud.header.frame_id == mapper.world_frame_, "RGB world frame changed");
      require(image.width == 64 && image.height == 64 && image.data.at(0) == 7,
              "RGB image snapshot was replaced by the next frame");
      PointCloudXYZRGB colored; pcl::fromROSMsg(cloud, colored);
      for (const auto &point : colored)
        require(point.r == 60 && point.g == 40 && point.b == 20 && point.z == 5.,
                "RGB worker used mutable camera pose or image");
    }
    // Exercise processFrame -> asynchronous publish through handleVIO itself.
    // Publication releases pcl_w_wait_pub; using that mutable output as the
    // post-cycle success test used to omit every real VIO compute sample.
    mapper.pcl_w_wait_pub.reset(new PointCloudXYZI());
    PointType cost_point{}; cost_point.z = 5.; mapper.pcl_w_wait_pub->push_back(cost_point);
    mapper.LidarMeasures.last_lio_update_time = 2.4;
    mapper.LidarMeasures.measures.back().vio_time = 2.4;
    mapper.LidarMeasures.measures.back().img = cv::Mat(64, 64, CV_8UC3, cv::Scalar(30, 40, 50));
    mapper.plot_time = -1000.; mapper.frame_cnt = 0;
    mapper.imu_prop_enable = false;
    mapper.adaptive_compute_window_s_ = 0.;
    mapper.handleVIO();
    require(mapper.pcl_w_wait_pub->empty(), "Real async VIO path did not release its output cloud");
    require(mapper.diagnostic_vio_fit_ > 0. &&
            mapper.adaptive_compute_window_s_ == mapper.diagnostic_vio_fit_,
            "Real async VIO compute was omitted from adaptive load window");
    const double measured_compute = mapper.adaptive_compute_window_s_;
    mapper.handleVIO(); // Empty input must not count the prior frame again.
    require(mapper.adaptive_compute_window_s_ == measured_compute,
            "Empty VIO cycle counted stale compute time");
    mapper.rgb_preview_->stop();
    mapper.vio_manager->new_frame_->T_f_w_ = SE3<double>();
    mapper.vio_manager->img_rgb = cv::Mat(64, 64, CV_8UC3, cv::Scalar(20, 40, 60));
    // Real fallback code must still append each research-export frame. The
    // display latest policy must never silently skip data destined for PCD.
    mapper.pcd_save_en = true; mapper.pcd_save_type = 0; mapper.pcd_save_interval = -1;
    require(!mapper.asyncRgbEnabled(), "PCD export inherited latest-wins preview");
    const auto previous = mapper.pcl_wait_save->size();
    for (int n = 0; n < 3; ++n) {
      PointType point{}; point.x = .125 * n; point.z = 5.;
      mapper.pcl_w_wait_pub.reset(new PointCloudXYZI());
      mapper.pcl_w_wait_pub->push_back(point);
      mapper.LidarMeasures.last_lio_update_time = 3. + .1 * n;
      mapper.publish_frame_world(mapper.pubLaserCloudFullRes, mapper.vio_manager);
    }
    require(mapper.pcl_wait_save->size() == previous + 3, "Legacy PCD export lost frames");
    mapper.pcd_save_en = false;
    mapper.img_save_en = true;
    require(!mapper.asyncRgbEnabled(), "Image export inherited latest-wins preview");
    mapper.img_save_en = false; mapper.colmap_output_en = true;
    require(!mapper.asyncRgbEnabled(), "COLMAP export inherited latest-wins preview");
    mapper.colmap_output_en = false;
    executor.remove_node(owner);
    std::cout << "Real mapper RGB output: ROS source stamp, immutable snapshots and complete export passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; result = 1;
  }
  executor.cancel(); serving.join(); rclcpp::shutdown();
  return result;
}

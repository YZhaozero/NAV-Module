#include "cloud_publication.h"
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cloud_publication_regression");
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_publication_regression", rclcpp::QoS(20).reliable());
  std::vector<sensor_msgs::msg::PointCloud2> received;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_publication_regression", rclcpp::QoS(20).reliable(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { received.push_back(*message); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!publisher->get_subscription_count() && std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  require(publisher->get_subscription_count() == 1, "test subscriber did not discover publisher");

  sensor_msgs::msg::PointCloud2 frame;
  frame.header.frame_id = "camera_init";
  frame.width = frame.height = 1;
  frame.point_step = frame.row_step = 16;
  frame.data.resize(16);
  for (std::uint32_t i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = i == 0 ? "x" : i == 1 ? "y" : i == 2 ? "z" : "rgb";
    field.offset = i * 4; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    frame.fields.push_back(field);
  }
  const float xyz[3]{1.f, 2.f, 3.f};
  std::memcpy(frame.data.data(), xyz, sizeof(xyz));
  const std::uint32_t rgb = 0x003388dd;
  std::memcpy(frame.data.data() + 12, &rgb, sizeof(rgb));
  frame.header.stamp.sec = 1;
  require(fast_livo_output::publishFrameCloudIfPresent(publisher, frame), "first coloured frame missing");
  for (int step = 0; step < 20; ++step) {
    sensor_msgs::msg::PointCloud2 pending;
    pending.header = frame.header;
    require(!fast_livo_output::publishFrameCloudIfPresent(publisher, pending), "empty LIO placeholder published");
  }
  frame.header.stamp.sec = 2;
  require(fast_livo_output::publishFrameCloudIfPresent(publisher, frame), "second coloured frame missing");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  require(received.size() == 2, "subscriber received an empty frame or lost a valid frame");
  for (std::size_t i = 0; i < received.size(); ++i) {
    require(received[i].header.stamp.sec == static_cast<int>(i + 1), "frame order changed");
    require(received[i].data == frame.data && received[i].width == 1 && received[i].height == 1,
            "point coordinates or colour changed in publication");
  }
  executor.remove_node(node);
  subscription.reset(); publisher.reset(); node.reset();
  rclcpp::shutdown();
}

#include "sensor_receiver.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>
#include <stdexcept>

static void require(bool value) {
  if (!value) throw std::runtime_error("Isolated IMU receiver regression");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sensor_receiver_regression");
  SensorInbox inbox;
  auto slow_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto imu_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions slow_options, imu_options;
  slow_options.callback_group = slow_group; imu_options.callback_group = imu_group;
  std::atomic<bool> slow_running{false};
  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
      "receiver_test/image", 2, [&](sensor_msgs::msg::Image::ConstSharedPtr) {
        slow_running.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        slow_running.store(false);
      }, slow_options);
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "receiver_test/imu", rclcpp::QoS(2000).reliable(),
      [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        inbox.push(SensorInbox::Imu, sizeof(*message), [message] {});
      }, imu_options);
  auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("receiver_test/image", 2);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("receiver_test/imu", rclcpp::QoS(2000).reliable());
  {
    SensorReceiver slow_receiver(node, slow_group, inbox), imu_receiver(node, imu_group, inbox);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((!image_pub->get_subscription_count() || !imu_pub->get_subscription_count()) &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    require(image_pub->get_subscription_count() && imu_pub->get_subscription_count());
    sensor_msgs::msg::Image image;
    image.height = 480; image.width = 640; image.step = 640 * 3;
    image.encoding = "rgb8"; image.data.resize(image.height * image.step);
    image_pub->publish(image);
    const auto entered = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!slow_running.load() && std::chrono::steady_clock::now() < entered)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    require(slow_running.load());
    const auto initial = inbox.stats().received[SensorInbox::Imu];
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < 60; ++i) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = node->now(); imu.linear_acceleration.z = 9.81;
      imu_pub->publish(imu);
      next += std::chrono::milliseconds(5);
      std::this_thread::sleep_until(next);
    }
    const auto received = inbox.stats().received[SensorInbox::Imu] - initial;
    std::cout << "IMU received while image callback blocked: " << received << "/60\n";
    require(slow_running.load());
    require(received >= 40);  // A shared single-thread executor delivers zero here.
    imu_receiver.stop(); imu_receiver.stop(); // Stop and destructor are idempotent.
    slow_receiver.stop();
  }
  // Stop immediately after construction must also be safe.
  { SensorReceiver stopped(node, imu_group, inbox); stopped.stop(); }
  rclcpp::shutdown();
}

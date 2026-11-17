#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fast_livo_output {

// LIO steps between camera updates have no colour frame. Publishing their
// default-constructed placeholder would repeatedly clear the last valid view.
inline bool publishFrameCloudIfPresent(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
    const sensor_msgs::msg::PointCloud2 &message) {
  if (!message.width || !message.height || message.data.empty()) return false;
  publisher->publish(message);
  return true;
}

}  // namespace fast_livo_output

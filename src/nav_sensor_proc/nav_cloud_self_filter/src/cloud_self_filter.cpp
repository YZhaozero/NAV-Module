#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudSelfFilter : public rclcpp::Node {
 public:
  CloudSelfFilter() : Node("cloud_self_filter") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    frame_ = declare_parameter<std::string>("expected_frame", "base_footprint", fixed);
    const auto minimum = declare_parameter<std::vector<double>>("min", {}, fixed);
    const auto maximum = declare_parameter<std::vector<double>>("max", {}, fixed);
    if (minimum.size() != 3 || maximum.size() != 3)
      throw std::invalid_argument("min and max must each contain three XYZ coordinates in meters");
    for (size_t i = 0; i < 3; ++i)
      if (!std::isfinite(minimum[i]) || !std::isfinite(maximum[i]) || minimum[i] >= maximum[i])
        throw std::invalid_argument("CropBox bounds must be finite, with min < max");
    declare_parameter<bool>("enabled", true);
    crop_.setMin(Eigen::Vector4f(minimum[0], minimum[1], minimum[2], 1.f));
    crop_.setMax(Eigen::Vector4f(maximum[0], maximum[1], maximum[2], 1.f));
    crop_.setNegative(true);  // Keep points outside the robot's exclusion box.
    crop_.setKeepOrganized(false);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS().keep_last(2));
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", rclcpp::SensorDataQoS().keep_last(2),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) { filter(input); });
    RCLCPP_INFO(get_logger(), "Self filter in %s: XYZ min=[%.3f %.3f %.3f] max=[%.3f %.3f %.3f] m",
        frame_.c_str(), minimum[0], minimum[1], minimum[2], maximum[0], maximum[1], maximum[2]);
  }

 private:
  void filter(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input) {
    if (!get_parameter("enabled").as_bool()) {
      publisher_->publish(*input);
      return;
    }
    if (input->header.frame_id != frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "Refusing cloud in frame '%s'; exclusion box requires '%s'",
          input->header.frame_id.c_str(), frame_.c_str());
      return;
    }
    const auto started = std::chrono::steady_clock::now();
    pcl::PCLPointCloud2::Ptr source(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *source);
    crop_.setInputCloud(source);
    pcl::PCLPointCloud2 filtered;
    crop_.filter(filtered);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::moveFromPCL(filtered, output);
    // PCL headers use microseconds. Restore the original ROS nanosecond stamp.
    // PCLPointCloud2 retains all point fields, including Livox's FLOAT64 timestamp.
    output.header = input->header;
    publisher_->publish(output);
    const size_t before = size_t(input->width) * input->height;
    const size_t after = size_t(output.width) * output.height;
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "self filter: input=%zu removed=%zu kept=%zu processing=%.3f ms",
        before, before - after, after, milliseconds);
  }

  std::string frame_;
  pcl::CropBox<pcl::PCLPointCloud2> crop_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSelfFilter>());
  rclcpp::shutdown();
}

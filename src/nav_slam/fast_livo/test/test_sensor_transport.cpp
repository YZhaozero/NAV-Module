#include "sensor_transport.h"
#include "sensor_receiver.h"
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
static void require(bool okay, const char *message) {
  if (!okay) throw std::runtime_error(message);
}
template<class Predicate> static void wait(Predicate predicate, const char *message) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(5ms);
  require(predicate(), message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "imu:=/transport_test/remapped_imu"});
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto estimator = std::make_shared<rclcpp::Node>("transport_test", "/private_ns", options);
  auto source = std::make_shared<rclcpp::Node>("transport_source");
  auto imu_pub = source->create_publisher<sensor_msgs::msg::Imu>("/transport_test/remapped_imu", 2000);
  auto clock_pub = source->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  for (int repetition = 0; repetition < 2; ++repetition) {
    SensorTransport transport(estimator);
    SensorTransport imu_transport(estimator, "_imu_input");
    const auto &receive = imu_transport.node();
    require(imu_transport.context() != transport.context(), "IMU shares large-data receive context");
    auto large_sub = transport.node()->create_subscription<sensor_msgs::msg::Imu>(
        "/transport_test/large_sensor", 10, [](sensor_msgs::msg::Imu::ConstSharedPtr) {});
    require(imu_transport.context() != estimator->get_node_base_interface()->get_context(), "Sensor context shared output context");
    require(std::string(receive->get_name()) == "transport_test_imu_input", "IMU node name remapped unexpectedly");
    require(std::string(transport.node()->get_name()) == "transport_test_sensor_input", "Default transport name changed");
    require(SensorTransport::resolve(estimator, "imu") == "/transport_test/remapped_imu", "Input remap lost");
    require(SensorTransport::resolve(estimator, "~/private") == "/private_ns/transport_test/private", "Private topic resolved on wrong node");
    require(!receive->get_node_options().enable_rosout() &&
        !receive->get_node_options().start_parameter_services() &&
        !receive->get_node_options().start_parameter_event_publisher(), "Receive participant has avoidable output writers");

    SensorInbox inbox;
    auto group = receive->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = group;
    auto sub = receive->create_subscription<sensor_msgs::msg::Imu>(SensorTransport::resolve(estimator, "imu"),
        rclcpp::QoS(2000).reliable(), [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          inbox.push(SensorInbox::Imu, sizeof(*message), [message] {},
              rclcpp::Time(message->header.stamp).seconds(), estimator->now().seconds());
        }, subscription_options);
    SensorReceiver worker(receive, group, inbox);
    wait([&]{ return imu_pub->get_subscription_count() == 1 && clock_pub->get_subscription_count() > 0; }, "Input/clock discovery failed");
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = 200 + repetition;
    // Repeat while discovery and the estimator's existing clock thread settle.
    wait([&]{clock_pub->publish(clock); return estimator->now().seconds() == 200 + repetition;}, "Estimator replay clock did not advance");
    sensor_msgs::msg::Imu message;
    message.header.stamp.sec = 199 + repetition;
    message.header.stamp.nanosec = 950000000;
    message.linear_acceleration.z = 9.81;
    for (int i = 0; i < 40; ++i) { imu_pub->publish(message); std::this_thread::sleep_for(5ms); }
    wait([&]{return inbox.stats().received[SensorInbox::Imu] == 40;}, "Isolated receiver lost messages");
    auto stats = inbox.stats();
    require(std::abs(stats.stamp[SensorInbox::Imu] - (199.95 + repetition)) < 1e-8, "Original sensor Header changed");
    require(stats.received_at[SensorInbox::Imu] == 200 + repetition, "Receive callback used its system clock instead of estimator replay clock");
    const auto infos = source->get_subscriptions_info_by_topic("/transport_test/remapped_imu");
    require(infos.size() == 1, "Unexpected sensor endpoint count");
    wait([&]{return source->get_subscriptions_info_by_topic("/transport_test/large_sensor").size() == 1;}, "Large-input discovery failed");
    const auto large_infos = source->get_subscriptions_info_by_topic("/transport_test/large_sensor");
    require(!std::equal(infos[0].endpoint_gid().begin(), infos[0].endpoint_gid().begin()+12,
        large_infos[0].endpoint_gid().begin()), "IMU and large sensors share a DDS participant/socket");
    const auto writer_gid = imu_pub->get_gid();
    require(!std::equal(infos[0].endpoint_gid().begin(), infos[0].endpoint_gid().begin()+12, writer_gid.data),
        "Independent ROS contexts did not produce independent DDS participant prefixes");
    worker.stop();
    worker.stop();
    sub.reset();
    // Explicit private shutdown must not shut down the application context.
    imu_transport.context()->shutdown("Regression iteration complete");
    require(rclcpp::ok(transport.context()), "IMU shutdown stopped large-input context");
    large_sub.reset();
    require(rclcpp::ok(estimator->get_node_base_interface()->get_context()), "Private transport shutdown stopped application");
    std::cout << "receive context round " << repetition << ": 40/40, remap, replay clock, IMU/large-data participant isolation and shutdown checked\n";
  }
  rclcpp::shutdown();
}

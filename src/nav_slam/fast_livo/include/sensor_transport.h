#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl/context.h>
#include <stdexcept>
#include <string>

// A separate executor is not a separate transport: an output writer can hold
// a DDS lock needed by the same participant's receive thread. Keep sensor
// subscriptions on their own ROS context/participant, without estimator output
// writers (DDS discovery and reliability control traffic still exist).
// The estimator remains the sole owner of state, parameters and ROS time.
class SensorTransport {
public:
  explicit SensorTransport(const rclcpp::Node::SharedPtr &estimator, const std::string &suffix = "_sensor_input")
      : context_(std::make_shared<rclcpp::Context>()) {
    const auto source = estimator->get_node_base_interface()->get_context();
    auto options = source->get_init_options();
    size_t domain = 0;
    const auto result = rcl_context_get_domain_id(source->get_rcl_context().get(), &domain);
    if (result != RCL_RET_OK)
      rclcpp::exceptions::throw_from_rcl_error(result, "Read estimator ROS domain");
    options.set_domain_id(domain);
    options.auto_initialize_logging(false);
    options.shutdown_on_signal = false;  // run() stops receivers before SaveMap/shutdown.
    context_->init(0, nullptr, options);
    rclcpp::NodeOptions node_options;
    node_options.context(context_).use_global_arguments(false)
        .enable_rosout(false).start_parameter_services(false)
        .start_parameter_event_publisher(false).use_clock_thread(false);
    node_ = std::make_shared<rclcpp::Node>(
        std::string(estimator->get_name()) + suffix, estimator->get_namespace(), node_options);
  }
  ~SensorTransport() {
    // Owner must destroy subscription handles and join their workers first.
    node_.reset();
    if (context_->is_valid()) context_->shutdown("Sensor transport released");
  }
  SensorTransport(const SensorTransport &) = delete;
  SensorTransport &operator=(const SensorTransport &) = delete;
  const rclcpp::Node::SharedPtr &node() const { return node_; }
  const rclcpp::Context::SharedPtr &context() const { return context_; }

  static std::string resolve(const rclcpp::Node::SharedPtr &estimator, const std::string &topic) {
    // Resolve namespace, private names and CLI remaps on the original node.
    // Do not copy global __node/parameter overrides onto the receive-only node.
    return estimator->get_node_topics_interface()->resolve_topic_name(topic);
  }
private:
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
};

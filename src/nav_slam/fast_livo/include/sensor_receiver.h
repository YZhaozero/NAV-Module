#pragma once

#include "sensor_inbox.h"
#include <rclcpp/rclcpp.hpp>

// One receive group per worker; all callbacks only enqueue immutable messages.
// In particular, large image takes must not occupy the IMU receive executor.
class SensorReceiver {
public:
  SensorReceiver(const rclcpp::Node::SharedPtr &node,
                 const rclcpp::CallbackGroup::SharedPtr &group, SensorInbox &inbox)
      : inbox_(inbox), context_(node->get_node_base_interface()->get_context()),
        executor_(executorOptions(context_)) {
    executor_.add_callback_group(group, node->get_node_base_interface());
    worker_ = std::thread([this] {
      try {
        while (!stopped_.load() && rclcpp::ok(context_))
          executor_.spin_once(std::chrono::milliseconds(20));
      } catch (...) { inbox_.fail(std::current_exception()); }
    });
  }
  ~SensorReceiver() { stop(); }
  SensorReceiver(const SensorReceiver &) = delete;
  SensorReceiver &operator=(const SensorReceiver &) = delete;
  void stop() {
    stopped_.store(true);
    if (worker_.joinable()) worker_.join();
  }
private:
  static rclcpp::ExecutorOptions executorOptions(const rclcpp::Context::SharedPtr &context) {
    rclcpp::ExecutorOptions options;
    options.context = context;
    return options;
  }
  SensorInbox &inbox_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stopped_{false};
  std::thread worker_;
};

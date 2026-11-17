#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fast_livo/srv/save_map.hpp>
#include <unordered_map>
#include "voxel_key.h"
#include <tuple>
#include <tf2_ros/static_transform_broadcaster.h>
#include "map_spool.h"
#include "map_preview.h"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <future>
#include <limits>
#include "latest_output_worker.h"

// A bounded geometry map, separate from the estimator and its research exports.
class NavigationOutput {
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
  using RgbCloud = pcl::PointCloud<pcl::PointXYZRGB>;
  using PackedCloud = std::vector<MapSpool::Point>;
  static constexpr uint64_t current_generation = std::numeric_limits<uint64_t>::max();
  explicit NavigationOutput(rclcpp::Node::SharedPtr node);
  ~NavigationOutput();
  void publish(Cloud::ConstPtr body, Cloud::ConstPtr world,
               const builtin_interfaces::msg::Time &stamp,
               const std::string &body_frame, const std::string &world_frame);
  void publish(const Cloud &body, const Cloud &world,
               const builtin_interfaces::msg::Time &stamp,
               const std::string &body_frame, const std::string &world_frame);
  void publishRgb(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                  const builtin_interfaces::msg::Time &stamp,
                  const std::string &world_frame);
  void publishRgb(RgbCloud::ConstPtr cloud,
                  const builtin_interfaces::msg::Time &stamp,
                  const std::string &world_frame,
                  uint64_t generation = current_generation);
  uint64_t generation() const { return input_generation_.load(); }
private:
  friend struct NavigationOutputTestAccess;
  struct FrameSnapshot {
    Cloud::ConstPtr body, world;
    builtin_interfaces::msg::Time stamp;
    std::string body_frame, world_frame;
    uint64_t generation;
  };
  struct PreviewSnapshot {
    std::vector<PreviewPoint> points;
    builtin_interfaces::msg::Time stamp;
    std::string world_frame;
    uint64_t generation;
  };
  struct RgbUpdate {
    RgbCloud::ConstPtr cloud;
    builtin_interfaces::msg::Time stamp;
    std::string world_frame;
    uint64_t generation;
  };
  void enqueue(std::function<void()> operation, size_t payload_bytes);
  void drain();
  void work();
  void archive(const PackedCloud &world, const builtin_interfaces::msg::Time &stamp,
               const std::string &world_frame, uint64_t generation);
  void publishFrame(const FrameSnapshot &frame);
  void publishPreview(const PreviewSnapshot &frame);
  void applyPendingRgb(double stamp, const std::string &world_frame, uint64_t generation);
  std::mutex queue_mutex_;
  std::condition_variable queue_changed_;
  struct Pending {
    std::function<void()> operation;
    size_t bytes;
  };
  std::deque<Pending> queue_;
  size_t pending_bytes_ = 0, pending_limit_ = 32 * 1024 * 1024;
  std::thread worker_;
  bool worker_busy_ = false, worker_stop_ = false;
  std::exception_ptr worker_error_;
  std::unique_ptr<fast_livo_output::LatestOutputWorker<FrameSnapshot>> frame_output_;
  std::unique_ptr<fast_livo_output::LatestOutputWorker<PreviewSnapshot>> preview_output_;
  std::mutex generation_mutex_, rgb_mutex_;
  std::atomic<uint64_t> input_generation_{0};
  double input_stamp_ = -1.0, last_rgb_stamp_ = -1.0;
  std::string input_frame_;
  uint64_t archive_generation_ = 0;
  std::shared_ptr<const RgbUpdate> pending_rgb_;
  void checkpoint();
  void save(const std::shared_ptr<fast_livo::srv::SaveMap::Request> request,
            std::shared_ptr<fast_livo::srv::SaveMap::Response> response);
  void waitForSave(std::future<void> &completed);
  // Export owns a separate executor, never the estimator or archive worker.
  rclcpp::CallbackGroup::SharedPtr save_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> save_executor_;
  std::thread save_thread_;
  std::atomic<bool> save_stopping_{false};
  std::mutex save_mutex_;
  rclcpp::Node::SharedPtr node_;
  bool enabled_;
  double voxel_size_, period_, last_publish_ = -1.0;
  int64_t max_voxels_;
  std::string spool_root_;
  // Keep the old capture's recovery lock until an in-flight snapshot finishes.
  std::shared_ptr<MapSpool> spool_;
  std::unique_ptr<MapPreview> preview_;
  std::chrono::steady_clock::time_point last_checkpoint_ = std::chrono::steady_clock::now();
  double checkpoint_period_ = 5.;
  std::unordered_map<VoxelKey, MapSpool::Point, VoxelKeyHash> map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_pub_, world_pub_, map_pub_, rgb_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_pub_;
  rclcpp::Service<fast_livo::srv::SaveMap>::SharedPtr save_service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

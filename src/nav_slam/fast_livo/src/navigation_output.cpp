#include "navigation_output.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <Eigen/Geometry>
#include <unistd.h>
#include <cstdlib>

NavigationOutput::NavigationOutput(rclcpp::Node::SharedPtr node) : node_(node) {
  enabled_ = node_->declare_parameter<bool>("navigation.enabled", false);
  voxel_size_ = node_->declare_parameter<double>("navigation.map_voxel_size", 0.05);
  period_ = node_->declare_parameter<double>("navigation.map_publish_period", 1.0);
  max_voxels_ = node_->declare_parameter<int64_t>("navigation.max_map_voxels", 1000000);
  if (!std::isfinite(voxel_size_) || voxel_size_ < 0.01 ||
      !std::isfinite(period_) || period_ < 0.1 || max_voxels_ < 1)
    throw std::invalid_argument("Invalid navigation map limits");
  if (!enabled_) return;
  const auto pending_limit = node_->declare_parameter<int64_t>(
      "navigation.map_pending_bytes", 32 * 1024 * 1024);
  if (pending_limit < 1024) throw std::invalid_argument("Invalid map pending memory budget");
  pending_limit_ = size_t(pending_limit);
  map_.reserve(std::min<int64_t>(max_voxels_, 250000));
  const auto preview_limit = node_->declare_parameter<int64_t>("navigation.preview_max_points", 120000);
  if (preview_limit < 1) throw std::invalid_argument("Invalid preview budget");
  preview_ = std::make_unique<MapPreview>(voxel_size_, size_t(preview_limit));
  checkpoint_period_ = node_->declare_parameter<double>("navigation.map_checkpoint_period", 5.);
  if (!std::isfinite(checkpoint_period_) || checkpoint_period_ < 1.)
    throw std::invalid_argument("Invalid map checkpoint interval");
  const char *home = std::getenv("HOME");
  spool_root_ = node_->declare_parameter<std::string>("navigation.map_spool_directory",
      std::string(home ? home : "/tmp") + "/.local/state/nav/map-spool");
  spool_ = std::make_shared<MapSpool>(spool_root_, voxel_size_);
  const auto body_frame = node_->get_parameter("navigation.body_frame").as_string();
  const auto lidar_frame = node_->declare_parameter<std::string>("navigation.lidar_frame", "livox_frame");
  const auto camera_frame = node_->declare_parameter<std::string>("navigation.camera_frame", "fast_livo_camera");
  auto vector = [&](const std::string &key, size_t n) {
    auto value = node_->get_parameter(key).as_double_array();
    if (value.size() != n) throw std::invalid_argument("Invalid extrinsic: " + key);
    for (auto x : value) if (!std::isfinite(x)) throw std::invalid_argument("Non-finite extrinsic: " + key);
    return value;
  };
  auto r = vector("extrin_calib.extrinsic_R", 9), t = vector("extrin_calib.extrinsic_T", 3);
  auto cr = vector("extrin_calib.Rcl", 9), ct = vector("extrin_calib.Pcl", 3);
  using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  const Eigen::Matrix3d R = Eigen::Map<RowMatrix>(r.data());
  const Eigen::Matrix3d Rcl = Eigen::Map<RowMatrix>(cr.data());
  const Eigen::Vector3d T = Eigen::Map<Eigen::Vector3d>(t.data());
  const Eigen::Vector3d Pcl = Eigen::Map<Eigen::Vector3d>(ct.data());
  auto transform = [&](const std::string &child, const Eigen::Matrix3d &rotation,
                       const Eigen::Vector3d &translation) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node_->now(); tf.header.frame_id = body_frame; tf.child_frame_id = child;
    tf.transform.translation.x = translation.x(); tf.transform.translation.y = translation.y();
    tf.transform.translation.z = translation.z();
    Eigen::Quaterniond q(rotation); q.normalize();
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    return tf;
  };
  static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  static_tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      transform(lidar_frame, R, T), transform(camera_frame, R * Rcl.transpose(), T - R * Rcl.transpose() * Pcl)});
  body_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/fast_livo/cloud_body", 2);
  world_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/fast_livo/cloud_world", 2);
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fast_livo/map", rclcpp::QoS(1).transient_local());
  rgb_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fast_livo/map_rgb", rclcpp::QoS(1).transient_local());
  preview_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fast_livo/map_preview", rclcpp::QoS(1).transient_local());
  // Do not auto-add this callback group to the estimator's default executor.
  save_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  save_service_ = node_->create_service<fast_livo::srv::SaveMap>(
      "/fast_livo/save_map", std::bind(&NavigationOutput::save, this,
                                      std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, save_group_);
  rclcpp::ExecutorOptions options;
  options.context = node_->get_node_base_interface()->get_context();
  save_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
  save_executor_->add_callback_group(save_group_, node_->get_node_base_interface());
  frame_output_ = std::make_unique<fast_livo_output::LatestOutputWorker<FrameSnapshot>>(
      [this](const auto &frame) {
        try { publishFrame(*frame); }
        catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "Live cloud display failed; retrying next frame: %s", e.what());
        }
      });
  preview_output_ = std::make_unique<fast_livo_output::LatestOutputWorker<PreviewSnapshot>>(
      [this](const auto &frame) {
        try { publishPreview(*frame); }
        catch (const std::exception &e) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                              "Map preview failed; retrying next snapshot: %s", e.what());
        }
      });
  worker_ = std::thread(&NavigationOutput::work, this);
  try {
    save_thread_ = std::thread([this] {
      try {
        // Bounded spin_once also handles cancellation before the thread first
        // starts spinning; no join can wait forever on an idle ROS executor.
        while (!save_stopping_.load() && rclcpp::ok(node_->get_node_base_interface()->get_context()))
          save_executor_->spin_once(std::chrono::milliseconds(50));
      } catch (const std::exception &e) {
        if (!save_stopping_.load() && rclcpp::ok(node_->get_node_base_interface()->get_context()))
          RCLCPP_ERROR(node_->get_logger(), "Map save executor stopped: %s", e.what());
      }
    });
  } catch (...) {
    { std::lock_guard<std::mutex> lock(queue_mutex_); worker_stop_ = true; }
    queue_changed_.notify_all();
    worker_.join();
    throw;
  }
}

NavigationOutput::~NavigationOutput() {
  // Finish a running save while its checkpoint/receipt worker and node still
  // exist. An idle save executor exits even if ROS has already shut down.
  save_stopping_.store(true);
  if (save_executor_) save_executor_->cancel();
  if (save_thread_.joinable()) save_thread_.join();
  try { if (enabled_) { drain(); checkpoint(); } }
  catch (const std::exception &e) { RCLCPP_ERROR(node_->get_logger(), "Map checkpoint failed: %s", e.what()); }
  { std::lock_guard<std::mutex> lock(queue_mutex_); worker_stop_ = true; }
  queue_changed_.notify_all();
  if (worker_.joinable()) worker_.join();
  if (frame_output_) frame_output_->stop(false);
  if (preview_output_) preview_output_->stop(false);
}

void NavigationOutput::enqueue(std::function<void()> operation, size_t payload_bytes) {
  // A two-frame queue makes even a short checkpoint/fsync pause stall the
  // estimator. Absorb bursts with a byte budget, including the active operation,
  // rather than expanding an unbounded frame queue or discarding measured data.
  // One oversized frame may run alone; callers already own that input cloud.
  const size_t bytes = payload_bytes + sizeof(Pending) + 256;
  std::unique_lock<std::mutex> lock(queue_mutex_);
  queue_changed_.wait(lock, [&] {
    return worker_error_ || worker_stop_ || pending_bytes_ == 0 ||
        (queue_.size() < 128 && bytes <= pending_limit_ &&
         pending_bytes_ <= pending_limit_ - bytes);
  });
  if (worker_error_) std::rethrow_exception(worker_error_);
  if (worker_stop_) throw std::runtime_error("Map output is stopped");
  queue_.push_back({std::move(operation), bytes});
  pending_bytes_ += bytes;
  queue_changed_.notify_all();
}

void NavigationOutput::drain() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  queue_changed_.wait(lock, [&] { return (queue_.empty() && !worker_busy_) || worker_error_; });
  if (worker_error_) std::rethrow_exception(worker_error_);
}

void NavigationOutput::work() {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  while (true) {
    queue_changed_.wait(lock, [&] { return worker_stop_ || !queue_.empty(); });
    if (worker_stop_ && queue_.empty()) return;
    auto pending = std::move(queue_.front()); queue_.pop_front(); worker_busy_ = true;
    queue_changed_.notify_all(); lock.unlock();
    try { pending.operation(); }
    catch (...) {
      lock.lock(); worker_error_ = std::current_exception(); worker_busy_ = false;
      queue_.clear(); pending_bytes_ = 0; queue_changed_.notify_all(); return;
    }
    // Release captured clouds before advertising their memory as available.
    pending.operation = {};
    lock.lock(); pending_bytes_ -= pending.bytes;
    worker_busy_ = false; queue_changed_.notify_all();
  }
}

void NavigationOutput::checkpoint() {
  if (map_.empty()) return;
  spool_->spill([&](const MapSpool::Sink &sink) {
    for (const auto &entry : map_) {
      sink(entry.second);
    }
  });
  map_.clear();
  last_checkpoint_ = std::chrono::steady_clock::now();
}

void NavigationOutput::publish(const Cloud &body, const Cloud &world,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &body_frame, const std::string &world_frame) {
  if (!enabled_) return;
  publish(std::make_shared<const Cloud>(body), std::make_shared<const Cloud>(world),
          stamp, body_frame, world_frame);
}

void NavigationOutput::publish(Cloud::ConstPtr body, Cloud::ConstPtr world,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &body_frame, const std::string &world_frame) {
  if (!enabled_) return;
  if (!body || !world) throw std::invalid_argument("Missing map frame");
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(generation_mutex_);
    const double seconds = rclcpp::Time(stamp).seconds();
    if (input_stamp_ >= 0.0 && (seconds < input_stamp_ || input_frame_ != world_frame))
      ++input_generation_;
    input_stamp_ = seconds;
    input_frame_ = world_frame;
    generation = input_generation_.load();
  }
  // PointXYZINormal also carries unused normals and alignment padding. Keep
  // every measured archive point as its exact four float XYZI values instead.
  auto packed = std::make_shared<PackedCloud>();
  packed->reserve(world->size());
  for (const auto &point : *world)
    packed->push_back({point.x, point.y, point.z, point.intensity});
  enqueue([this, packed, stamp, world_frame, generation] {
    archive(*packed, stamp, world_frame, generation);
  }, packed->capacity() * sizeof(MapSpool::Point));
  auto snapshot = std::make_shared<FrameSnapshot>(FrameSnapshot{
      std::move(body), std::move(world), stamp, body_frame, world_frame, generation});
  try { frame_output_->submit(std::move(snapshot)); }
  catch (const std::exception &e) {
    // A failed display must not discard the independently committed geometry.
    RCLCPP_WARN_ONCE(node_->get_logger(), "Live cloud display unavailable: %s", e.what());
  }
}

void NavigationOutput::publishFrame(const FrameSnapshot &frame) {
  if (frame.generation != input_generation_.load()) return;
  sensor_msgs::msg::PointCloud2 msg;
  if (body_pub_->get_subscription_count() > 0) {
    pcl::toROSMsg(*frame.body, msg);
    msg.header.stamp = frame.stamp;
    msg.header.frame_id = frame.body_frame;
    body_pub_->publish(msg);
  }
  if (world_pub_->get_subscription_count() > 0 && frame.generation == input_generation_.load()) {
    pcl::toROSMsg(*frame.world, msg);
    msg.header.stamp = frame.stamp;
    msg.header.frame_id = frame.world_frame;
    world_pub_->publish(msg);
  }
}

void NavigationOutput::archive(const PackedCloud &world,
    const builtin_interfaces::msg::Time &stamp, const std::string &world_frame,
    uint64_t generation) {
  const double now = rclcpp::Time(stamp).seconds();
  if (generation != archive_generation_) {
    checkpoint(); preview_->clear(); last_publish_ = -1.0;
    spool_ = std::make_shared<MapSpool>(spool_root_, voxel_size_);
    archive_generation_ = generation;
    last_rgb_stamp_ = -1.0;
  }
  for (const auto &p : world) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
        std::abs(p[0]) > 1e6 || std::abs(p[1]) > 1e6 || std::abs(p[2]) > 1e6) continue;
    auto key = std::make_tuple(int64_t(std::floor(p[0] / voxel_size_)),
                              int64_t(std::floor(p[1] / voxel_size_)),
                              int64_t(std::floor(p[2] / voxel_size_)));
    if (map_.size() >= size_t(max_voxels_) && map_.find(key) == map_.end()) {
      checkpoint();
    }
    map_.insert_or_assign(key, p);
    preview_->add({p[0], p[1], p[2]});
  }
  applyPendingRgb(now, world_frame, generation);
  if (std::chrono::duration<double>(std::chrono::steady_clock::now()-last_checkpoint_).count() >= checkpoint_period_)
    checkpoint();
  if (last_publish_ >= 0.0 && now - last_publish_ < period_) return;
  last_publish_ = now;
  auto snapshot = std::make_shared<PreviewSnapshot>();
  snapshot->stamp = stamp;
  snapshot->world_frame = world_frame;
  snapshot->generation = generation;
  snapshot->points.reserve(preview_->points().size());
  for (const auto &entry : preview_->points()) snapshot->points.push_back(entry.second);
  try { preview_output_->submit(std::move(snapshot)); }
  catch (const std::exception &e) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Map preview unavailable: %s", e.what());
  }
}

void NavigationOutput::publishPreview(const PreviewSnapshot &frame) {
  if (frame.generation != input_generation_.load()) return;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> colored;
  cloud.reserve(frame.points.size()); colored.reserve(frame.points.size());
  for (const auto &p : frame.points) {
    pcl::PointXYZI geometry; geometry.x=p.x; geometry.y=p.y; geometry.z=p.z; geometry.intensity=0;
    cloud.push_back(geometry);
    pcl::PointXYZRGB rgb; rgb.x=p.x; rgb.y=p.y; rgb.z=p.z; rgb.r=p.r; rgb.g=p.g; rgb.b=p.b;
    colored.push_back(rgb);
  }
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = frame.stamp;
  msg.header.frame_id = frame.world_frame;
  map_pub_->publish(msg);
  if (frame.generation != input_generation_.load()) return;
  pcl::toROSMsg(colored, msg);
  msg.header.stamp=frame.stamp; msg.header.frame_id=frame.world_frame;
  preview_pub_->publish(msg);
  // Legacy preview consumers use the same complete-scene display sample.
  // A second million-point RGB accumulator is neither the persisted map nor
  // needed for rendering; copying it every second stalled the estimator.
  rgb_map_pub_->publish(msg);
}

void NavigationOutput::publishRgb(
    const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &world_frame) {
  if (!enabled_ || cloud.empty()) return;
  publishRgb(std::make_shared<const RgbCloud>(cloud), stamp, world_frame);
}

void NavigationOutput::publishRgb(RgbCloud::ConstPtr cloud,
    const builtin_interfaces::msg::Time &stamp, const std::string &world_frame,
    uint64_t generation) {
  if (!enabled_ || !cloud || cloud->empty()) return;
  if (generation == current_generation) generation = input_generation_.load();
  if (generation != input_generation_.load()) return;
  auto update = std::make_shared<RgbUpdate>(RgbUpdate{std::move(cloud), stamp, world_frame, generation});
  std::shared_ptr<const RgbUpdate> replaced;
  {
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    if (pending_rgb_ && pending_rgb_->generation == generation &&
        rclcpp::Time(stamp) < rclcpp::Time(pending_rgb_->stamp)) return;
    replaced.swap(pending_rgb_);
    pending_rgb_ = std::move(update);
  }
  // Releasing a large previous colour frame never holds a producer lock.
}

void NavigationOutput::applyPendingRgb(double stamp, const std::string &world_frame,
    uint64_t generation) {
  std::shared_ptr<const RgbUpdate> update;
  {
    std::lock_guard<std::mutex> lock(rgb_mutex_);
    if (!pending_rgb_) return;
    if (pending_rgb_->generation > generation) return;
    const double rgb_stamp = rclcpp::Time(pending_rgb_->stamp).seconds();
    if (pending_rgb_->generation == generation && rgb_stamp > stamp + 1e-6) return;
    update.swap(pending_rgb_);
  }
  const double rgb_stamp = rclcpp::Time(update->stamp).seconds();
  if (update->generation != generation || update->world_frame != world_frame ||
      rgb_stamp < last_rgb_stamp_) return;
  last_rgb_stamp_ = rgb_stamp;
  for (const auto &point : *update->cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || std::abs(point.x) > 1e6 ||
        std::abs(point.y) > 1e6 || std::abs(point.z) > 1e6) continue;
    preview_->add({point.x,point.y,point.z,point.r,point.g,point.b,true});
  }
}

void NavigationOutput::save(
    const std::shared_ptr<fast_livo::srv::SaveMap::Request> request,
    std::shared_ptr<fast_livo::srv::SaveMap::Response> response) {
  namespace fs = std::filesystem;
  std::lock_guard<std::mutex> save_lock(save_mutex_);
  fs::path temporary;
  bool committed = false;
  try {
    const fs::path path(request->save_path);
    if (!path.is_absolute() || path.extension() != ".pcd")
      throw std::runtime_error("Use an absolute .pcd output path");
    if (!std::isfinite(request->leaf_size) || request->leaf_size <= 0.)
      throw std::runtime_error("Save leaf size must be positive and finite");
    const double export_leaf = std::max(request->leaf_size, voxel_size_);
    if (request->leaf_size < voxel_size_)
      RCLCPP_WARN(node_->get_logger(), "Requested save resolution %.3f m; using stored map resolution %.3f m",
                  request->leaf_size, voxel_size_);
    if (fs::exists(path)) throw std::runtime_error("Output file already exists");
    struct Job {
      std::shared_ptr<MapSpool> owner;
      MapSpool::Snapshot snapshot;
      uint64_t generation = 0;
    };
    auto job = std::make_shared<Job>();
    auto done = std::make_shared<std::promise<void>>();
    auto completed = done->get_future();
    enqueue([this, job, done] {
      // A short FIFO checkpoint fixes the exact set of accepted frames. Export
      // reads only immutable files after this operation releases the worker.
      // A save error must not poison the archive/estimator queue.
      try {
        if (map_.empty() && spool_->empty()) throw std::runtime_error("No map points received");
        checkpoint();
        job->owner = spool_;
        job->snapshot = spool_->snapshot();
        job->generation = archive_generation_;
        done->set_value();
      } catch (...) { done->set_exception(std::current_exception()); }
    }, 0);
    waitForSave(completed);
    fs::create_directories(path.parent_path());
    auto pattern = path.string() + ".XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end()); buffer.push_back('\0');
    const int fd = mkstemp(buffer.data());
    if (fd < 0) throw std::runtime_error("Cannot create temporary map file");
    close(fd); temporary = buffer.data();
    MapSpool::ExportStats stats;
    const auto cancelled = [this] {
      return save_stopping_.load() || !rclcpp::ok(node_->get_node_base_interface()->get_context());
    };
    MapSpool::exportSnapshot(job->snapshot, temporary.string(), export_leaf,
                            [](const MapSpool::Sink &) {}, &stats, cancelled);
    if (cancelled()) throw std::runtime_error("Map export cancelled; checkpoints preserved");
    MapSpool::commitPcd(temporary, path);
    committed = true;
    // Persist completion before waiting on any later frame/checkpoint. This
    // survives a process exit between commit and the DDS service response.
    try { MapSpool::commitReceipt(path, job->snapshot, job->generation, stats); }
    catch (const std::exception &e) {
      RCLCPP_WARN(node_->get_logger(), "Map saved, durable completion receipt unavailable: %s", e.what());
    }
    try {
      auto marked = std::make_shared<std::promise<void>>();
      auto receipt = marked->get_future();
      enqueue([this, job, path, marked] {
        try {
          // An export of a prefix must not hide newer chunks (or buffered
          // points) from recovery. A different generation has its own spool.
          if (spool_ == job->owner && archive_generation_ == job->generation && map_.empty())
            spool_->markExportedIfCurrent(job->snapshot, path.string());
          marked->set_value();
        } catch (...) { marked->set_exception(std::current_exception()); }
      }, 0);
      waitForSave(receipt);
    } catch (const std::exception &e) {
      RCLCPP_WARN(node_->get_logger(), "Map saved, recovery receipt unavailable: %s", e.what());
    }
    RCLCPP_INFO(node_->get_logger(), "Map saved: %llu points from %zu chunks; %llu observations, %llu SQL rows, %llu merge batches",
                static_cast<unsigned long long>(stats.output_points), job->snapshot.chunks.size(),
                static_cast<unsigned long long>(stats.input_points), static_cast<unsigned long long>(stats.sql_rows),
                static_cast<unsigned long long>(stats.batches));
    RCLCPP_INFO(node_->get_logger(), "Map merge partitions: %llu passes, depth %u, %llu bytes written, %llu peak temporary bytes; fallback low-space %llu / depth %llu",
                static_cast<unsigned long long>(stats.partition_passes), stats.max_partition_depth,
                static_cast<unsigned long long>(stats.partition_bytes_written), static_cast<unsigned long long>(stats.peak_temporary_bytes),
                static_cast<unsigned long long>(stats.low_space_fallbacks), static_cast<unsigned long long>(stats.depth_fallbacks));
    response->success = true;
    response->message = path.string();
  } catch (const std::exception &e) {
    if (!temporary.empty()) {
      std::error_code ignored; fs::remove(temporary, ignored);
    }
    response->success = committed;
    response->message = committed ? request->save_path : e.what();
  }
}

void NavigationOutput::waitForSave(std::future<void> &completed) {
  while (completed.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // The export may finish while this caller is waiting for the queue lock.
    // An error in a later frame must not relabel its committed file as failed.
    if (completed.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) break;
    if (worker_error_) std::rethrow_exception(worker_error_);
  }
  completed.get();
}

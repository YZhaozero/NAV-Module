#include "navigation_output.h"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

using Clock = std::chrono::steady_clock;
static void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

struct Gate {
  std::mutex mutex;
  std::condition_variable wake;
  bool released = false;
  unsigned entered = 0;
  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    ++entered; wake.notify_all();
    wake.wait(lock, [&] { return released; });
  }
  void await(unsigned count) {
    std::unique_lock<std::mutex> lock(mutex);
    require(wake.wait_for(lock, std::chrono::seconds(5), [&] { return entered >= count; }),
            "output callback did not start");
  }
  void release() { std::lock_guard<std::mutex> lock(mutex); released = true; wake.notify_all(); }
  ~Gate() { release(); }
};

// Delay the actual production output workers, never the estimator or archive.
// This reproduces a blocked downstream publisher without requiring a network.
struct NavigationOutputTestAccess {
  static void delayDisplays(NavigationOutput &output, const std::shared_ptr<Gate> &gate) {
    output.frame_output_->stop(); output.preview_output_->stop();
    output.frame_output_ = std::make_unique<fast_livo_output::LatestOutputWorker<NavigationOutput::FrameSnapshot>>(
        [gate](auto) { gate->wait(); });
    output.preview_output_ = std::make_unique<fast_livo_output::LatestOutputWorker<NavigationOutput::PreviewSnapshot>>(
        [gate](auto) { gate->wait(); });
  }
  static void drain(NavigationOutput &output) { output.drain(); }
  static void delayArchive(NavigationOutput &output, const std::shared_ptr<Gate> &gate) {
    output.enqueue([gate] { gate->wait(); }, 0);
  }
  static size_t bytes(NavigationOutput &output) {
    std::lock_guard<std::mutex> lock(output.queue_mutex_); return output.pending_bytes_;
  }
  static bool displaysBounded(NavigationOutput &output) {
    const auto a = output.frame_output_->stats();
    const auto b = output.preview_output_->stats();
    return a.active && a.pending && b.active && b.pending && a.replaced > 0 && b.replaced > 0;
  }
  static std::shared_ptr<fast_livo::srv::SaveMap::Response> save(NavigationOutput &output,
      const std::string &path) {
    auto request = std::make_shared<fast_livo::srv::SaveMap::Request>();
    auto response = std::make_shared<fast_livo::srv::SaveMap::Response>();
    request->save_path = path; request->leaf_size = .05;
    output.save(request, response); return response;
  }
  static bool colored(NavigationOutput &output, float x) {
    for (const auto &entry : output.preview_->points())
      if (entry.second.x == x && entry.second.colored) return true;
    return false;
  }
  static void completedSaveWinsLaterFailure(NavigationOutput &output) {
    std::promise<void> saved;
    auto future = saved.get_future();
    std::exception_ptr failure;
    std::unique_lock<std::mutex> lock(output.queue_mutex_);
    std::thread waiter([&] {
      try { output.waitForSave(future); } catch (...) { failure = std::current_exception(); }
    });
    // Make the first wait time out while the save callback cannot inspect the
    // queue. Complete its own job, then represent a later archive job failure.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    saved.set_value();
    output.worker_error_ = std::make_exception_ptr(std::runtime_error("later frame failed"));
    lock.unlock(); waiter.join(); lock.lock(); output.worker_error_ = nullptr;
    require(!failure, "later archive failure overwrote completed save result");
  }
};

static NavigationOutput::Cloud::ConstPtr cloud(int region, int count = 200) {
  auto result = std::make_shared<NavigationOutput::Cloud>();
  for (int i = 0; i < count; ++i) {
    pcl::PointXYZINormal point{};
    point.x = region * 100.f + i * .125f; point.y = .25f; point.z = 1.f;
    point.intensity = float(region * 1000 + i);
    result->push_back(point);
  }
  return result;
}
static builtin_interfaces::msg::Time stamp(int seconds) {
  builtin_interfaces::msg::Time result; result.sec = seconds; return result;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  char root[] = "/tmp/nav-isolation-XXXXXX";
  require(mkdtemp(root), "temp directory failed");
  auto node = std::make_shared<rclcpp::Node>("output_isolation",
      rclcpp::NodeOptions().parameter_overrides({
          rclcpp::Parameter("navigation.enabled", true),
          rclcpp::Parameter("navigation.map_voxel_size", .05),
          rclcpp::Parameter("navigation.max_map_voxels", 50000),
          rclcpp::Parameter("navigation.map_checkpoint_period", 3600.),
          rclcpp::Parameter("navigation.map_spool_directory", std::string(root))}));
  node->declare_parameter("navigation.body_frame", "base_link");
  for (auto key : {"extrin_calib.extrinsic_R", "extrin_calib.Rcl"})
    node->declare_parameter(key, std::vector<double>{1,0,0,0,1,0,0,0,1});
  for (auto key : {"extrin_calib.extrinsic_T", "extrin_calib.Pcl"})
    node->declare_parameter(key, std::vector<double>{0,0,0});
  {
    NavigationOutput output(node);
    auto display_gate = std::make_shared<Gate>();
    // Release before output destruction even if a test assertion fails.
    struct Release { std::shared_ptr<Gate> gate; ~Release() { gate->release(); } } release{display_gate};
    NavigationOutputTestAccess::delayDisplays(output, display_gate);
    auto first = cloud(0);
    output.publish(first, first, stamp(1), "base_link", "odom");
    display_gate->await(2);
    const auto start = Clock::now();
    for (int i = 1; i < 100; ++i) {
      auto frame = cloud(i);
      output.publish(frame, frame, stamp(i + 1), "base_link", "odom");
    }
    const double submit_s = std::chrono::duration<double>(Clock::now() - start).count();
    require(submit_s < 1., "slow displays blocked frame submission");
    auto saved = NavigationOutputTestAccess::save(output, std::string(root) + "/complete.pcd");
    require(saved->success, saved->message.c_str());
    require(NavigationOutputTestAccess::displaysBounded(output), "display buffers did not coalesce");
    pcl::PointCloud<pcl::PointXYZI> points;
    require(pcl::io::loadPCDFile(std::string(root) + "/complete.pcd", points) == 0 && points.size() == 20000,
            "slow displays caused geometry loss");
    for (const auto &point : points) {
      const int region = int(point.x / 100.f);
      const int index = int(std::lround((point.x - region * 100.f) * 8.f));
      require(point.intensity == float(region * 1000 + index), "packed XYZI changed measured values");
    }
    display_gate->release();

    // A stopped archive absorbs compact records rather than padded PCL points.
    auto disk_gate = std::make_shared<Gate>();
    Release disk_release{disk_gate};
    NavigationOutputTestAccess::delayArchive(output, disk_gate); disk_gate->await(1);
    auto large = cloud(150, 1000);
    output.publish(large, large, stamp(101), "base_link", "odom");
    const auto queued_bytes = NavigationOutputTestAccess::bytes(output);
    require(queued_bytes >= 1000 * sizeof(MapSpool::Point) && queued_bytes < 18000,
            "archive queue still stores padded full clouds");
    auto colors = std::make_shared<NavigationOutput::RgbCloud>();
    pcl::PointXYZRGB color{}; color.x = 30000; color.y = .25; color.z = 1; color.r = 255;
    colors->push_back(color);
    const auto old_generation = output.generation();
    output.publishRgb(colors, stamp(101), "odom", old_generation);
    disk_gate->release(); NavigationOutputTestAccess::drain(output);

    // A timestamp rewind AND frame switch starts a new saved map. An old RGB
    // callback finishing later may not insert its old coordinates into it.
    auto current = cloud(200);
    output.publish(current, current, stamp(1), "base_link", "new_odom");
    output.publishRgb(colors, stamp(101), "odom", old_generation);
    NavigationOutputTestAccess::drain(output);
    require(!NavigationOutputTestAccess::colored(output, 30000.f), "old RGB contaminated new session");
    colors->front().x = current->front().x;
    output.publishRgb(colors, stamp(1), "new_odom", output.generation());
    output.publish(current, current, stamp(2), "base_link", "new_odom");
    NavigationOutputTestAccess::drain(output);
    require(NavigationOutputTestAccess::colored(output, current->front().x), "new RGB was not applied");
    saved = NavigationOutputTestAccess::save(output, std::string(root) + "/new-session.pcd");
    require(saved->success, saved->message.c_str());
    require(pcl::io::loadPCDFile(std::string(root) + "/new-session.pcd", points) == 0 && points.size() == 200,
            "saved map joined two coordinate frames");
    NavigationOutputTestAccess::completedSaveWinsLaterFailure(output);
    std::cout << "output isolation passed: 20000 exact points, submit_s=" << submit_s
              << ", compact_queue_bytes=" << queued_bytes << '\n';
  }
  std::filesystem::remove_all(root);
  rclcpp::shutdown();
}

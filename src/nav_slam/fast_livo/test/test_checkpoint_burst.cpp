#include "navigation_output.h"
#include <pcl/io/pcd_io.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/syscall.h>
#include <thread>

// Only this test executable interposes the syscall. Production binaries never
// acquire this delay. Exercise a real checkpoint, not a mocked archive queue.
static std::atomic<bool> delay_checkpoint{false};
extern "C" int fdatasync(int fd) {
  char link[64], target[1024];
  snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
  const auto count = readlink(link, target, sizeof(target)-1);
  if (count > 0) {
    target[count] = 0;
    if (delay_checkpoint && std::strstr(target, "/nav-burst-test-") &&
        std::strstr(target, ".bin.partial"))
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }
  return syscall(SYS_fdatasync, fd);
}

static void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

static void burst(int64_t budget, bool should_wait) {
  char root[] = "/tmp/nav-burst-test-XXXXXX";
  check(mkdtemp(root), "cannot create test directory");
  auto node = std::make_shared<rclcpp::Node>("checkpoint_burst",
      rclcpp::NodeOptions().parameter_overrides({
          rclcpp::Parameter("navigation.enabled", true),
          rclcpp::Parameter("navigation.map_voxel_size", .05),
          rclcpp::Parameter("navigation.max_map_voxels", 5000),
          rclcpp::Parameter("navigation.map_pending_bytes", budget),
          rclcpp::Parameter("navigation.map_checkpoint_period", 3600.),
          rclcpp::Parameter("navigation.map_spool_directory", std::string(root))}));
  node->declare_parameter("navigation.body_frame", "base_link");
  for (auto key : {"extrin_calib.extrinsic_R", "extrin_calib.Rcl"})
    node->declare_parameter(key, std::vector<double>{1,0,0,0,1,0,0,0,1});
  for (auto key : {"extrin_calib.extrinsic_T", "extrin_calib.Pcl"})
    node->declare_parameter(key, std::vector<double>{0,0,0});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  {
    NavigationOutput output(node);
    delay_checkpoint = true;
    const auto start = std::chrono::steady_clock::now();
    for (int frame=0; frame<20; ++frame) {
      NavigationOutput::Cloud cloud;
      cloud.reserve(1000);
      for (int i=0; i<1000; ++i) {
        pcl::PointXYZINormal p{};
        p.x = frame*10 + (i%100)*.1f; p.y = (i/100)*.1f;
        p.z = 1.; p.intensity = frame;
        cloud.push_back(p);
      }
      builtin_interfaces::msg::Time stamp; stamp.sec = frame+1;
      output.publish(cloud, cloud, stamp, "base_link", "odom");
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now()-start).count();
    std::cout << "budget=" << budget << " submit_seconds=" << elapsed << std::endl;
    check(should_wait ? elapsed >= .4 : elapsed < .35,
          "checkpoint burst did not obey memory/backpressure policy");
    auto client = node->create_client<fast_livo::srv::SaveMap>("/fast_livo/save_map");
    check(client->wait_for_service(std::chrono::seconds(2)), "save unavailable");
    auto request = std::make_shared<fast_livo::srv::SaveMap::Request>();
    request->save_path = std::string(root)+"/result.pcd"; request->leaf_size = .05;
    auto future = client->async_send_request(request);
    check(executor.spin_until_future_complete(future, std::chrono::seconds(20)) ==
          rclcpp::FutureReturnCode::SUCCESS, "save timed out");
    check(future.get()->success, "save failed");
    pcl::PointCloud<pcl::PointXYZI> saved;
    check(pcl::io::loadPCDFile(request->save_path, saved) == 0 && saved.size() == 20000,
          "archive lost measured points during a checkpoint burst");
    delay_checkpoint = false;
  }
  executor.remove_node(node);
  std::filesystem::remove_all(root);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  burst(32*1024*1024, false);
  // A deliberately small budget must apply backpressure rather than grow
  // memory without bound. Even one frame larger than it must save completely.
  burst(32*1024, true);
  rclcpp::shutdown();
}

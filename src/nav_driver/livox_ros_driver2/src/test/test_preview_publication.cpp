// Exercise the production Lddc path; block only the display publish at its ROS
// boundary. No SDK, sensor, network fault or live application is involved.
#include "lddc.h"
#include "comm/ldq.h"
#include <rcl/publisher.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace livox_ros;
using Clock = std::chrono::steady_clock;
namespace {
std::mutex gate;
std::condition_variable changed;
bool entered = false, release_preview = false;
std::atomic<int> primary{0}, previews{0}, malformed{0};
std::atomic<uint64_t> last_preview{0};
size_t imu_depth=0; bool imu_reliable=false;
const uint64_t epoch = 1800000000000000000ULL;
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *publisher,
    const void *data, rmw_publisher_allocation_t *) {
  const std::string topic = rcl_publisher_get_topic_name(publisher);
  if (topic.find("/imu") != std::string::npos) {
    const auto *qos=rcl_publisher_get_actual_qos(publisher);
    if (qos) { imu_depth=qos->depth; imu_reliable=qos->reliability==RMW_QOS_POLICY_RELIABILITY_RELIABLE; }
  } else if (topic.find("/pointcloud") != std::string::npos) {
    if (++previews == 1) {
      std::unique_lock<std::mutex> lock(gate);
      entered = true;
      changed.notify_all();
      // Timeout makes the old synchronous implementation fail, not deadlock.
      changed.wait_for(lock, std::chrono::milliseconds(800), [] { return release_preview; });
    }
    auto &cloud = *static_cast<const PointCloud2 *>(data);
    last_preview = rclcpp::Time(cloud.header.stamp).nanoseconds();
  } else if (topic.find("/lidar") != std::string::npos) {
    const auto &cloud = *static_cast<const CustomMsg *>(data);
    const int n = primary.fetch_add(1);
    if (cloud.timebase != epoch + uint64_t(n) * 100000000 || cloud.points.size() != 20 ||
        cloud.points.back().offset_time != 19000 || cloud.points.back().x != 19.f)
      ++malformed;
  }
  return RCL_RET_OK;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("preview_isolation_test");
  Lds source(10);
  source.lidars_[0].lidar_type = kLivoxLidarType;
  source.lidars_[0].handle = 1;
  source.lidars_[0].connect_state = kConnectStateSampling;
  std::string frame = "livox_frame";
  Lddc publisher(kAllMsg, std::getenv("LIVOX_TEST_MULTI") ? 1 : 0, frame);
  publisher.SetRosNode(node.get());
  publisher.RegisterLds(&source);
  auto send = [&](int n) {
    std::vector<PointXyzlt> points(20);
    const uint64_t stamp = epoch + uint64_t(n) * 100000000;
    for (size_t i = 0; i < points.size(); ++i) {
      points[i].x = i;
      points[i].offset_time = stamp + i * 1000;
    }
    PointPacket packet{};
    packet.points = points.data(); packet.points_num = points.size();
    source.PushLidarData(&packet, 0, stamp);
    publisher.DistributePointCloudData();
  };
  const auto begin = Clock::now();
  send(0);
  {
    std::unique_lock<std::mutex> lock(gate);
    changed.wait_for(lock, std::chrono::seconds(2), [] { return entered; });
  }
  for (int i = 1; i <= 100; ++i) send(i);
  const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
  const int primary_while_blocked = primary;
  {
    std::lock_guard<std::mutex> lock(gate);
    release_preview = true;
  }
  changed.notify_all();
  for (int i = 0; i < 200 && last_preview != epoch + 100ULL * 100000000; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  // Inspect actual production publisher QoS for both shared and private topics.
  // It must cover at least six seconds at 200 Hz, with a bounded allocation.
  ImuData imu{}; imu.time_stamp=epoch; imu.acc_z=1.f;
  source.lidar_count_=1; source.lidars_[0].imu_data.Push(&imu);
  source.imu_semaphore_.Signal(); publisher.DistributeImuData();
  publisher.PrepareExit();
  rclcpp::shutdown();
  std::cout << "{\"primary_frames\":" << primary_while_blocked
            << ",\"preview_frames\":" << previews
            << ",\"producer_seconds\":" << elapsed
            << ",\"malformed\":" << malformed << ",\"imu_history\":" << imu_depth << "}\n";
  return (primary_while_blocked == 101 && malformed == 0 && elapsed < .5 &&
          last_preview == epoch + 100ULL * 100000000 && previews <= 2 &&
          imu_reliable && imu_depth>=1200 && imu_depth<=4000) ? 0 : 1;
}

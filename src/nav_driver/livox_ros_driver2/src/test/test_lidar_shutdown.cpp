// Exercise the production distribution waits after the sensor stops sending.
// No SDK, sensor or application process is started. Publication is intercepted
// at the ROS boundary for the queued-data test.
#include "lddc.h"
#include "comm/ldq.h"
#include <rcl/publisher.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

using namespace livox_ros;
using Clock = std::chrono::steady_clock;

namespace {
Lds *stop_after_publish = nullptr;
std::atomic<int> primary_published{0};
}

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *publisher,
    const void *, rmw_publisher_allocation_t *) {
  const std::string topic = rcl_publisher_get_topic_name(publisher);
  if (topic.find("/lidar") != std::string::npos &&
      topic.find("/pointcloud") == std::string::npos && stop_after_publish) {
    ++primary_published;
    stop_after_publish->RequestExit();
  }
  return RCL_RET_OK;
}

int main(int argc, char **argv) {
  bool passed = true;
  for (int attempt = 0; attempt != 5; ++attempt) {
    Lds source(10);
    std::string frame = "livox_frame";
    Lddc distributor(kAllMsg, 0, frame);
    distributor.RegisterLds(&source);
    std::atomic<int> entered{0}, returned{0};
    auto cloud = std::thread([&] {
      ++entered;
      distributor.DistributePointCloudData();
      ++returned;
    });
    auto imu = std::thread([&] {
      ++entered;
      distributor.DistributeImuData();
      ++returned;
    });
    while (entered != 2) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (returned != 0) passed = false;
    const auto begin = Clock::now();
    source.RequestExit();
    const auto deadline = begin + std::chrono::milliseconds(250);
    while (returned != 2 && Clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool woke = returned == 2;
    passed = passed && woke;
    if (!woke) {
      // Let the old implementation finish so a regression fails, not hangs.
      source.pcd_semaphore_.Signal();
      source.imu_semaphore_.Signal();
    }
    cloud.join();
    imu.join();
    source.RequestExit();  // Repeated shutdown remains safe.
    distributor.DistributePointCloudData();
    distributor.DistributeImuData();
    std::cout << "attempt=" << attempt << " consumers_woke=" << woke
              << " elapsed_ms=" << std::chrono::duration<double, std::milli>(
                     Clock::now() - begin).count() << '\n';
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lidar_shutdown_test");
  for (auto format : {kPointCloud2Msg, kLivoxCustomMsg, kAllMsg}) {
    Lds source(10);
    source.lidars_[0].lidar_type = kLivoxLidarType;
    source.lidars_[0].handle = 1;
    source.lidars_[0].connect_state = kConnectStateSampling;
    std::string frame = "livox_frame";
    Lddc distributor(format, 0, frame);
    distributor.SetRosNode(node.get());
    distributor.RegisterLds(&source);
    PointXyzlt point{};
    for (uint64_t n = 0; n != 3; ++n) {
      point.offset_time = 1800000000000000000ULL + n * 100000000;
      PointPacket packet{};
      packet.points = &point;
      packet.points_num = 1;
      source.PushLidarData(&packet, 0, point.offset_time);
    }
    primary_published = 0;
    stop_after_publish = &source;
    distributor.DistributePointCloudData();
    distributor.PrepareExit();  // Joins any pending preview before next source.
    stop_after_publish = nullptr;
    const auto remaining = QueueUsedSize(&source.lidars_[0].data);
    const bool stopped_between_frames = primary_published == 1 && remaining == 2;
    passed = passed && stopped_between_frames;
    std::cout << "format=" << int(format) << " primary_published="
              << primary_published << " remaining=" << remaining << '\n';
  }
  node.reset();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}

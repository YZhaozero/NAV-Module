// Exercise the production packet callback, per-lidar transforms and ROS output.
// The SDK observer and ROS publish boundaries are intercepted; no device starts.
#include "comm/pub_handler.h"
#include "lddc.h"
#include <arpa/inet.h>
#include <rcl/publisher.h>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace livox_ros;
namespace {
LivoxLidarPointCloudObserver observer;
void *observer_context;
std::map<std::string, ImuMsg> imu_messages;
std::map<std::string, PointCloud2> cloud_messages;
}
extern "C" uint16_t LivoxLidarAddPointCloudObserver(
    LivoxLidarPointCloudObserver callback, void *context) {
  observer = callback; observer_context = context; return 1;
}
extern "C" void LivoxLidarRemovePointCloudObserver(uint16_t) {}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *publisher,
    const void *data, rmw_publisher_allocation_t *) {
  const std::string topic = rcl_publisher_get_topic_name(publisher);
  if (topic.find("/livox/imu_") == 0)
    imu_messages[topic] = *static_cast<const ImuMsg *>(data);
  else if (topic.find("/livox/lidar_") == 0)
    cloud_messages[topic] = *static_cast<const PointCloud2 *>(data);
  return RCL_RET_OK;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("woosh_three_lidar_test");
  Lds source(10); source.lidar_count_ = 3;
  std::string frame = "base_footprint";
  Lddc output(kPointCloud2Msg, 1, frame);
  output.SetRosNode(node.get()); output.RegisterLds(&source);
  PubHandler packets;
  packets.SetImuDataCallback([&](ImuData *imu, void *) { source.StorageImuData(imu); }, nullptr);
  packets.SetPointCloudsCallback([](PointFrame *, void *) {}, nullptr);
  std::array<LidarPubHandler, 3> clouds;
  const ExtParameter mounts[3] = {{0,0,45,338,190,330},
      {0,0,-135,-338,-190,330}, {0,30,0,429,0,605}};
  const float expected[3][3] = {{1.04510678f,.89710678f,.330f},
      {-1.04510678f,-.89710678f,.330f}, {1.29502540f,0,.105f}};
  const uint64_t epoch = 1800000000000000000ULL;
  for (int i = 0; i < 3; ++i) {
    const std::string ip = "192.168.128." + std::to_string(i + 3);
    const uint32_t handle = inet_addr(ip.c_str());
    uint8_t index = 0;
    if (source.cache_index_.GetFreeIndex(kLivoxLidarType, handle, index) || index != i)
      throw std::runtime_error("Unexpected device index");
    source.lidars_[i].handle = handle;
    source.lidars_[i].lidar_type = kLivoxLidarType;
    source.lidars_[i].connect_state = kConnectStateSampling;
    LidarExtParameter mount{}; mount.handle = handle;
    mount.lidar_type = kLivoxLidarType; mount.param = mounts[i];
    clouds[i].SetLidarsExtParam(mount);
  }
  // Initialize every mount first: the old shared IMU rotation used the last one.
  for (int i = 0; i < 3; ++i) {
    LivoxLidarCartesianHighRawPoint point{}; point.x = 1000;
    RawPacket raw{}; raw.lidar_type = kLivoxLidarType;
    raw.data_type = kLivoxLidarCartesianCoordinateHighData;
    raw.line_num = 4; raw.point_num = 1; raw.time_stamp = epoch + i * 1000000;
    raw.raw_data.resize(sizeof(point)); memcpy(raw.raw_data.data(), &point, sizeof(point));
    clouds[i].PointCloudProcess(raw);
    std::vector<PointXyzlt> transformed; clouds[i].GetLidarPointClouds(transformed);
    PointPacket frame_packet{}; frame_packet.points = transformed.data();
    frame_packet.points_num = transformed.size();
    source.PushLidarData(&frame_packet, i, raw.time_stamp);
    output.DistributePointCloudData();
    RawImuPoint imu{.1f+i,.2f+i,.3f+i,.4f+i,.5f+i,1.f+i};
    std::vector<uint8_t> bytes(sizeof(LivoxLidarEthernetPacket)-1+sizeof(imu));
    auto *packet = reinterpret_cast<LivoxLidarEthernetPacket *>(bytes.data());
    packet->length = bytes.size(); packet->time_type = kTimestampTypeGptpOrPtp;
    packet->data_type = kLivoxLidarImuData;
    memcpy(packet->timestamp, &raw.time_stamp, sizeof(raw.time_stamp));
    memcpy(packet->data, &imu, sizeof(imu));
    observer(source.lidars_[i].handle, LivoxLidarDeviceType::kLivoxLidarTypeMid360,
             packet, observer_context);
    output.DistributeImuData();
  }
  if (imu_messages.size() != 3 || cloud_messages.size() != 3)
    throw std::runtime_error("Three distinct pointcloud and IMU topics required");
  for (int i = 0; i < 3; ++i) {
    const std::string suffix = "192_168_128_" + std::to_string(i+3);
    const auto &cloud = cloud_messages.at("/livox/lidar_"+suffix);
    const auto &imu = imu_messages.at("/livox/imu_"+suffix);
    if (cloud.header.frame_id != "base_footprint" || imu.header.frame_id != "livox_frame" ||
        cloud.width*cloud.height != 1 ||
        rclcpp::Time(cloud.header.stamp).nanoseconds() != int64_t(epoch+i*1000000) ||
        rclcpp::Time(imu.header.stamp).nanoseconds() != int64_t(epoch+i*1000000))
      throw std::runtime_error("Frame or per-device timestamp changed");
    float xyz[3]; memcpy(xyz, cloud.data.data(), sizeof(xyz));
    for (int j=0; j<3; ++j)
      if (std::abs(xyz[j]-expected[i][j])>1e-6)
        throw std::runtime_error("Wrong per-lidar pointcloud rotation or translation");
    const double values[] = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
    const float expected_imu[] = {.1f+i,.2f+i,.3f+i,.4f+i,.5f+i,1.f+i};
    for (int j=0; j<6; ++j)
      if (values[j] != expected_imu[j])
        throw std::runtime_error("IMU axes/units changed or data crossed between devices");
  }
  output.PrepareExit(); packets.Uninit(); rclcpp::shutdown();
  std::cout << "PASS: three independent clouds in base_footprint, raw IMU axes/units and timestamps preserved\n";
}

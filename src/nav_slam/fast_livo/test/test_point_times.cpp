#include "preprocess.h"
#include <array>
#include <cmath>
#include <stdexcept>

static void require(bool okay) {
  if (!okay) throw std::runtime_error("Livox acquisition-time regression failed");
}

int main() {
  Preprocess preprocess;
  preprocess.set(false, AVIA, 0.5, 1);
  auto message = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  // Packet reordering, equal-time returns, and a real inter-packet gap.
  // The first acquired return need not coincide with the scan header.
  const std::array<uint32_t, 6> times{2000000, 18000000, 6000000, 6000000, 90000000, 30000000};
  for (size_t i = 0; i < times.size(); ++i) {
    livox_ros_driver2::msg::CustomPoint point;
    point.x = 1.0f + float(i); point.y = 0.2f; point.z = -0.3f;
    point.line = 0; point.tag = 0x10; point.reflectivity = uint8_t(i);
    point.offset_time = times[i]; message->points.push_back(point);
  }
  message->point_num = message->points.size();
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  preprocess.process(message, cloud);
  require(cloud->size() == times.size());
  const std::array<size_t, 6> order{0, 2, 3, 1, 5, 4};
  for (size_t i = 0; i < order.size(); ++i) {
    const auto &point = cloud->points[i];
    require(std::abs(point.curvature - times[order[i]] / 1000000.0) < 1e-5);
    require(point.intensity == order[i]);
    require(point.x == 1.0f + float(order[i]));
  }
  // Reusing the processor must not retain points from the preceding scan.
  message->points.clear(); message->point_num = 0;
  preprocess.process(message, cloud);
  require(cloud->empty());
}

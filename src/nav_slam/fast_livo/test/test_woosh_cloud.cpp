#include "preprocess.h"
#include <array>
#include <cstring>
#include <cmath>
#include <stdexcept>

static void require(bool okay) {
  if (!okay) throw std::runtime_error("WOOSH cloud timing/frame regression failed");
}

int main() {
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  message->header.frame_id = "base_footprint";
  message->header.stamp.sec = 1700000000; message->header.stamp.nanosec = 123456789;
  message->width = 3; message->height = 2; message->point_step = 26;
  message->row_step = 3 * 26 + 4; // Include row padding and an unaligned timestamp.
  using Field = sensor_msgs::msg::PointField;
  const std::array<std::string, 7> names{"x", "y", "z", "intensity", "tag", "line", "timestamp"};
  const std::array<uint32_t, 7> offsets{0, 4, 8, 12, 16, 17, 18};
  for (size_t i = 0; i < names.size(); ++i) {
    Field field; field.name = names[i]; field.offset = offsets[i]; field.count = 1;
    field.datatype = i == 6 ? Field::FLOAT64 : i >= 4 ? Field::UINT8 : Field::FLOAT32;
    message->fields.push_back(field);
  }
  message->data.resize(message->row_step * message->height);
  const std::array<double, 6> milliseconds{2., 18., 6., 6., 90., 30.};
  const double header_ns = double(message->header.stamp.sec) * 1e9 + message->header.stamp.nanosec;
  for (size_t i = 0; i < milliseconds.size(); ++i) {
    auto *data = message->data.data() + i / 3 * message->row_step + i % 3 * 26;
    const std::array<float, 4> values{float(i + 1), .2f, -.3f, float(i)};
    std::memcpy(data, values.data(), 16); data[16] = 0x10; data[17] = 0;
    const double stamp = header_ns + milliseconds[i] * 1e6;
    std::memcpy(data + 18, &stamp, 8);
  }
  Preprocess preprocess; preprocess.set(false, WOOSH, .5, 1);
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  preprocess.process(message, cloud);
  require(cloud->size() == 6);
  const std::array<size_t, 6> order{0, 2, 3, 1, 5, 4};
  for (size_t i = 0; i < order.size(); ++i) {
    require(std::abs(cloud->points[i].curvature - milliseconds[order[i]]) < .001);
    require(cloud->points[i].x == float(order[i] + 1)); // Keep driver-transformed base XYZ.
    require(cloud->points[i].intensity == float(order[i]));
  }
  message->header.frame_id = "livox_frame";
  bool rejected = false;
  try { preprocess.process(message, cloud); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
  message->header.frame_id = "base_footprint";
  const double seconds_instead_of_nanoseconds = 1700000000.;
  std::memcpy(message->data.data() + 18, &seconds_instead_of_nanoseconds, 8);
  rejected = false;
  try { preprocess.process(message, cloud); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
  message->fields.pop_back(); rejected = false;
  try { preprocess.process(message, cloud); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
}

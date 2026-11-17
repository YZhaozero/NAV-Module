#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cstdint>
#include <string>

namespace tracking {
// Validate the byte layout before PCL touches it. PCL assumes the ROS producer
// has supplied a complete, native-endian XYZ float32 buffer.
inline bool cloudLayoutValid(const sensor_msgs::msg::PointCloud2 &cloud) {
    if (uint64_t(cloud.width) * cloud.height < 20 || cloud.point_step < 12 ||
        cloud.is_bigendian || uint64_t(cloud.width) * cloud.point_step > cloud.row_step ||
        uint64_t(cloud.row_step) * cloud.height != cloud.data.size()) return false;
    for (const std::string name : {"x", "y", "z"}) {
        const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
            [&](const auto &item) { return item.name == name; });
        if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field->count != 1 || uint64_t(field->offset) + 4 > cloud.point_step) return false;
    }
    // Validate additional fields too: PCL may map intensity from them.
    for (const auto &field : cloud.fields) {
        const uint64_t sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
        if (field.datatype < 1 || field.datatype > 8 || !field.count ||
            uint64_t(field.offset) + sizes[field.datatype] * field.count > cloud.point_step) return false;
    }
    return true;
}
inline bool framesValid(const std::string &cloud, const std::string &odom,
                        const std::string &body, const std::string &map) {
    return !odom.empty() && !body.empty() && cloud == body &&
        odom != body && map != body && map != odom;
}
}

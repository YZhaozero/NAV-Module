#include "../src/input_policy.h"
#include <cassert>
#include <limits>

int main() {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width = 20; cloud.height = 1; cloud.point_step = 12; cloud.row_step = 240;
    cloud.data.resize(240);
    for (const auto &name : {"x", "y", "z"}) {
        sensor_msgs::msg::PointField field;
        field.name = name; field.offset = cloud.fields.size() * 4;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        cloud.fields.push_back(field);
    }
    assert(tracking::cloudLayoutValid(cloud));
    cloud.data.resize(12); assert(!tracking::cloudLayoutValid(cloud));
    cloud.data.resize(240); cloud.fields[2].offset = 12;
    assert(!tracking::cloudLayoutValid(cloud));
    cloud.fields[2].offset = 8; cloud.fields[2].datatype = 99;
    assert(!tracking::cloudLayoutValid(cloud));
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.height = 2; assert(!tracking::cloudLayoutValid(cloud));
    cloud.height = 1; cloud.width = std::numeric_limits<uint32_t>::max();
    assert(!tracking::cloudLayoutValid(cloud));
    assert(tracking::framesValid("body", "odom", "body", "map"));
    assert(!tracking::framesValid("odom", "odom", "body", "map"));
    assert(!tracking::framesValid("", "odom", "", "map"));
    assert(!tracking::framesValid("body", "map", "body", "map"));
}

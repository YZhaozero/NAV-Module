#include "IMU_Processing.h"
#include <stdexcept>

static void require(bool okay) {
  if (!okay) throw std::runtime_error("IMU initialization time-window regression failed");
}

int main(int argc, char **argv) {
  ImuProcess processor;
  processor.set_imu_init_frame_num(30);
  StatesGroup state;
  LidarMeasureGroup group;
  group.lio_vio_flg = LIO; group.last_lio_update_time = 1.0;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  // 200 Hz IMU requires more than one 100 ms LiDAR window to initialize.
  for (int window = 0; window < 3; ++window) {
    MeasureGroup measure;
    measure.lio_time = 1.1 + window * .1;
    for (int i = 0; i < 20; ++i) {
      auto imu = std::make_shared<sensor_msgs::msg::Imu>();
      imu->header.stamp = sec2Stamp(1.001 + window * .1 + i * .005);
      imu->linear_acceleration.z = 1.0;
      measure.imu.push_back(imu);
    }
    group.measures.clear(); group.measures.push_back(measure);
    group.pcl_proc_cur->clear();
    for (float offset : {0.0f, 30.0f, 80.0f, 99.0f}) {
      PointType point; point.x = 2; point.y = .5; point.z = -1; point.curvature = offset;
      group.pcl_proc_cur->push_back(point);
    }
    processor.Process2(group, state, output);
    require(std::abs(group.last_lio_update_time - measure.lio_time) < 1e-8);
    require(processor.imu_need_init == (window == 0));
    require(state.pos_end.norm() < 1e-8 && state.vel_end.norm() < 1e-8);
    if (window < 2) require(output->empty());
  }
  require(output->size() == 4);
  for (const auto &point : output->points)
    require(std::abs(point.x - 2) < 1e-6 && std::abs(point.z + 1) < 1e-6);

  // A 0.5 s sensor hole followed by 0.1 s of 1 rad/s rotation must integrate
  // exactly the covered 0.1 s. --without-gap-reset is a negative control that
  // reproduces the old retained integration anchor on the same processor.
  auto resumed = std::make_shared<sensor_msgs::msg::Imu>();
  resumed->header.stamp = sec2Stamp(1.8);
  resumed->linear_acceleration.z = 1.0;
  resumed->angular_velocity.z = 1.0;
  if (!(argc == 2 && std::string(argv[1]) == "--without-gap-reset"))
    processor.ResumeAfterGap(resumed);
  require(!processor.imu_need_init);
  const auto before_rotation = state.rot_end;
  group = LidarMeasureGroup();
  group.lio_vio_flg = LIO; group.last_lio_update_time = 1.8;
  MeasureGroup after;
  after.lio_time = 1.9;
  for (int i = 1; i <= 20; ++i) {
    auto imu = std::make_shared<sensor_msgs::msg::Imu>(*resumed);
    imu->header.stamp = sec2Stamp(1.8 + i * .005);
    after.imu.push_back(imu);
  }
  group.measures.push_back(after);
  for (float offset : {0.0f, 30.0f, 80.0f, 99.0f}) {
    PointType point; point.x = 2; point.y = .5; point.z = -1; point.curvature = offset;
    group.pcl_proc_cur->push_back(point);
  }
  processor.Process2(group, state, output);
  const auto change = before_rotation.transpose() * state.rot_end;
  const double yaw = std::atan2(change(1, 0), change(0, 0));
  std::cout << "post-gap yaw=" << yaw << " expected=0.1 rad" << std::endl;
  require(std::abs(yaw - .1) < 1e-7);
  require(output->size() == 4);
}

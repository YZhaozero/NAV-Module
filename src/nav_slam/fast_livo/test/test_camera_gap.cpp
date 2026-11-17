#include "LIVMapper.h"
#include "update_cadence.h"
#include <stdexcept>
#include <set>
#include <thread>

volatile std::sig_atomic_t g_stop_requested = 0;
static void check(bool okay, int line) {
  if (!okay) throw std::runtime_error("Camera-gap LIO continuity regression failed at line " + std::to_string(line));
}
#define require(condition) check((condition), __LINE__)

int main() {
  const char *arguments[] = {"camera_gap_regression", "--ros-args", "--params-file",
      ROOT_DIR "config/mid360_live.yaml", "-p", "common.img_en:=0",
      "-p", "pcd_save.pcd_save_en:=false"};
  rclcpp::init(8, arguments);
  auto calibration = std::make_shared<rclcpp::Node>("parameter_blackboard");
  calibration->declare_parameter("cam_model", "Pinhole");
  calibration->declare_parameter("cam_width", 64);
  calibration->declare_parameter("cam_height", 64);
  calibration->declare_parameter("scale", 1.0);
  for (const auto *key : {"cam_fx", "cam_fy"}) calibration->declare_parameter(key, 40.0);
  for (const auto *key : {"cam_cx", "cam_cy"}) calibration->declare_parameter(key, 32.0);
  for (const auto *key : {"cam_d0", "cam_d1", "cam_d2", "cam_d3"}) calibration->declare_parameter(key, 0.0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(calibration);
  std::thread serving([&]() { executor.spin(); });
  {
    auto node = std::make_shared<rclcpp::Node>("camera_gap_owner");
    LIVMapper mapper(node, "camera_gap_test");
    mapper.LidarMeasures.last_lio_update_time = 1.0;
    mapper.handleLIO();
    require(mapper.diagnostic_empty_lio_ == 1);
    require(mapper.diagnostic_odom_stamp_ < 0.0);
    std::unique_ptr<vk::AbstractCamera> camera(mapper.vio_manager->cam);
    mapper.slam_mode_ = LIVO; mapper.img_en = 1;
    auto &meas = mapper.LidarMeasures;
    meas.last_lio_update_time = 1.0;
    auto scan = [&](double stamp) {
      PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
      for (float ms : {0.0f, 50.0f, 99.0f, 100.4f}) {
        PointType point; point.x = 1; point.y = 0; point.z = -1; point.curvature = ms;
        cloud->push_back(point);
      }
      mapper.lid_raw_data_buffer.push_back(cloud); mapper.lid_header_time_buffer.push_back(stamp);
    };
    auto imu = [&](double stamp) {
      auto message = std::make_shared<sensor_msgs::msg::Imu>();
      message->header.stamp = sec2Stamp(stamp); message->linear_acceleration.z = 1;
      mapper.imu_buffer.push_back(message);
    };
    for (int i = 0; i < 3; ++i) scan(1.0 + i * .1);
    for (int i = 0; i <= 61; ++i) imu(1.0 + i * .005);
    size_t consumed = 0;
    for (int i = 1; i <= 3; ++i) {
      require(mapper.sync_packages(meas));
      require(meas.lio_vio_flg == LIO && meas.measures.size() == 1);
      require(std::abs(meas.measures.back().lio_time - (1.0 + i * .1)) < 1e-8);
      require(mapper.pending_vio_image_.empty());
      consumed += meas.pcl_proc_cur->size();
      // The IMU processor advances this after consuming the measurement.
      meas.last_lio_update_time = meas.measures.back().lio_time;
    }
    require(consumed == 11 && meas.pcl_proc_next->size() == 1);
    require(!mapper.sync_packages(meas)); // No extrapolation beyond sensor coverage.
    // Old or almost coincident images must not rewind the estimator or create
    // a tiny, underconstrained LiDAR fit. A separated fresh one resumes VIO.
    mapper.img_buffer.emplace_back(64, 64, CV_8UC3, cv::Scalar(11)); mapper.img_time_buffer.push_back(1.15);
    mapper.img_buffer.emplace_back(64, 64, CV_8UC3, cv::Scalar(22)); mapper.img_time_buffer.push_back(1.3004);
    mapper.img_buffer.emplace_back(64, 64, CV_8UC3, cv::Scalar(77)); mapper.img_time_buffer.push_back(1.35);
    scan(1.3); for (int i = 62; i <= 83; ++i) imu(1.0 + i * .005);
    require(mapper.sync_packages(meas));
    require(std::abs(meas.measures.back().lio_time - 1.35) < 1e-8);
    require(mapper.img_buffer.empty() && !mapper.pending_vio_image_.empty());
    meas.last_lio_update_time = meas.measures.back().lio_time;
    mapper.lid_raw_data_buffer.clear(); mapper.lid_header_time_buffer.clear(); mapper.imu_buffer.clear();
    require(mapper.sync_packages(meas)); // Pending VIO needs no new sensor input.
    require(meas.lio_vio_flg == VIO && meas.measures.size() == 1);
    require(std::abs(meas.measures.back().vio_time - 1.35) < 1e-8);
    require(meas.measures.back().img.at<cv::Vec3b>(0, 0)[0] == 77);
    require(mapper.pending_vio_image_.empty());
    // Real 15 Hz images have an arbitrary phase relative to 10 Hz LiDAR.
    // Exercise the actual image callback and incremental sensor arrival, not
    // just a prefilled queue with image stamps exactly on 100 ms boundaries.
    for (double epoch : {10., 1792316860.}) {
      for (double phase : {.013, .0, .001, .003, .005, .025, .033, .047, .065}) {
        for (unsigned adaptive_level : {0u, 3u, 7u}) {
        for (double delivery_delay : {.09, .13, .8}) {
          const bool camera_gap = epoch == 10. && phase == .047 && delivery_delay == .13;
          mapper.lid_raw_data_buffer.clear(); mapper.lid_header_time_buffer.clear();
          mapper.imu_buffer.clear(); mapper.img_buffer.clear(); mapper.img_time_buffer.clear();
          mapper.pending_vio_image_.release();
          mapper.lio_min_interval_ = .095; mapper.vio_min_interval_ = .2;
          mapper.adaptive_load_.enable(false); mapper.adaptive_load_.enable(adaptive_level != 0);
          for (unsigned window = 0; window < adaptive_level * 2; ++window)
            mapper.adaptive_load_.observe(1., .95, .3, 10);
          const double visual_interval = mapper.effectiveVioInterval();
          mapper.last_vio_update_time_ = -1.; mapper.last_timestamp_img = -1.;
          mapper.last_timestamp_lidar = epoch;
          mapper.img_time_offset = .1; mapper.exposure_time_init = 0.;
          mapper.raw_lidar_end_cache_.invalidate(); mapper.carried_lidar_end_cache_.invalidate();
          meas.last_lio_update_time = epoch; meas.pcl_proc_next->clear();
          size_t lio_updates = 0, vio_updates = 0, produced_points = 0;
          std::set<int> point_ids, imu_ids;
          double previous_vio = -1., max_vio_gap = 0.;
          int next_scan = 0, next_image = 0;
          for (int tick = 0; tick <= 2500; ++tick) {
            const double now = tick * .001;
            if (tick % 5 == 0) imu(epoch + now);
            while (next_scan * .1 + .105 <= now + 1e-9) {
              PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
              for (float ms : {0.0f, 50.0f, 99.0f, 100.4f}) {
                PointType point; point.x = ++produced_points;
                point.y = 0; point.z = -1; point.curvature = ms;
                cloud->push_back(point);
              }
              mapper.lid_raw_data_buffer.push_back(cloud);
              mapper.lid_header_time_buffer.push_back(epoch + next_scan * .1);
              mapper.last_timestamp_lidar = epoch + next_scan * .1;
              ++next_scan;
            }
            while (phase + next_image / 15. + delivery_delay <= now + 1e-9) {
              if (camera_gap && phase + next_image / 15. + .1 >= .9 &&
                  phase + next_image / 15. + .1 <= 1.5) {
                ++next_image;
                continue;
              }
              auto image = std::make_shared<sensor_msgs::msg::Image>();
              image->header.stamp = sec2Stamp(epoch + phase + next_image / 15.);
              image->height = 64; image->width = 64; image->step = 64 * 3;
              image->encoding = "bgr8"; image->data.resize(64 * 64 * 3, 77);
              mapper.img_cbk(image);
              ++next_image;
            }
            for (int pending = 0; pending < 6 && mapper.sync_packages(meas); ++pending) {
              const auto &group = meas.measures.back();
              if (meas.lio_vio_flg == LIO) {
                require(group.lio_time - meas.last_lio_update_time >= .005 - 1e-6);
                require(group.lio_time - meas.last_lio_update_time <= .1 + 1e-6);
                for (const auto &point : meas.pcl_proc_cur->points)
                  require(point_ids.insert(static_cast<int>(point.x)).second);
                for (const auto &message : group.imu)
                  require(imu_ids.insert(std::lround((stamp2Sec(message->header.stamp) - epoch) * 200)).second);
                meas.last_lio_update_time = group.lio_time;
                ++lio_updates;
              } else {
                require(std::abs(group.vio_time - meas.last_lio_update_time) < 1e-6);
                if (previous_vio >= 0.) {
                  const double gap = group.vio_time - previous_vio;
                  require(gap >= visual_interval - 1e-6); max_vio_gap = std::max(max_vio_gap, gap);
                }
                previous_vio = group.vio_time;
                ++vio_updates;
              }
            }
            if (camera_gap && tick == 1500) {
              // 100 ms scans arrive after their last point, and an arbitrary
              // cut phase can retain the tail until the next complete scan.
              std::cout << "camera gap state age=" << epoch + now - meas.last_lio_update_time << std::endl;
              require(meas.last_lio_update_time >= epoch + 1.3 - 1e-6);
            }
          }
          size_t retained_points = meas.pcl_proc_next->size();
          for (const auto &cloud : mapper.lid_raw_data_buffer) retained_points += cloud->size();
          std::cout << "15Hz phase=" << phase << " delay=" << delivery_delay
                    << " adaptive_level=" << adaptive_level << " camera_gap=" << camera_gap
                    << " epoch=" << epoch << " LIO=" << lio_updates << " VIO=" << vio_updates
                    << " max_vio_gap=" << max_vio_gap << " points=" << point_ids.size()
                    << "+" << retained_points << "/" << produced_points << std::endl;
          if (delivery_delay < .5) {
          require(vio_updates >= (adaptive_level ? 3 : 8) && vio_updates <= 13 &&
                  max_vio_gap <= (camera_gap ? 1.3 : visual_interval + .2) + 1e-6);
          require(previous_vio > epoch + 2.5 - visual_interval - .2);
          } else {
            // Late images cannot correct a newer state without historical
            // smoothing. LIO must continue with all points/IMU, never rewind.
            require(vio_updates==0);
          }
          require(lio_updates >= 23 && lio_updates <= 32);
          require(point_ids.size() + retained_points == produced_points);
          // A stamp exactly on the mathematical cut can remain just after it
          // due to double rounding. Check actual samples, including the future
          // queue, rather than rounding the cut to an expected message count.
          for (const auto &message : mapper.imu_buffer) {
            require(stamp2Sec(message->header.stamp) > meas.last_lio_update_time);
            require(imu_ids.insert(std::lround((stamp2Sec(message->header.stamp) - epoch) * 200)).second);
          }
          require(imu_ids.size() == 500); // Initial epoch sample precedes the first interval.
          for (int index = 1; index <= 500; ++index) require(imu_ids.count(index) == 1);
        }
      }
    }
    }
    mapper.adaptive_load_.enable(false);
    // An aligned 20 Hz camera must not force 20 costly LiDAR fits per second.
    // Start at the first full 100 ms state; arbitrary phases are covered above.
    // All 10 Hz scans and their IMU intervals remain in the estimator.
    for (double epoch : {10., 1792164000.}) {
    mapper.lid_raw_data_buffer.clear(); mapper.lid_header_time_buffer.clear();
    mapper.imu_buffer.clear(); mapper.img_buffer.clear(); mapper.img_time_buffer.clear();
    mapper.lio_min_interval_ = .095; mapper.vio_min_interval_ = .2;
    mapper.last_vio_update_time_ = -1.;
    mapper.raw_lidar_end_cache_.invalidate(); mapper.carried_lidar_end_cache_.invalidate();
    meas.last_lio_update_time = epoch; meas.pcl_proc_next->clear();
    for (int i = 0; i < 10; ++i) scan(epoch + i * .1);
    for (int i = 0; i <= 201; ++i) imu(epoch + i * .005);
    for (int i = 2; i <= 20; ++i) {
      mapper.img_buffer.emplace_back(64, 64, CV_8UC3, cv::Scalar(i));
      mapper.img_time_buffer.push_back(epoch + i * .05);
    }
    size_t lio_updates = 0, vio_updates = 0, points = 0, imus = 0;
    double previous_vio = -1.;
    for (int cycle = 0; cycle < 40 && mapper.sync_packages(meas); ++cycle) {
      const auto &group = meas.measures.back();
      if (meas.lio_vio_flg == LIO) {
        require(group.lio_time - meas.last_lio_update_time >= .095 - 1e-6);
        require(group.lio_time - meas.last_lio_update_time <= .1 + 1e-6);
        points += meas.pcl_proc_cur->size(); imus += group.imu.size(); ++lio_updates;
        meas.last_lio_update_time = group.lio_time;
      } else {
        require(std::abs(group.vio_time - meas.last_lio_update_time) < 1e-8);
        require(previous_vio < 0. || group.vio_time - previous_vio >= .2 - 1e-6);
        previous_vio = group.vio_time; ++vio_updates;
      }
    }
    std::cout << "cadence: LIO=" << lio_updates << " VIO=" << vio_updates
              << " points=" << points << " pending=" << meas.pcl_proc_next->size()
              << " IMU=" << imus << " time=" << meas.last_lio_update_time << std::endl;
    require(lio_updates == 10 && vio_updates == 5);
    require(points == 39 && meas.pcl_proc_next->size() == 1);
    require(imus >= 199 && imus <= 200);
    require(std::abs(meas.last_lio_update_time - (epoch + 1.)) < 1e-6);
    }
    // Exercise the real callback too: a gap must invalidate the propagated
    // pose and restart the measurement clock at the first covered sample.
    mapper.imu_en = true;
    mapper.last_timestamp_lidar = 100.;
    mapper.last_timestamp_imu = 100.;
    mapper.ekf_finish_once = true;
    mapper.state_update_flg = true;
    auto resumed = std::make_shared<sensor_msgs::msg::Imu>();
    resumed->header.stamp = sec2Stamp(100.5);
    resumed->linear_acceleration.z = 1.;
    mapper.imu_cbk(resumed);
    require(std::abs(mapper.LidarMeasures.last_lio_update_time - 100.5) < 1e-8);
    require(mapper.imu_buffer.size() == 1);
    require(!mapper.ekf_finish_once && !mapper.state_update_flg);
    require(mapper.lid_raw_data_buffer.empty() && mapper.img_buffer.empty());
    // DDS / SensorInbox may deliver an old LiDAR frame after the reset.
    // Clear the fully expired frame even when no fresh frame has arrived yet.
    mapper.last_timestamp_img = -1.; mapper.last_vio_update_time_ = -1.;
    scan(100.1);
    require(!mapper.sync_packages(meas));
    require(mapper.lid_raw_data_buffer.empty() && mapper.lid_header_time_buffer.empty());
    // Preserve the measured tail of a frame crossing the new IMU boundary.
    scan(100.45); scan(100.55);
    for (int i = 1; i <= 40; ++i) imu(100.5 + i * .005);
    require(mapper.sync_packages(meas));
    require(meas.lio_vio_flg == LIO);
    require(meas.pcl_proc_cur->size() == 4 && meas.pcl_proc_next->size() == 3);
    for (const auto &point : meas.pcl_proc_cur->points)
      require(point.curvature >= 0.f && point.curvature < 100.f);
    for (const auto &point : meas.pcl_proc_next->points) require(point.curvature >= 0.f);
    require(mapper.lid_raw_data_buffer.empty());
    // The geometry-only mode uses the same acquired-time boundary, including
    // the last point of its whole scan and excluding the missing prefix.
    mapper.slam_mode_ = ONLY_LIO; mapper.img_en = 0;
    mapper.last_timestamp_lidar = 200.; mapper.last_timestamp_imu = 200.;
    resumed->header.stamp = sec2Stamp(200.5);
    mapper.imu_cbk(resumed);
    scan(200.1);
    require(!mapper.sync_packages(meas));
    require(mapper.lid_raw_data_buffer.empty());
    scan(200.45);
    for (int i = 1; i <= 40; ++i) imu(200.5 + i * .005);
    mapper.last_timestamp_imu = 200.7;
    require(mapper.sync_packages(meas));
    require(meas.pcl_proc_cur->size() == 3);
    require(std::abs(meas.measures.back().lio_time - 200.5504) < 1e-6);
    for (const auto &point : meas.pcl_proc_cur->points) require(point.curvature >= 0.f);
  }
    require(!fast_livo_schedule::imuGapIsFatal(.25, 1));
    require(!fast_livo_schedule::imuGapIsFatal(1.9, 4));
    require(fast_livo_schedule::imuGapIsFatal(2.5, 1));
    require(fast_livo_schedule::imuGapIsFatal(.25, 5));
    std::cout << "imu gap classification: sustained holes require a new tracking segment" << std::endl;
  executor.cancel(); serving.join(); rclcpp::shutdown();
}

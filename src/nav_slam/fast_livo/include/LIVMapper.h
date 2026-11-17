#include "async_rgb_preview.h"
/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vikit/camera_loader.h>
#include <csignal>
#include "navigation_output.h"
#include "bounded_buffers.h"
#include "async_state_log.h"
#include "sensor_inbox.h"
#include "sensor_transport.h"
#include "lidar_slice_end_cache.h"
#include "adaptive_load.h"
#include "async_path_preview.h"
#include "async_pose_output.h"
#include <std_msgs/msg/string.hpp>
#include <iomanip>
#include <sstream>

// Ctrl+C / SIGTERM 只置位, 不直接 shutdown(见 main.cpp)。
// run() 轮询它, 正常退出后才 savePCD(), 避免 shutdown 竞态导致地图不落盘。
extern volatile std::sig_atomic_t g_stop_requested;

class LIVMapper
{
public:
  // Declared before subscription handles so it is destroyed after them.
  std::unique_ptr<SensorTransport> sensor_transport_, imu_sensor_transport_;
  LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &nh, image_transport::ImageTransport &it_);
  void initializeComponents(rclcpp::Node::SharedPtr &node);
  void initializeFiles();
  void writeDebugState(bool after);
  void run(rclcpp::Node::SharedPtr &node);
  void publishDiagnostics();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  std::string diagnostic_instance_, wait_reason_ = "starting";
  double diagnostic_last_ = 0, diagnostic_busy_ = 0, diagnostic_callbacks_ = 0;
  double diagnostic_sync_ = 0;
  double adaptive_compute_window_s_ = 0.;
  std::size_t adaptive_lio_window_cycles_ = 0;
  fast_livo_schedule::AdaptiveLoad adaptive_load_;
  std::size_t lio_query_budget_ = 4000;
  double effectiveVioInterval() const { return adaptive_load_.vioInterval(vio_min_interval_); }
  double diagnostic_lio_ = 0, diagnostic_vio_ = 0, diagnostic_cycle_ = 0;
  double diagnostic_imu_process_ = 0, diagnostic_lio_downsample_ = 0;
  double diagnostic_lio_fit_ = 0, diagnostic_lio_map_ = 0, diagnostic_lio_output_ = 0;
  double diagnostic_vio_fit_ = 0, diagnostic_vio_output_ = 0;
  double diagnostic_odom_stamp_ = -1.;
  std::unique_ptr<AsyncPoseOutput> pose_output_, imu_pose_output_;
  bool propagated_tf_ = false;
  double imu_propagated_stamp_ = -1.;
  double imu_acc_scale_ = 1.;
  std::string imu_input_frame_;
  bool tracking_suspended_ = false;
  std::string tracking_fault_;
  void suspendTracking(const std::string &reason);
  double diagnostic_pose_submit_ = 0.;
  size_t diagnostic_empty_lio_ = 0;
  double raw_lidar_stamp_ = 0, raw_imu_stamp_ = 0, raw_image_stamp_ = 0;
  double raw_lidar_received_ = 0, raw_imu_received_ = 0, raw_image_received_ = 0;
  size_t diagnostic_sequence_ = 0, diagnostic_cycles_ = 0;
  size_t diagnostic_lio_cycles_ = 0, diagnostic_vio_cycles_ = 0;
  size_t imu_gap_total_ = 0, imu_gap_window_count_ = 0;
  double imu_gap_window_begin_ = -1.;
  size_t diagnostic_lidar_drops_ = 0, diagnostic_image_drops_ = 0;
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void trimLidarBuffer();
  void writePCDChunk(const PointCloudXYZRGB &cloud);
  void writePCDChunk(const PointCloudXYZI &cloud);
  void processImu();

  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  template <class Message>
  void receiveSensor(SensorInbox::Stream stream, std::size_t bytes,
                     std::shared_ptr<const Message> msg,
                     void (LIVMapper::*consume)(const std::shared_ptr<const Message> &)) {
    const double received = node->get_clock()->now().seconds();
    sensor_inbox_.push(stream, bytes, [this, stream, msg, consume, received] {
      (this->*consume)(msg);
      if (stream == SensorInbox::Lidar) raw_lidar_received_ = received;
      else if (stream == SensorInbox::Imu) raw_imu_received_ = received;
      else raw_image_received_ = received;
    }, stamp2Sec(msg->header.stamp), received);
  }
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void initializeAsyncRgbPreview();
  bool asyncRgbEnabled() const;
  void publishAsyncRgbPreview();
  std::optional<fast_livo_rgb::PinholeParameters> rgb_camera_snapshot_;
  fast_livo_rgb::GeometryWindow rgb_geometry_window_;
  std::unique_ptr<fast_livo_rgb::AsyncRgbPreview> rgb_preview_;
  int rgb_preview_counter_ = 1;

  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pmavros_pose_publisherubOdomAftMapped);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath);
  void readParameters(rclcpp::Node::SharedPtr &node);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;

  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  std::string world_frame_, body_frame_;
  std::unique_ptr<NavigationOutput> navigation_output_;
  std::unique_ptr<fast_livo_preview::AsyncPathPreview> path_preview_;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  int outlier_threshold;
  int max_points_per_voxel = 200;
  int max_visual_frames = 150;
  int pcd_max_points_ = 1000000;
  int pcd_chunk_index_ = 0;
  std::uint64_t pcd_colmap_index_ = 0;
  int pcd_scan_count_ = 0;
  std::string pcd_directory_;
  std::string pcd_session_;
  cv::Mat pending_vio_image_;
  double pending_vio_time_ = 0.0;
  double lio_min_interval_ = .005, vio_min_interval_ = 0.;
  double input_holdback_s_ = 0.;
  double last_vio_update_time_ = -1.;
  std::uint64_t cadence_image_drops_ = 0;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  fast_livo_schedule::LidarSliceEndCache<PointCloudXYZI::ConstPtr> raw_lidar_end_cache_;
  fast_livo_schedule::LidarSliceEndCache<PointCloudXYZI::ConstPtr> carried_lidar_end_cache_;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  int IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  MapPreview rgb_pending_{.03, 120000};
  double rgb_pending_start_ = -1.;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_visual_pos, fout_lidar_pos, fout_points;
  std::unique_ptr<AsyncStateLog> state_log_;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  std::shared_ptr<rclcpp::SubscriptionBase> sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  rclcpp::Node::SharedPtr node;
  SensorInbox sensor_inbox_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr input_diagnostics_callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr input_diagnostics_timer_;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif

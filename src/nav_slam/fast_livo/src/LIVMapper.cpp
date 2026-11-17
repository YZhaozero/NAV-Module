/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "frame_logging.h"
#include "parallelism.h"
#include "update_cadence.h"
#include "lidar_slice.h"
#include "sensor_receiver.h"
#include "cloud_publication.h"
#include <filesystem>
#include <chrono>
#include <limits>
#include <unistd.h>
#include <vikit/camera_loader.h>

using namespace Sophus;
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name)
    : node(std::make_shared<rclcpp::Node>(node_name)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  navigation_output_ = std::make_unique<NavigationOutput>(this->node);
  path.header.stamp = this->node->now();
  path.header.frame_id = world_frame_;
}

LIVMapper::~LIVMapper() {
  if (pose_output_ && !pose_output_->stop())
    RCLCPP_ERROR(node->get_logger(), "Pose transport did not drain before shutdown; map saving remains independent");
  if (imu_pose_output_ && !imu_pose_output_->stop())
    RCLCPP_ERROR(node->get_logger(), "Propagated pose output did not drain before shutdown");
  if (rgb_preview_) rgb_preview_->stop();
  // The callback owns a publisher copy, but stop its worker before tearing down
  // the remaining ROS context and mapper resources.
  if (path_preview_) path_preview_->stop();
}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // navigation.launch supplies the calibration atomically in this process.
  // Legacy launches can still use parameter_blackboard, but navigation no longer
  // depends on cross-process service discovery during estimator construction.
  for (const auto &entry : this->node->get_node_parameters_interface()->get_parameter_overrides())
  {
    if (entry.first.rfind("cam_", 0) == 0 || entry.first == "scale")
      this->node->declare_parameter(entry.first, entry.second);
  }
  world_frame_ = this->node->declare_parameter<std::string>("navigation.world_frame", "camera_init");
  body_frame_ = this->node->declare_parameter<std::string>("navigation.body_frame", "aft_mapped");
  propagated_tf_ = this->node->declare_parameter<bool>("navigation.propagated_tf", false);
  imu_acc_scale_ = this->node->declare_parameter<double>("common.imu_acc_scale", 1.);
  imu_input_frame_ = this->node->declare_parameter<std::string>("common.imu_frame", "");
  if (!std::isfinite(imu_acc_scale_) || imu_acc_scale_ <= 0.)
    throw std::invalid_argument("IMU acceleration scale must be positive and finite");
  lio_min_interval_ = this->node->declare_parameter<double>("navigation.lio_min_interval_s", .005);
  vio_min_interval_ = this->node->declare_parameter<double>("navigation.vio_min_interval_s", 0.);
  fast_livo_schedule::validate(lio_min_interval_, vio_min_interval_);
  input_holdback_s_ = this->node->declare_parameter<double>("navigation.input_holdback_s", 0.);
  if (!std::isfinite(input_holdback_s_) || input_holdback_s_ < 0.)
    throw std::invalid_argument("Input holdback must be finite and nonnegative");
  if (this->node->get_parameter("use_sim_time").as_bool()) input_holdback_s_ = 0.;
  adaptive_load_.enable(this->node->declare_parameter<bool>("navigation.adaptive_load", false));
  lio_query_budget_ = static_cast<std::size_t>(std::max<int64_t>(0,
      this->node->declare_parameter<int>("navigation.lio_max_match_queries", 4000)));
  // declare parameters
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  this->node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<int>("vio.max_points_per_voxel", 200);
  this->node->declare_parameter<int>("vio.max_visual_frames", 150);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->node->declare_parameter<bool>("uav.gravity_align_en", false);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  this->node->declare_parameter<int>("imu.imu_int_frame", 30);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<int>("publish.pub_scan_num", 1);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("vio.max_points_per_voxel", max_points_per_voxel);
  this->node->get_parameter("vio.max_visual_frames", max_visual_frames);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.type", pcd_save_type);
  pcd_max_points_ = this->node->declare_parameter<int>("pcd_save.max_points_per_file", 1000000);
  pcd_directory_ = this->node->declare_parameter<std::string>("pcd_save.directory", DEBUG_FILE_DIR("pcd"));
  if (max_visual_frames < 0 || max_points_per_voxel < 1 || pcd_max_points_ < 1 || pcd_save_interval == 0)
    throw std::invalid_argument("invalid visual/PCD memory limits");
  RCLCPP_INFO(this->node->get_logger(), "Memory policy v1: visual_age=%d voxel_points=%d input_images=8 input_lidar=30 input_imu=4000 pcd_points=%d",
              max_visual_frames, max_points_per_voxel, pcd_max_points_);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node)
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->max_points_per_voxel = max_points_per_voxel;
  vio_manager->max_visual_frames = max_visual_frames;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles()
{
  RCLCPP_INFO(this->node->get_logger(), "Runtime log directory: %s", fast_livo_logs::sessionDirectory().c_str());
  if (pcd_save_en) {
    pcd_session_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "_" + std::to_string(getpid());
    pcd_directory_ = (std::filesystem::path(pcd_directory_) / pcd_session_).string();
    std::filesystem::create_directories(pcd_directory_);
    RCLCPP_INFO(this->node->get_logger(), "PCD session directory: %s", pcd_directory_.c_str());
  }
  // Optional exports prepare their directories once at startup. The former
  // source-tree shell helper removed earlier COLMAP output and required ROS 1.
  const auto prepare_export = [this](const char *relative) {
    std::error_code error;
    const auto directory = DEBUG_FILE_DIR(relative);
    std::filesystem::create_directories(directory, error);
    if (error) RCLCPP_WARN(this->node->get_logger(), "Optional export disabled: cannot create %s: %s",
                           directory.c_str(), error.message().c_str());
    return !error;
  };
  if (colmap_output_en) {
    colmap_output_en = prepare_export("Colmap/sparse/0") && prepare_export("Colmap/images");
    if (colmap_output_en) fout_points.open(DEBUG_FILE_DIR("Colmap/sparse/0/points3D.txt"), std::ios::out);
  }
  if (pose_output_en) pose_output_en = prepare_export("result");
  if (plot_time + frame_cnt / 2 * 0.1 >= 0. && !prepare_export("ref_cur_combine"))
    plot_time = -std::numeric_limits<double>::infinity();
  if(pcd_save_en) fout_lidar_pos.open(pcd_directory_ + "/lidar_poses.txt", std::ios::out);
  if(img_save_en) {
    img_save_en = prepare_export("image");
    if (img_save_en) fout_visual_pos.open(DEBUG_FILE_DIR("image/image_poses.txt"), std::ios::out);
  }
  state_log_ = std::make_unique<AsyncStateLog>(std::array<std::string,2>{
      DEBUG_FILE_DIR("mat_pre.txt"), DEBUG_FILE_DIR("mat_out.txt")});
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  image_transport::ImageTransport it(this->node);
  sensor_transport_ = std::make_unique<SensorTransport>(this->node);
  // IMU must also have its own receive socket, not just a callback executor.
  imu_sensor_transport_ = std::make_unique<SensorTransport>(this->node, "_imu_input");
  const auto &sensor_node = sensor_transport_->node();
  const auto &imu_node = imu_sensor_transport_->node();
  sensor_callback_group_ = sensor_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  imu_callback_group_ = imu_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  // Diagnostic output must not put a writer back onto the sensor participant,
  // nor wait for that writer on either receive executor.
  input_diagnostics_callback_group_ = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = sensor_callback_group_;
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;
  // Summarize reception where the messages already exist. The UI need not
  // subscribe to another 40 MB/s of images plus 200 Hz IMU just to inspect Header.
  input_diagnostics_pub_ = this->node->create_publisher<std_msgs::msg::String>("/fast_livo/input_diagnostics", 5);
  input_diagnostics_timer_ = this->node->create_wall_timer(1s, [this] {
    const auto input = sensor_inbox_.stats();
    std::ostringstream out;
    out << std::setprecision(16) << "{\"schema\":1,\"instance\":" << std::quoted(diagnostic_instance_)
        << ",\"stamp\":" << this->node->get_clock()->now().seconds() << ",\"inputs\":{";
    const char *names[] = {"lidar", "imu", "image"};
    for (unsigned i=0; i<3; ++i) {
      if (i) out << ',';
      out << std::quoted(names[i]) << ":{\"stamp\":" << input.stamp[i]
          << ",\"received_at\":" << input.received_at[i] << ",\"count\":" << input.received[i]
          << ",\"interval_s\":" << input.interval[i]
          << ",\"backwards\":" << (input.backwards[i] ? "true" : "false") << '}';
    }
    out << "}}";
    std_msgs::msg::String message; message.data = out.str(); input_diagnostics_pub_->publish(message);
  }, input_diagnostics_callback_group_);
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = sensor_node->create_subscription<livox_ros_driver2::msg::CustomMsg>(SensorTransport::resolve(this->node, lid_topic), rclcpp::SensorDataQoS().keep_last(10),
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
          receiveSensor(SensorInbox::Lidar, sizeof(*msg) + msg->header.frame_id.size() + msg->points.size()*sizeof(msg->points[0]), msg, &LIVMapper::livox_pcl_cbk);
        }, input_options);
  } else {
    sub_pcl = sensor_node->create_subscription<sensor_msgs::msg::PointCloud2>(SensorTransport::resolve(this->node, lid_topic), rclcpp::SensorDataQoS().keep_last(10),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          receiveSensor(SensorInbox::Lidar, sizeof(*msg) + msg->header.frame_id.size() + msg->data.size(), msg, &LIVMapper::standard_pcl_cbk);
        }, input_options);
  }
  // MID-360 publishes IMU reliably. A deep best-effort reader queue cannot
  // recover transport losses while this process is stalled; request repair
  // of those samples instead of turning local scheduling into an IMU hole.
  sub_imu = imu_node->create_subscription<sensor_msgs::msg::Imu>(SensorTransport::resolve(this->node, imu_topic), rclcpp::SensorDataQoS().keep_last(2000).reliable(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        receiveSensor(SensorInbox::Imu, sizeof(*msg) + msg->header.frame_id.size(), msg, &LIVMapper::imu_cbk);
      }, imu_options);
  sub_img = sensor_node->create_subscription<sensor_msgs::msg::Image>(SensorTransport::resolve(this->node, img_topic), rclcpp::SensorDataQoS().keep_last(4),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        receiveSensor(SensorInbox::Image, sizeof(*msg) + msg->header.frame_id.size() + msg->encoding.size() + msg->data.size(), msg, &LIVMapper::img_cbk);
      }, input_options);

  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  path_preview_ = std::make_unique<fast_livo_preview::AsyncPathPreview>(
      [publisher = pubPath, logger = this->node->get_logger(), clock = this->node->get_clock()]
      (nav_msgs::msg::Path::UniquePtr message) {
        try {
          publisher->publish(std::move(message));
        } catch (const std::exception &error) {
          // A disconnected visualization must not terminate the estimator.
          RCLCPP_WARN_THROTTLE(logger, *clock, 5000, "Path preview publication failed: %s", error.what());
        }
      },
      [publisher = pubPath] {
        try {
          return publisher->get_subscription_count() + publisher->get_intra_process_subscription_count() > 0;
        } catch (const std::exception &) {
          return false;
        }
      });
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  initializeAsyncRgbPreview();
  if (!propagated_tf_) pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame()
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment()
{
  if (!p_imu->imu_need_init && !gravity_align_finished)
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu()
{
  const double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;
  diagnostic_imu_process_ = omp_get_wtime() - t0;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping()
{
  switch (LidarMeasures.lio_vio_flg)
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::writeDebugState(bool after)
{
  if (!state_log_) return;
  std::ostringstream line;
  line << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " "
       << euler_cur.transpose()*57.3 << " " << _state.pos_end.transpose() << " "
       << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
       << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time,0,0).transpose();
  if (after) line << " " << feats_undistort->points.size();
  line << '\n';
  state_log_->append(after ? 1 : 0, line.str());
}

void LIVMapper::handleVIO()
{
  euler_cur = RotMtoEuler(_state.rot_end);
  writeDebugState(false);

  if (!pcl_w_wait_pub || pcl_w_wait_pub->empty())
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }

  if (verboseFrameLogs()) std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1))
  {
    vio_manager->plot_flag = true;
  }
  else
  {
    vio_manager->plot_flag = false;
  }

  const double vio_start = omp_get_wtime();
  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);
  diagnostic_vio_fit_ = omp_get_wtime() - vio_start;
  // Count compute before asynchronous RGB publication releases its cloud.
  adaptive_compute_window_s_ += diagnostic_vio_fit_;
  const double output_start = omp_get_wtime();

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++)
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (!asyncRgbEnabled()) publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  writeDebugState(true);
  diagnostic_vio_output_ = omp_get_wtime() - output_start;
}

void LIVMapper::handleLIO()
{
  euler_cur = RotMtoEuler(_state.rot_end);
  writeDebugState(false);

  if (!feats_undistort || feats_undistort->empty())
  {
    ++diagnostic_empty_lio_;
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  double t_down = omp_get_wtime();
  diagnostic_lio_downsample_ = t_down - t0;

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited)
  {
    lidar_map_inited = true;
    // The seed world cloud is consumed only by the initial map build.
    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
    voxelmap_manager->feats_down_world_ = feats_down_world;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->adaptive_query_fraction_ = adaptive_load_.queryFraction();
  voxelmap_manager->adaptive_query_budget_ = adaptive_load_.enabled() ? lio_query_budget_ : 0;
  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;

  double t2 = omp_get_wtime();
  diagnostic_lio_fit_ = t2 - t1;

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en)
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend)
    {
      evoFile.open(DEBUG_FILE_DIR("result/" + seq_name + ".txt"), std::ios::out | std::ios::app);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    else
    {
      evoFile.open(DEBUG_FILE_DIR("result/" + seq_name + ".txt"), std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  const double pose_submit_start = omp_get_wtime();
  publish_odometry(pubOdomAftMapped);
  diagnostic_pose_submit_ = omp_get_wtime() - pose_submit_start;

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  const M3D world_from_lidar = _state.rot_end * extR;
  const M3D rotation_covariance = _state.cov.block<3, 3>(0, 0);
  const M3D position_covariance = _state.cov.block<3, 3>(3, 3);
  #ifdef MP_EN
    #pragma omp parallel for num_threads(fast_livo_parallel::threadCount()) schedule(static)
  #endif
  for (int i = 0; i < static_cast<int>(world_lidar->points.size()); i++)
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = world_from_lidar * var * world_from_lidar.transpose() +
          (-point_crossmat) * rotation_covariance * (-point_crossmat).transpose() + position_covariance;
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  if (verboseFrameLogs()) std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;

  double t4 = omp_get_wtime();
  diagnostic_lio_map_ = t4 - t3;

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }

  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  #ifdef MP_EN
    #pragma omp parallel for num_threads(fast_livo_parallel::threadCount()) schedule(static)
  #endif
  for (int i = 0; i < size; i++)
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  // Published snapshots are immutable. Retain the same scan for the next
  // colour update without another dense-cloud copy.
  pcl_w_wait_pub = laserCloudWorld;

  // Full-field deskewed geometry in the IMU/body frame, not RGB-cropped world points.
  NavigationOutput::Cloud::Ptr body_cloud(new NavigationOutput::Cloud());
  body_cloud->resize(laserCloudFullRes->size());
  #ifdef MP_EN
    #pragma omp parallel for num_threads(fast_livo_parallel::threadCount()) schedule(static)
  #endif
  for (int i = 0; i < static_cast<int>(laserCloudFullRes->size()); ++i)
    RGBpointBodyLidarToIMU(&laserCloudFullRes->points[i], &body_cloud->points[i]);
  navigation_output_->publish(body_cloud, laserCloudWorld,
      sec2Stamp(LidarMeasures.last_lio_update_time), body_frame_, world_frame_);

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  // MAVROS, TF and odometry share the same immutable pose in the output worker.

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  if (verboseFrameLogs()) {
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  writeDebugState(true);
  diagnostic_lio_output_ = omp_get_wtime() - t4;
}

void LIVMapper::writePCDChunk(const PointCloudXYZRGB &cloud)
{
  const auto filename = pcd_directory_ + "/" + std::to_string(pcd_chunk_index_++) + "_rgb.pcd";
  if (pcl::PCDWriter().writeBinary(filename, cloud) < 0)
    throw std::runtime_error("PCD write failed: " + filename);
  if (colmap_output_en) {
    // Chunked points keep IDs unique without retaining a second global cloud.
    for (const auto &point : cloud) {
      fout_points << pcd_colmap_index_++ << " " << std::setprecision(9)
                  << point.x << " " << point.y << " " << point.z << " "
                  << int(point.r) << " " << int(point.g) << " " << int(point.b) << " 0\n";
    }
    if (!fout_points) throw std::runtime_error("COLMAP point output failed");
  }
}

void LIVMapper::writePCDChunk(const PointCloudXYZI &cloud)
{
  const auto filename = pcd_directory_ + "/" + std::to_string(pcd_chunk_index_++) + "_intensity.pcd";
  if (pcl::PCDWriter().writeBinary(filename, cloud) < 0)
    throw std::runtime_error("PCD write failed: " + filename);
}

void LIVMapper::savePCD()
{
  if (!pcd_save_en) return;
  if (!pcl_wait_save->empty()) {
    writePCDChunk(*pcl_wait_save);
    PointCloudXYZRGB().swap(*pcl_wait_save);
  }
  if (!pcl_wait_save_intensity->empty()) {
    writePCDChunk(*pcl_wait_save_intensity);
    PointCloudXYZI().swap(*pcl_wait_save_intensity);
  }
  pcd_scan_count_ = 0;
}

volatile std::sig_atomic_t g_stop_requested = 0;

void LIVMapper::run(rclcpp::Node::SharedPtr &node)
{
  diagnostics_pub_ = this->node->create_publisher<std_msgs::msg::String>("/fast_livo/diagnostics", 5);
  diagnostic_instance_ = std::to_string(this->node->get_clock()->now().nanoseconds());
  diagnostic_last_ = omp_get_wtime();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->node);
  SensorReceiver receiver(sensor_transport_->node(), sensor_callback_group_, sensor_inbox_);
  SensorReceiver imu_receiver(imu_sensor_transport_->node(), imu_callback_group_, sensor_inbox_);
  SensorReceiver input_diagnostics_receiver(this->node, input_diagnostics_callback_group_, sensor_inbox_);
  rclcpp::Rate rate(1000);
  while (rclcpp::ok() && !g_stop_requested)
  {
    const double spin_start = omp_get_wtime();
    // Services and IMU propagation still share estimator ownership. The separate
    // receive executor only enqueues immutable messages; no EKF/IMU state races.
    executor.spin_all(std::chrono::milliseconds(5));
    const double receive_deadline = omp_get_wtime() + .005;
    while (auto consume = sensor_inbox_.take()) {
      consume();
      if (omp_get_wtime() >= receive_deadline) break;
    }
    diagnostic_callbacks_ += omp_get_wtime() - spin_start;
    wait_reason_ = "other_sync_wait";
    const double sync_start = omp_get_wtime();
    const bool synchronized = sync_packages(LidarMeasures);
    diagnostic_sync_ += omp_get_wtime() - sync_start;
    if (!synchronized)
    {
      publishDiagnostics();
      rate.sleep();
      continue;
    }
    wait_reason_ = "processing";
    const double cycle_start = omp_get_wtime();
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
    diagnostic_cycle_ = omp_get_wtime() - cycle_start;
    diagnostic_busy_ += diagnostic_cycle_;
    ++diagnostic_cycles_;
    if (LidarMeasures.lio_vio_flg == LIO) {
      diagnostic_lio_ = diagnostic_cycle_;
      ++diagnostic_lio_cycles_;
      if (feats_undistort && !feats_undistort->empty()) {
        adaptive_compute_window_s_ += diagnostic_imu_process_ + diagnostic_lio_downsample_ +
            diagnostic_lio_fit_ + diagnostic_lio_map_;
        ++adaptive_lio_window_cycles_;
      }
    } else if (LidarMeasures.lio_vio_flg == VIO) {
      diagnostic_vio_ = diagnostic_cycle_;
      ++diagnostic_vio_cycles_;

    }
    wait_reason_ = "ready";
    publishDiagnostics();
  }
  imu_receiver.stop();
  receiver.stop();
  input_diagnostics_receiver.stop();
  savePCD();
}

void LIVMapper::publishDiagnostics()
{
  const double wall = omp_get_wtime(), elapsed = wall - diagnostic_last_;
  if (elapsed < 1.0) return;
  const double now = this->node->get_clock()->now().seconds();
  const double oldest = lid_header_time_buffer.empty() ? last_timestamp_lidar : lid_header_time_buffer.front();
  const auto input = sensor_inbox_.stats();
  const auto pose_output = pose_output_ ? pose_output_->stats() : AsyncPoseOutput::Stats{};
  const auto imu_pose_output = imu_pose_output_ ? imu_pose_output_->stats() : AsyncPoseOutput::Stats{};
  if (pose_output_) diagnostic_odom_stamp_ = pose_output.stamp;
  const double covered_backlog = LidarMeasures.last_lio_update_time < 0. ? 0. :
      std::max(0., std::min(last_timestamp_lidar, last_timestamp_imu) - LidarMeasures.last_lio_update_time);
  adaptive_load_.observe(elapsed, adaptive_compute_window_s_, covered_backlog, adaptive_lio_window_cycles_);
  std::ostringstream out;
  out << std::setprecision(16) << "{\"schema\":1,\"instance\":\"" << diagnostic_instance_
      << "\",\"sequence\":" << ++diagnostic_sequence_ << ",\"stamp\":" << now
      << ",\"wait_reason\":\"" << wait_reason_ << "\",\"lidar_topic\":" << std::quoted(lid_topic)
      << ",\"imu_topic\":" << std::quoted(imu_topic) << ",\"image_topic\":" << std::quoted(img_topic)
      << ",\"state_stamp\":" << LidarMeasures.last_lio_update_time
      << ",\"state_age_s\":" << now - LidarMeasures.last_lio_update_time
      << ",\"published_odom_stamp\":" << diagnostic_odom_stamp_
      << ",\"published_odom_age_s\":" << (diagnostic_odom_stamp_ < 0. ? -1. : now - diagnostic_odom_stamp_)
      << ",\"empty_lio_cycles\":" << diagnostic_empty_lio_
      << ",\"tracking_suspended\":" << (tracking_suspended_ ? "true" : "false")
      << ",\"tracking_fault\":" << std::quoted(tracking_fault_)
      << ",\"pose_output_pending\":" << pose_output.pending
      << ",\"pose_output_rejected\":" << pose_output.rejected
      << ",\"pose_output_failed\":" << (pose_output.failed ? "true" : "false")
      << ",\"pose_output_completed\":" << pose_output.completed
      << ",\"pose_publish_wall_s\":" << pose_output.last_publish_s
      << ",\"imu_pose_output_pending\":" << imu_pose_output.pending
      << ",\"imu_pose_output_failed\":" << (imu_pose_output.failed ? "true" : "false")
      << ",\"imu_pose_output_completed\":" << imu_pose_output.completed
      << ",\"imu_pose_publish_wall_s\":" << imu_pose_output.last_publish_s
      << ",\"published_imu_odom_stamp\":" << imu_pose_output.stamp
      << ",\"published_imu_odom_age_s\":" << (imu_pose_output.stamp < 0. ? -1. : now-imu_pose_output.stamp)
      << ",\"last_pose_submit_s\":" << diagnostic_pose_submit_
      << ",\"latest_lidar_s\":" << last_timestamp_lidar << ",\"latest_imu_s\":" << last_timestamp_imu
      << ",\"latest_image_s\":" << last_timestamp_img
      << ",\"lidar_queue\":" << lid_raw_data_buffer.size() << ",\"imu_queue\":" << imu_buffer.size()
      << ",\"image_queue\":" << img_buffer.size()
      << ",\"lidar_backlog_s\":" << std::max(0.0, last_timestamp_lidar - oldest)
      << ",\"state_backlog_s\":" << std::max(0.0, last_timestamp_lidar - LidarMeasures.last_lio_update_time)
      << ",\"lidar_drops\":" << diagnostic_lidar_drops_ + input.dropped[SensorInbox::Lidar]
      << ",\"image_drops\":" << diagnostic_image_drops_ + input.dropped[SensorInbox::Image]
      << ",\"input_lidar_queue\":" << input.count[SensorInbox::Lidar]
      << ",\"input_imu_queue\":" << input.count[SensorInbox::Imu]
      << ",\"input_image_queue\":" << input.count[SensorInbox::Image]
      << ",\"input_received_imu\":" << input.received[SensorInbox::Imu]
      << ",\"input_bytes\":" << input.bytes[0] + input.bytes[1] + input.bytes[2]
      << ",\"cadence_image_drops\":" << cadence_image_drops_
      << ",\"lio_min_interval_s\":" << lio_min_interval_ << ",\"vio_min_interval_s\":" << vio_min_interval_
      << ",\"input_holdback_s\":" << input_holdback_s_
      << ",\"adaptive_enabled\":" << (adaptive_load_.enabled() ? "true" : "false")
      << ",\"adaptive_level\":" << adaptive_load_.level()
      << ",\"adaptive_reason\":" << std::quoted(adaptive_load_.reason())
      << ",\"adaptive_changes\":" << adaptive_load_.changes()
      << ",\"adaptive_compute_ratio\":" << adaptive_load_.computeRatio()
      << ",\"adaptive_ready_backlog_s\":" << adaptive_load_.readyBacklog()
      << ",\"effective_vio_interval_s\":" << effectiveVioInterval()
      << ",\"lio_query_fraction\":" << adaptive_load_.queryFraction()
      << ",\"lio_match_queries\":" << voxelmap_manager->last_match_query_count_
      << ",\"lio_query_budget\":" << voxelmap_manager->adaptive_query_budget_
      << ",\"lio_full_query_fallbacks\":" << voxelmap_manager->adaptive_full_query_fallbacks_
      << ",\"lio_effective_matches\":" << voxelmap_manager->effct_feat_num_
      << ",\"busy_ratio\":" << (diagnostic_busy_ + diagnostic_callbacks_ + diagnostic_sync_) / elapsed
      << ",\"synchronization_wall_s\":" << diagnostic_sync_
      << ",\"callback_wall_s\":" << diagnostic_callbacks_ << ",\"processing_wall_s\":" << diagnostic_busy_
      << ",\"window_wall_s\":" << elapsed << ",\"cycles\":" << diagnostic_cycles_
      << ",\"state_log_dropped\":" << (state_log_ ? state_log_->dropped() : 0)
      << ",\"state_log_errors\":" << (state_log_ ? state_log_->errors() : 0)
      << ",\"imu_log_dropped\":" << p_imu->debugLogDropped()
      << ",\"imu_log_errors\":" << p_imu->debugLogErrors()
      << ",\"imu_gaps\":" << imu_gap_total_
      << ",\"lio_cycles\":" << diagnostic_lio_cycles_ << ",\"vio_cycles\":" << diagnostic_vio_cycles_
      << ",\"last_cycle_s\":" << diagnostic_cycle_ << ",\"last_lio_s\":" << diagnostic_lio_
      << ",\"last_vio_s\":" << diagnostic_vio_
      << ",\"last_imu_process_s\":" << diagnostic_imu_process_
      << ",\"last_lio_downsample_s\":" << diagnostic_lio_downsample_
      << ",\"last_lio_fit_s\":" << diagnostic_lio_fit_
      << ",\"last_lio_map_s\":" << diagnostic_lio_map_
      << ",\"last_lio_output_s\":" << diagnostic_lio_output_
      << ",\"last_vio_fit_s\":" << diagnostic_vio_fit_
      << ",\"last_vio_output_s\":" << diagnostic_vio_output_
      << ",\"vio_prepare_s\":" << vio_manager->last_prepare_s
      << ",\"vio_retrieve_s\":" << vio_manager->last_retrieve_s
      << ",\"vio_update_s\":" << vio_manager->last_visual_update_s
      << ",\"vio_generate_s\":" << vio_manager->last_generate_s
      << ",\"vio_plot_s\":" << vio_manager->last_plot_s
      << ",\"vio_observation_s\":" << vio_manager->last_observation_s
      << ",\"vio_reference_s\":" << vio_manager->last_reference_s
      << ",\"lio_input_points\":" << (feats_undistort ? feats_undistort->size() : 0)
      << ",\"lio_downsampled_points\":" << feats_down_size
      << ",\"lio_root_voxels\":" << voxelmap_manager->voxel_map_.size()
      << ",\"raw_lidar_stamp\":" << raw_lidar_stamp_ << ",\"raw_lidar_received\":" << raw_lidar_received_
      << ",\"raw_imu_stamp\":" << raw_imu_stamp_ << ",\"raw_imu_received\":" << raw_imu_received_
      << ",\"raw_image_stamp\":" << raw_image_stamp_ << ",\"raw_image_received\":" << raw_image_received_
      << ",\"lidar_offset_config_s\":" << lidar_time_offset << ",\"imu_offset_subtracted_s\":" << imu_time_offset
      << ",\"image_offset_added_s\":" << img_time_offset << ",\"exposure_offset_s\":" << exposure_time_init
      << ",\"driver_round_fix\":" << (ros_driver_fix_en ? "true" : "false") << "}";
  std_msgs::msg::String message; message.data = out.str(); diagnostics_pub_->publish(message);
  diagnostic_last_ = wall; diagnostic_busy_ = 0; diagnostic_callbacks_ = 0; diagnostic_sync_ = 0;
  adaptive_compute_window_s_ = 0.; adaptive_lio_window_cycles_ = 0;
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (tracking_suspended_ || p_imu->imu_need_init || !new_imu || !ekf_finish_once || !imu_prop_enable) return;
  std::lock_guard<std::mutex> lock(mtx_buffer_imu_prop);
  new_imu = false;
  if (state_update_flg) {
    imu_propagate = latest_ekf_state;
    imu_propagated_stamp_ = latest_ekf_time;
    while (!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time)
      prop_imu_buffer.pop_front();
    state_update_flg = false;
  }
  // A callback may consume a burst after a LIO update. Integrate every measured
  // sample, rather than applying only the newest sample across the whole burst.
  for (const auto &imu : prop_imu_buffer) {
    const double stamp = stamp2Sec(imu.header.stamp);
    if (stamp <= imu_propagated_stamp_) continue;
    prop_imu_once(imu_propagate, stamp - imu_propagated_stamp_,
        V3D(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z),
        V3D(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z));
    imu_propagated_stamp_ = stamp;
  }
  // Navigation needs a current 50 Hz pose, not another 200 Hz DDS/TF stream.
  // All measured IMU samples above are still integrated; only output is decimated.
  if (propagated_tf_ && imu_propagated_stamp_ - stamp2Sec(imu_prop_odom.header.stamp) < .02) return;
  const Eigen::Quaterniond q(imu_propagate.rot_end);
  imu_prop_odom.header.frame_id = propagated_tf_ ? world_frame_ : "world";
  imu_prop_odom.child_frame_id = propagated_tf_ ? body_frame_ : "";
  imu_prop_odom.header.stamp = sec2Stamp(imu_propagated_stamp_);
  imu_prop_odom.pose.pose.position.x = imu_propagate.pos_end.x();
  imu_prop_odom.pose.pose.position.y = imu_propagate.pos_end.y();
  imu_prop_odom.pose.pose.position.z = imu_propagate.pos_end.z();
  imu_prop_odom.pose.pose.orientation.w = q.w();
  imu_prop_odom.pose.pose.orientation.x = q.x();
  imu_prop_odom.pose.pose.orientation.y = q.y();
  imu_prop_odom.pose.pose.orientation.z = q.z();
  const V3D velocity = propagated_tf_ ? V3D(imu_propagate.rot_end.transpose() * imu_propagate.vel_end) : imu_propagate.vel_end;
  const V3D angular = V3D(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z) - imu_propagate.bias_g;
  imu_prop_odom.twist.twist.linear.x = velocity.x();
  imu_prop_odom.twist.twist.linear.y = velocity.y();
  imu_prop_odom.twist.twist.linear.z = velocity.z();
  imu_prop_odom.twist.twist.angular.x = angular.x();
  imu_prop_odom.twist.twist.angular.y = angular.y();
  imu_prop_odom.twist.twist.angular.z = angular.z();
  if (!propagated_tf_) { pubImuPropOdom->publish(imu_prop_odom); return; }
  if (!imu_pose_output_) {
    // A separate worker still shares the large-cloud participant's DDS locks.
    // Keep measured navigation poses/TF on their own small-message participant.
    const auto source = node->get_node_base_interface()->get_context();
    auto options = source->get_init_options();
    size_t domain = 0;
    const auto result = rcl_context_get_domain_id(source->get_rcl_context().get(), &domain);
    if (result != RCL_RET_OK)
      rclcpp::exceptions::throw_from_rcl_error(result, "Read navigation pose ROS domain");
    options.set_domain_id(domain);
    options.auto_initialize_logging(false);
    options.shutdown_on_signal = false;
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, options);
    rclcpp::NodeOptions node_options;
    node_options.context(context).use_global_arguments(false).enable_rosout(false)
        .start_parameter_services(false).start_parameter_event_publisher(false).use_clock_thread(false);
    auto pose_node = std::make_shared<rclcpp::Node>("fast_livo_navigation_pose", node->get_namespace(), node_options);
    auto br = std::make_shared<tf2_ros::TransformBroadcaster>(pose_node);
    pubImuPropOdom = pose_node->create_publisher<nav_msgs::msg::Odometry>(
        SensorTransport::resolve(node, "/LIVO2/imu_propagate"), 50);
    imu_pose_output_ = std::make_unique<AsyncPoseOutput>(
        [pose_node, br, publisher=pubImuPropOdom](const nav_msgs::msg::Odometry &pose) {
          geometry_msgs::msg::TransformStamped transform;
          transform.header = pose.header; transform.child_frame_id = pose.child_frame_id;
          transform.transform.translation.x = pose.pose.pose.position.x;
          transform.transform.translation.y = pose.pose.pose.position.y;
          transform.transform.translation.z = pose.pose.pose.position.z;
          transform.transform.rotation = pose.pose.pose.orientation;
          br->sendTransform(transform); publisher->publish(pose);
        });
  }
  if (!imu_pose_output_->submit(imu_prop_odom)) suspendTracking("imu_pose_output_failure");
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  trans_cloud->clear();
  trans_cloud->resize(input_cloud->size());
  #ifdef MP_EN
    #pragma omp parallel for num_threads(fast_livo_parallel::threadCount()) schedule(static)
  #endif
  for (int i = 0; i < static_cast<int>(input_cloud->size()); i++)
  {
    const pcl::PointXYZINormal &p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType &pi = trans_cloud->points[i];
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::trimLidarBuffer()
{
  const auto dropped = fast_livo_memory::trimPaired(lid_raw_data_buffer, lid_header_time_buffer, fast_livo_memory::kLidar);
  diagnostic_lidar_drops_ += dropped;
  if (dropped) {
    // ONLY_LIO may have pinned the old front scan while waiting for IMU.
    lidar_pushed = false;
    RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 2000,
                         "Lidar backlog: dropped %zu scans", dropped);
  }
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  raw_lidar_stamp_ = stamp2Sec(msg->header.stamp);
  raw_lidar_received_ = this->node->get_clock()->now().seconds();
  if (tracking_suspended_) return;
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    raw_lidar_end_cache_.invalidate();
    carried_lidar_end_cache_.invalidate();
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  if (!ptr || ptr->empty()) { mtx_buffer.unlock(); return; }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  trimLidarBuffer();

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  raw_lidar_stamp_ = stamp2Sec(msg_in->header.stamp);
  raw_lidar_received_ = this->node->get_clock()->now().seconds();
  if (tracking_suspended_) return;
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  if (verboseFrameLogs()) RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    raw_lidar_end_cache_.invalidate();
    carried_lidar_end_cache_.invalidate();
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
  }
  if (verboseFrameLogs()) RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;
  trimLidarBuffer();

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::suspendTracking(const std::string &reason)
{
  if (tracking_suspended_) return;
  tracking_suspended_ = true;
  tracking_fault_ = reason;
  // Keep map/archive and service executor alive. Never invent motion across an
  // uncovered IMU interval. The existing save-and-restart action starts a new
  // segment; absolute continuity requires relocalization, not a filter reset.
  imu_buffer.clear(); lid_raw_data_buffer.clear(); lid_header_time_buffer.clear();
  img_buffer.clear(); img_time_buffer.clear(); pending_vio_image_.release();
  raw_lidar_end_cache_.invalidate(); carried_lidar_end_cache_.invalidate();
  prop_imu_buffer.clear();
  ekf_finish_once = state_update_flg = new_imu = false;
  RCLCPP_ERROR(node->get_logger(), "Tracking suspended (%s); map and save service retained", reason.c_str());
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  raw_imu_stamp_ = stamp2Sec(msg_in->header.stamp);
  raw_imu_received_ = this->node->get_clock()->now().seconds();
  if (tracking_suspended_) return;
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  RCLCPP_DEBUG(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  if (!imu_input_frame_.empty()) {
    if (msg->header.frame_id != imu_input_frame_) {
      RCLCPP_ERROR(node->get_logger(), "Unexpected IMU frame: %s, expected %s",
                   msg->header.frame_id.c_str(), imu_input_frame_.c_str());
      suspendTracking("imu_frame_mismatch");
      return;
    }
    msg->header.frame_id = body_frame_;
  }
  msg->linear_acceleration.x *= imu_acc_scale_;
  msg->linear_acceleration.y *= imu_acc_scale_;
  msg->linear_acceleration.z *= imu_acc_scale_;
  if (msg->linear_acceleration_covariance[0] >= 0.)
    for (auto &value : msg->linear_acceleration_covariance) value *= imu_acc_scale_ * imu_acc_scale_;
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 2000, "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 &&
      timestamp > last_timestamp_imu + fast_livo_schedule::kImuGapWarnS)
  {
    const double gap = timestamp - last_timestamp_imu;
    // Catch-up delivery must not compress physically separate gaps into one
    // wall-time window and spuriously trigger the repeated-gap policy.
    const double gap_source_time = timestamp;
    if (imu_gap_window_begin_ < 0. ||
        gap_source_time - imu_gap_window_begin_ > fast_livo_schedule::kImuGapWindowS)
    {
      imu_gap_window_begin_ = gap_source_time;
      imu_gap_window_count_ = 0;
    }
    ++imu_gap_window_count_;
    ++imu_gap_total_;
    if (fast_livo_schedule::imuGapIsFatal(gap, imu_gap_window_count_))
    {
      mtx_buffer.unlock();
      RCLCPP_ERROR(node->get_logger(),
          "IMU gap %.6f s from %.6f to %.6f; %zu in source-time window; retaining map and save service",
          gap, last_timestamp_imu, timestamp, imu_gap_window_count_);
      suspendTracking("imu_gap");
      return;
    }
    // A short hole cannot be integrated, but it must not throw away the whole
    // mapping session: the APP would only report "startup failed". Drop the
    // measurement queues that straddle the hole and let the next LIO cut start
    // at the newest sample. Larger/repeated gaps suspend tracking above while
    // preserving the measured map and SaveMap service for segment recovery.
    RCLCPP_ERROR(this->node->get_logger(),
                 "IMU timestamp gap %.3f s; dropping stale buffers and restarting the LIO interval (gap %zu in window)",
                 gap, imu_gap_window_count_);
    imu_buffer.clear();
    raw_lidar_end_cache_.invalidate();
    carried_lidar_end_cache_.invalidate();
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    img_buffer.clear();
    img_time_buffer.clear();
    pending_vio_image_.release();
    lidar_pushed = false;
    // Reset the cut bookkeeping too, not just the queues. A half-cleared
    // group keeps the scan index of a frame that no longer exists, so every
    // later cut produced an empty scan ("No point!!!") and the IMU backlog
    // eventually tripped its 4000-sample limit.
    LidarMeasures = LidarMeasureGroup();
    LidarMeasures.last_lio_update_time = timestamp;
    p_imu->ResumeAfterGap(msg);
    {
      std::lock_guard<std::mutex> lock(mtx_buffer_imu_prop);
      prop_imu_buffer.clear();
      ekf_finish_once = false;  // Resume propagation only after a fresh LIO update.
      state_update_flg = false;
      new_imu = false;
    }
    // Fall through so this newest sample stays in the buffer: an empty
    // imu_buffer would leave the next pending-VIO cut without an IMU span.
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  if (imu_buffer.size() > fast_livo_memory::kImu) {
    mtx_buffer.unlock();
    throw std::runtime_error("IMU backlog exceeded 4000 samples; restart localization");
  }
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    if (prop_imu_buffer.size() > fast_livo_memory::kImu) {
      mtx_buffer_imu_prop.unlock();
      throw std::runtime_error("IMU propagation backlog exceeded 4000 samples; restart localization");
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  raw_image_stamp_ = stamp2Sec(msg_in->header.stamp);
  raw_image_received_ = this->node->get_clock()->now().seconds();
  if (tracking_suspended_) return;
  if (!img_en) return;
  const auto &msg = msg_in;
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  if (verboseFrameLogs()) RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct + fast_livo_schedule::kTimeTolerance - last_timestamp_img <
      fast_livo_schedule::imageDecodeInterval(lio_min_interval_, effectiveVioInterval()))
  {
    ++cadence_image_drops_;
    // Intentional visual cadence selection is counted, not a warning per frame.
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  const auto dropped = fast_livo_memory::trimPaired(img_buffer, img_time_buffer, fast_livo_memory::kImages);
  diagnostic_image_drops_ += dropped;
  if (dropped)
    RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 2000,
                         "Image backlog: dropped %zu frames", dropped);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (pose_output_ && pose_output_->stats().failed && !tracking_suspended_)
    suspendTracking("pose_output_failure");
  if (tracking_suspended_) { wait_reason_ = "tracking_suspended"; return false; }
  const bool pending_vio = slam_mode_ == LIVO && meas.lio_vio_flg == LIO && !pending_vio_image_.empty();
  if (!pending_vio) {
    if (lid_raw_data_buffer.empty() && lidar_en &&
        !(slam_mode_ == LIVO && !meas.pcl_proc_next->empty())) { wait_reason_ = "lidar_missing"; return false; }
    if (img_buffer.empty() && img_en && slam_mode_ != LIVO) { wait_reason_ = "image_missing"; return false; }
    if (imu_buffer.empty() && imu_en) { wait_reason_ = "imu_missing"; return false; }
  }

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    while (!lid_raw_data_buffer.empty() && fast_livo_schedule::lidarSliceExpired(
               *lid_raw_data_buffer.front(), lid_header_time_buffer.front(),
               meas.last_lio_update_time)) {
      lid_raw_data_buffer.pop_front();
      lid_header_time_buffer.pop_front();
      lidar_pushed = false;
    }
    if (lid_raw_data_buffer.empty()) { wait_reason_ = "lidar_missing"; return false; }
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = fast_livo_schedule::lidarSliceEnd(*meas.lidar, meas.lidar_frame_beg_time);
      meas.pcl_proc_cur.reset(new PointCloudXYZI);
      PointCloudXYZI unused_future;
      fast_livo_schedule::appendLidarSlice(*meas.lidar, meas.lidar_frame_beg_time,
          meas.last_lio_update_time, meas.lidar_frame_end_time,
          *meas.pcl_proc_cur, unused_future, true);
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      wait_reason_ = "imu_behind";
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    // A geometry-only update has no pending image. Continue LIO instead of
    // waiting forever in the LIO -> VIO transition.
    EKF_STATE last_lio_vio_flg = pending_vio ? LIO : WAIT;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      // IMU gap recovery clears processed buffers, but older scans may still
      // arrive from DDS / SensorInbox afterwards. Remove only fully expired
      // scans here; appendLidarSlice preserves a crossing scan's valid tail.
      while (!lid_raw_data_buffer.empty() && fast_livo_schedule::lidarSliceExpired(
                 *lid_raw_data_buffer.front(), lid_header_time_buffer.front(),
                 meas.last_lio_update_time)) {
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }
      if (lid_raw_data_buffer.empty() && meas.pcl_proc_next->empty()) {
        wait_reason_ = "lidar_missing";
        return false;
      }
      double lid_newest_time = meas.last_lio_update_time;
      // Raw queued scans are immutable. A carried cloud changes only in the
      // successful cut below, where its memo is invalidated before mutation.
      // Repeated waits for delayed IMU/images therefore do not rescan every
      // LiDAR point, while keeping the exact maximum acquisition timestamp.
      if (!meas.pcl_proc_next->empty())
        lid_newest_time = std::max(lid_newest_time, carried_lidar_end_cache_.get(
            meas.pcl_proc_next, meas.last_lio_update_time));
      if (!lid_raw_data_buffer.empty())
        lid_newest_time = std::max(lid_newest_time, raw_lidar_end_cache_.get(
            lid_raw_data_buffer.back(), lid_header_time_buffer.back()));
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      // MID360 IMU samples are 5 ms apart. An image immediately after a
      // fallback cut would otherwise produce another LIO fit with as few as
      // one point and no new IMU sample. Skip that image, retaining its LiDAR
      // returns for the next sufficiently separated state.
      while (!img_time_buffer.empty() &&
             fast_livo_schedule::skipImage(img_time_buffer.front() + exposure_time_init,
                 meas.last_lio_update_time, last_vio_update_time_, lio_min_interval_, effectiveVioInterval()))
      {
        ++cadence_image_drops_;
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 5000,
                            "Discarding an image too close to or older than the current LiDAR state");
      }

      // Bound the motion-compensation interval independently of camera delivery.
      // Waiting for a delayed image used to combine seconds of scans into one
      // EKF update, then insert the distorted geometry into the map.
      const double next_lio_time = fast_livo_schedule::nextLioTime(
          meas.last_lio_update_time, last_timestamp_img + exposure_time_init,
          last_vio_update_time_, lio_min_interval_);
      const bool fuse_image = !img_time_buffer.empty() &&
          img_time_buffer.front() + exposure_time_init <= next_lio_time + fast_livo_schedule::kTimeTolerance;
      const double img_capture_time = fuse_image ?
          img_time_buffer.front() + exposure_time_init : next_lio_time;

      // Native LiDAR can precede network camera delivery. Keep a bounded
      // processing delay instead of shifting timestamps or losing every image.
      if (input_holdback_s_ > 0. &&
          this->node->get_clock()->now().seconds() - img_capture_time < input_holdback_s_)
      {
        wait_reason_ = "input_holdback";
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        wait_reason_ = img_capture_time > imu_newest_time ? "imu_behind" : "lidar_behind";
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }
      if (!fuse_image && fast_livo_schedule::waitForImage(
              meas.last_lio_update_time, next_lio_time, last_timestamp_img + exposure_time_init,
              imu_newest_time, lio_min_interval_, effectiveVioInterval())) {
        wait_reason_ = "image_arrival_grace";
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      carried_lidar_end_cache_.invalidate();
      PointCloudXYZI carried_points;
      carried_points.swap(*meas.pcl_proc_next);
      meas.pcl_proc_cur->clear();
      fast_livo_schedule::appendLidarSlice(carried_points, meas.last_lio_update_time,
          meas.last_lio_update_time, m.lio_time, *meas.pcl_proc_cur, *meas.pcl_proc_next);

      size_t max_size = meas.pcl_proc_cur->size();
      for (size_t i = 0; i < lid_header_time_buffer.size() && lid_header_time_buffer[i] <= img_capture_time; ++i)
        max_size += lid_raw_data_buffer[i]->size();
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        double frame_header_time(lid_header_time_buffer.front());
        fast_livo_schedule::appendLidarSlice(*lid_raw_data_buffer.front(), frame_header_time,
            meas.last_lio_update_time, m.lio_time, *meas.pcl_proc_cur, *meas.pcl_proc_next);
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      if (fuse_image) {
        last_vio_update_time_ = img_capture_time;
        pending_vio_image_ = img_buffer.front();
        pending_vio_time_ = img_capture_time;
        img_buffer.pop_front();
        img_time_buffer.pop_front();
      } else {
        pending_vio_image_.release();
        RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 5000,
                            "Camera update delayed; continuing LiDAR/IMU updates");
      }
      meas.measures.clear();
      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      if (pending_vio_image_.empty()) return false;
      meas.lio_vio_flg = VIO;
      meas.measures.clear();
      struct MeasureGroup m;
      m.vio_time = pending_vio_time_;
      m.lio_time = meas.last_lio_update_time;
      m.img = pending_vio_image_;
      pending_vio_image_.release();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed)
    {
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}


bool LIVMapper::asyncRgbEnabled() const {
  return rgb_preview_ && slam_mode_ == LIVO &&
      fast_livo_rgb::asynchronousPreviewAllowed(pcd_save_en, img_save_en,
          colmap_output_en, rgb_camera_snapshot_.has_value());
}

void LIVMapper::initializeAsyncRgbPreview() {
  // navigation.enabled supplies the session generation used to reject old jobs.
  // Unsupported camera/research-export modes keep their complete legacy path.
  if (slam_mode_ != LIVO || !node->get_parameter("navigation.enabled").as_bool()) return;
  rgb_camera_snapshot_ = fast_livo_rgb::PinholeParameters::snapshot(*vio_manager->cam);
  if (!fast_livo_rgb::asynchronousPreviewAllowed(pcd_save_en, img_save_en,
      colmap_output_en, rgb_camera_snapshot_.has_value())) return;
  auto logger = node->get_logger();
  auto clock = node->get_clock();
  rgb_preview_ = std::make_unique<fast_livo_rgb::AsyncRgbPreview>(
      [this, cloud_publisher = pubLaserCloudFullRes, image_publisher = pubImage,
       logger, clock](const fast_livo_rgb::PreviewResult &result) {
        if (result.generation != navigation_output_->generation()) return;
        try {
          if (result.cloud && !result.cloud->empty()) {
            if (cloud_publisher->get_subscription_count() +
                cloud_publisher->get_intra_process_subscription_count() > 0) {
              sensor_msgs::msg::PointCloud2 message;
              pcl::toROSMsg(*result.cloud, message);
              message.header.stamp = result.stamp;
              message.header.frame_id = result.world_frame;
              if (result.generation == navigation_output_->generation())
                cloud_publisher->publish(message);
            }
            navigation_output_->publishRgb(result.cloud, result.stamp,
                result.world_frame, result.generation);
          }
        } catch (const std::exception &error) {
          RCLCPP_WARN_THROTTLE(logger, *clock, 5000,
                              "RGB cloud display failed; retrying next frame: %s", error.what());
        }
        try {
          if (image_publisher.getNumSubscribers() > 0 && !result.annotated.empty() &&
              result.generation == navigation_output_->generation()) {
            cv_bridge::CvImage image;
            image.header.stamp = result.stamp;  // Capture/VIO time, never publication wall time.
            image.encoding = sensor_msgs::image_encodings::BGR8;
            image.image = result.annotated;
            image_publisher.publish(image.toImageMsg());
          }
        } catch (const std::exception &error) {
          RCLCPP_WARN_THROTTLE(logger, *clock, 5000,
                              "RGB image display failed; retrying next frame: %s", error.what());
        }
      }, [this] { return navigation_output_->generation(); },
      [logger, clock](std::exception_ptr failure) {
        try { std::rethrow_exception(failure); }
        catch (const std::exception &error) {
          RCLCPP_WARN_THROTTLE(logger, *clock, 5000,
                              "RGB preview frame failed; retrying next frame: %s", error.what());
        }
        catch (...) {
          RCLCPP_WARN_THROTTLE(logger, *clock, 5000,
                              "RGB preview frame failed with an unknown display error");
        }
      });
}

void LIVMapper::publishAsyncRgbPreview() {
  const double time = LidarMeasures.last_lio_update_time;
  const auto generation = navigation_output_->generation();
  rgb_geometry_window_.begin(time, generation, world_frame_);
  ++rgb_preview_counter_;
  if (LidarMeasures.lio_vio_flg != VIO || rgb_geometry_window_.empty())
    rgb_geometry_window_.append(pcl_w_wait_pub);
  if (LidarMeasures.lio_vio_flg != VIO) return;
  if (!vio_manager->new_frame_) return;
  auto snapshot = std::make_shared<fast_livo_rgb::PreviewInput>();
  snapshot->camera = *rgb_camera_snapshot_;
  snapshot->q_camera_world = vio_manager->new_frame_->T_f_w_.unit_quaternion();
  snapshot->t_camera_world = vio_manager->new_frame_->T_f_w_.translation();
  snapshot->rgb = vio_manager->img_rgb;
  snapshot->annotated = vio_manager->img_cp;
  snapshot->blind = blind_rgb_points;
  snapshot->stamp = sec2Stamp(LidarMeasures.measures.back().vio_time);
  snapshot->world_frame = world_frame_;
  snapshot->generation = generation;
  snapshot->colorize_cloud = rgb_preview_counter_ >= pub_scan_num;
  if (snapshot->colorize_cloud) {
    snapshot->geometry = rgb_geometry_window_.take();
    rgb_preview_counter_ = 1;
  }
  try {
    rgb_preview_->submit(std::move(snapshot));
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "RGB preview unavailable; tracking continues: %s", error.what());
  } catch (...) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "RGB preview unavailable; tracking continues");
  }
  // Release our handle only; other immutable consumers can still own this scan.
  pcl_w_wait_pub.reset(new PointCloudXYZI());
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = sec2Stamp(LidarMeasures.measures.back().vio_time);
  // out_msg.header.frame_id = world_frame_;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  if (asyncRgbEnabled()) { publishAsyncRgbPreview(); return; }
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  // LIO continues while visual updates are throttled. Preserve all geometry
  // within the recent colouring window instead of only the last LIO slice.
  const double rgb_time = LidarMeasures.last_lio_update_time;
  if (rgb_pending_start_ < 0 || rgb_time < rgb_pending_start_ || rgb_time-rgb_pending_start_ > .5) {
    rgb_pending_.clear(); rgb_pending_start_ = rgb_time;
  }
  if (slam_mode_ == LIVO && (LidarMeasures.lio_vio_flg != VIO || rgb_pending_.points().empty())) {
    for (const auto &p : *pcl_w_wait_pub) rgb_pending_.add({p.x,p.y,p.z});
  }

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    pcl_wait_pub->clear();
    for (const auto &entry : rgb_pending_.points()) {
      const auto &p=entry.second;
      PointType point;point.x=p.x;point.y=p.y;point.z=p.z;
      pcl_wait_pub->push_back(point);
    }
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
      // This publication window is consumed even when no point projects into
      // the camera. Otherwise looking away from geometry retains every scan.
      PointCloudXYZI().swap(*pcl_wait_pub);
      rgb_pending_.clear(); rgb_pending_start_ = -1.;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  {
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
  }
  laserCloudmsg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  laserCloudmsg.header.frame_id = world_frame_;
  // A LIVO-only LIO step accumulates geometry for the next coloured frame;
  // it has no RGB publication. Do not interleave empty frames that clear the
  // viewer, and still run the independent recording logic below.
  fast_livo_output::publishFrameCloudIfPresent(pubLaserCloudFullRes, laserCloudmsg);
  if (!laserCloudWorldRGB->empty())
    navigation_output_->publishRgb(
        laserCloudWorldRGB, laserCloudmsg.header.stamp, world_frame_);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {


    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          fast_livo_memory::appendCloud(*pcl_wait_save, *laserCloudWorldRGB, pcd_max_points_,
              [this](const auto &cloud) { writePCDChunk(cloud); });
        }
        else
        {
          fast_livo_memory::appendCloud(*pcl_wait_save_intensity, *pcl_w_wait_pub, pcd_max_points_,
              [this](const auto &cloud) { writePCDChunk(cloud); });
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) pcd_scan_count_++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          fast_livo_memory::appendCloud(*pcl_wait_save_intensity, *laserCloudBody, pcd_max_points_,
              [this](const auto &cloud) { writePCDChunk(cloud); });
          pcd_scan_count_++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;

        break;

      default:
        pcd_save_interval = 1;
        pcd_scan_count_++;
        break;
    }
    if (pcd_save_interval > 0 && pcd_scan_count_ >= pcd_save_interval) savePCD();

    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(DEBUG_FILE_DIR("image/" + ss_time.str() + ".png"), vio_manager->img_rgb);

      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if (LidarMeasures.lio_vio_flg == VIO) pcl_w_wait_pub.reset(new PointCloudXYZI());
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = this->node->get_clock()->now();
    laserCloudmsg.header.frame_id = world_frame_;
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = world_frame_;
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  if (tracking_suspended_) return;
  odomAftMapped.header.frame_id = world_frame_;
  odomAftMapped.child_frame_id = body_frame_;
  odomAftMapped.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  set_posestamp(odomAftMapped.pose.pose);

  if (!pose_output_) {
    auto br = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    pose_output_ = std::make_unique<AsyncPoseOutput>(
      [br, broadcast_tf=!propagated_tf_, publisher=pubOdomAftMapped, mavros=mavros_pose_publisher](const nav_msgs::msg::Odometry &pose) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = pose.header; transform.child_frame_id = pose.child_frame_id;
        transform.transform.translation.x = pose.pose.pose.position.x;
        transform.transform.translation.y = pose.pose.pose.position.y;
        transform.transform.translation.z = pose.pose.pose.position.z;
        transform.transform.rotation = pose.pose.pose.orientation;
        if (broadcast_tf) br->sendTransform(transform);
        publisher->publish(pose);
        if (mavros) {
          geometry_msgs::msg::PoseStamped message;
          message.header = pose.header; message.pose = pose.pose.pose;
          mavros->publish(message);
        }
      });
  }
  if (!pose_output_->submit(odomAftMapped)) suspendTracking("pose_output_failure");
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = world_frame_;
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = world_frame_;
  path_preview_->append(msg_body_pose);
}

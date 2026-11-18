#include <atomic>
#include <future>
/**
 * @file localizer_node.cpp
 * @brief 基于ICP的定位节点实现（支持 initialpose map->base_link -> map->odom 计算）
 */

#include <queue>
#include <deque>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <fstream>
#include <openssl/evp.h>

// ROS2核心头文件
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL和TF相关头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 本地头文件
#include "localizers/commons.h"
#include "tracking_policy.h"
#include <optional>
#include "initialization_window.h"
#include "localizers/icp_localizer.h"
#include "localizers/pose_graph_manager.h" // 引入PoseGraph
#include <yaml-cpp/yaml.h>
#include "input_policy.h"

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fast_livo/cloud_body";
    std::string odom_topic = "/aft_mapped_to_init";
    std::string map_frame = "map";
    std::string local_frame = "base_link";
    double update_hz = 1.0;
    std::string default_map_path = "";
    bool save_clouds = false;  // ������ 新增：是否保存定位时的点云
    std::string save_clouds_dir = "/tmp/localizer_clouds/";  // 保存目录
};

// 在NodeState结构体中添加匹配控制标志
struct NodeState
{
    std::mutex message_mutex;
    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    bool map_loaded = false;
    bool need_icp_match = true; // 添加标志：是否需要进行ICP匹配
    bool skip_global_registration = false; // 添加标志：是否跳过全局配准（TEASER++）
    bool force_sc_relocalization = false; // 标志：是否强制进行SC全局重定位
    int icp_failure_count = 0;  // ICP失败计数器
    const int max_failure_attempts = 3;  // 最大失败尝试次数
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;

    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r = M3D::Identity();  // ������ 修复：初始化为单位矩阵
    V3D last_t = V3D::Zero();      // ������ 修复：初始化为零向量

    // ������ 定位过程中的 odom 位移补偿
    M3D odom_start_r = M3D::Identity();  // 定位开始时的 odom 旋转
    V3D odom_start_t = V3D::Zero();      // 定位开始时的 odom 位置
    bool odom_start_recorded = false;   // 是否已记录定位开始时的 odom

    // 原 map->odom
    M3D last_offset_r = M3D::Identity();
    V3D last_offset_t = V3D::Zero();

    // 初始位姿 map->base_link
    M3D map_base_r = M3D::Identity();
    V3D map_base_t = V3D::Zero();

    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode()
        : Node("localizer_node"),
          max_time_diff_s(0.02)
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started (initialpose topic, max_time_diff=%.3f s)", max_time_diff_s);
        loadParameters();
        rcl_interfaces::msg::ParameterDescriptor tracking_startup;
        tracking_startup.read_only = true;
        match_timeout_ = declare_parameter<double>("tracking_timeout_s", 5.0, tracking_startup);
        max_correction_ = declare_parameter<double>("tracking_max_correction_m", 0.5, tracking_startup);
        max_rotation_ = declare_parameter<double>("tracking_max_correction_deg", 15.0, tracking_startup);
        tracking_min_scan_points_ = declare_parameter<int>("tracking_min_scan_points", 0, tracking_startup);
        confirm_translation_ = declare_parameter<double>("tracking_confirm_translation_m", 0., tracking_startup);
        rcl_interfaces::msg::ParameterDescriptor relocalization_runtime;
        relocalization_runtime.description =
            "Maximum age/compute window for one manual or global relocalization; runtime configurable for field tests.";
        relocalization_timeout_s_ = declare_parameter<double>(
            "relocalization_timeout_s", relocalization_timeout_s_,
            relocalization_runtime);
        if (!std::isfinite(match_timeout_) || !std::isfinite(max_correction_) || !std::isfinite(max_rotation_) ||
            !std::isfinite(m_config.update_hz) || m_config.update_hz <= 0 ||
            !std::isfinite(confirm_translation_) || confirm_translation_ < 0. ||
            tracking_min_scan_points_ < 0 || match_timeout_ <= 0 || max_correction_ <= 0 || max_rotation_ <= 0 ||
            !std::isfinite(relocalization_timeout_s_) ||
            relocalization_timeout_s_ < 30. || relocalization_timeout_s_ > 600.)
            throw std::invalid_argument("tracking thresholds must be positive");
        quality_pub_ = create_publisher<std_msgs::msg::String>("~/quality", 10);
        instance_id_ = std::to_string(now().nanoseconds()) + "-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());


        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_config.cloud_topic, qos,
            std::bind(&LocalizerNode::cloudCB, this, std::placeholders::_1));

        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            m_config.odom_topic, qos,
            std::bind(&LocalizerNode::odomCB, this, std::placeholders::_1));

        m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1));
        m_global_relocalize_service = this->create_service<std_srvs::srv::Trigger>(
            "~/global_relocalize",
            std::bind(
                &LocalizerNode::globalRelocalizeCB, this,
                std::placeholders::_1, std::placeholders::_2));
        m_cancel_relocalize_service = this->create_service<std_srvs::srv::Trigger>(
            "~/cancel_relocalization",
            std::bind(
                &LocalizerNode::cancelRelocalizeCB, this,
                std::placeholders::_1, std::placeholders::_2));

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        m_parameter_callback_handle = this->add_on_set_parameters_callback(
            std::bind(
                &LocalizerNode::onParametersSet, this,
                std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "������ 地图配置信息:");
        RCLCPP_INFO(this->get_logger(), "  配置文件路径: %s", m_config.default_map_path.c_str());

        if (!m_config.default_map_path.empty() && std::filesystem::exists(m_config.default_map_path))
        {
            RCLCPP_INFO(this->get_logger(), "  地图文件状态: ✅ 存在");
            const auto before_load_digest = mapDigest(m_config.default_map_path);
            if (m_localizer->loadMap(m_config.default_map_path))
            {
                RCLCPP_INFO(this->get_logger(), "  加载结果: ✅ 成功");
                RCLCPP_INFO(this->get_logger(), "������️  当前使用地图: %s", m_config.default_map_path.c_str());
                m_state.map_loaded = true;
                map_sha256_ = mapDigest(m_config.default_map_path);
                if (map_sha256_ != before_load_digest)
                    throw std::runtime_error("map changed while loading; retry with a stable file");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  加载结果: ❌ 失败");
                RCLCPP_ERROR(this->get_logger(), "❌ 地图加载失败: %s", m_config.default_map_path.c_str());
                throw std::runtime_error(
                    "configured localization map or Scan Context cache failed to load");
            }
        }
        else
        {
            if (m_config.default_map_path.empty())
            {
                RCLCPP_WARN(this->get_logger(), "  地图文件状态: ⚠️  配置为空");
                RCLCPP_WARN(this->get_logger(), "⚠️  未配置地图路径！");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  地图文件状态: ❌ 不存在");
                RCLCPP_ERROR(this->get_logger(), "❌ 地图文件不存在: %s", m_config.default_map_path.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // 添加匹配过程可视化的发布器（添加localizer命名空间）
        m_teaser_source_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/teaser_source_cloud", 10);
        m_teaser_target_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/teaser_target_cloud", 10);
        m_teaser_corr_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("localizer/teaser_correspondences", 10);
        m_rough_source_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/rough_source_cloud", 10);
        m_rough_target_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/rough_target_cloud", 10);
        m_refine_source_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/refine_source_cloud", 10);
        m_refine_target_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("localizer/refine_target_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "发布话题:");
        RCLCPP_INFO(this->get_logger(), "  - /map_cloud: 地图点云");
        RCLCPP_INFO(this->get_logger(), "  - /teaser_aligned_cloud: TEASER++全局配准结果");
        RCLCPP_INFO(this->get_logger(), "  - /rough_aligned_cloud: GICP粗匹配结果");
        RCLCPP_INFO(this->get_logger(), "  - /refine_aligned_cloud: GICP精匹配结果");

        auto period = std::chrono::milliseconds(10);
        m_timer = this->create_wall_timer(period, std::bind(&LocalizerNode::timerCB, this));
    }

    ~LocalizerNode() override {
        // The worker captures this, including the diagnostics mutex. A future
        // declared first is destroyed last: its implicit join would be too late.
        ++match_generation_;
        if (match_future_.valid()) {
            try { match_future_.get(); } catch (...) { /* destructor must not throw */ }
        }
        if (m_localizer) {
            m_localizer->setDiagnosticsCallback({});
            m_localizer->setCancellationCheck({});
        }
    }

    void declareScanContextParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor restart_required;
        restart_required.read_only = true;
        restart_required.description =
            "Applied while loading the map/SC database; change at startup and restart the node.";

        rcl_interfaces::msg::ParameterDescriptor runtime_configurable;
        runtime_configurable.description =
            "Can be changed at runtime and is applied to the next global relocalization.";

        const auto positiveOverride = [this, &restart_required](
            const char *name, double current) {
            const double value = this->declare_parameter<double>(
                std::string(name) + "_override", -1., restart_required);
            if (value == -1.) return current;
            if (!std::isfinite(value) || value <= 0.)
                throw std::invalid_argument(
                    std::string(name) +
                    " override must be finite and positive");
            return value;
        };
        const auto iterationOverride = [this, &restart_required](
            const char *name, int current) {
            const int value = this->declare_parameter<int>(
                std::string(name) + "_override", -1, restart_required);
            if (value == -1) return current;
            if (value < 1)
                throw std::invalid_argument(
                    std::string(name) +
                    " override must be a positive integer");
            return value;
        };
        m_localizer_config.rough_score_thresh = positiveOverride(
            "rough_score_thresh", m_localizer_config.rough_score_thresh);
        m_localizer_config.refine_score_thresh = positiveOverride(
            "refine_score_thresh", m_localizer_config.refine_score_thresh);
        m_localizer_config.rough_scan_resolution = positiveOverride(
            "rough_scan_resolution",
            m_localizer_config.rough_scan_resolution);
        m_localizer_config.refine_scan_resolution = positiveOverride(
            "refine_scan_resolution",
            m_localizer_config.refine_scan_resolution);
        m_localizer_config.rough_map_resolution = positiveOverride(
            "rough_map_resolution", m_localizer_config.rough_map_resolution);
        m_localizer_config.refine_map_resolution = positiveOverride(
            "refine_map_resolution", m_localizer_config.refine_map_resolution);
        m_localizer_config.rough_max_iteration = iterationOverride(
            "rough_max_iteration", m_localizer_config.rough_max_iteration);
        m_localizer_config.refine_max_iteration = iterationOverride(
            "refine_max_iteration", m_localizer_config.refine_max_iteration);

        m_localizer_config.use_scan_context = this->declare_parameter<bool>(
            "use_scan_context", m_localizer_config.use_scan_context,
            restart_required);
        m_localizer_config.scan_context_path = this->declare_parameter<std::string>(
            "scan_context_path", m_localizer_config.scan_context_path,
            restart_required);
        const auto scan_context_override = this->declare_parameter<std::string>(
            "scan_context_path_override", "", restart_required);
        if (!scan_context_override.empty())
            m_localizer_config.scan_context_path = scan_context_override;
        m_localizer_config.sc_max_radius = this->declare_parameter<double>(
            "sc_max_radius", m_localizer_config.sc_max_radius,
            restart_required);
        m_localizer_config.sc_grid_resolution = this->declare_parameter<double>(
            "sc_grid_resolution", m_localizer_config.sc_grid_resolution,
            restart_required);
        const auto gravity_override = this->declare_parameter<std::string>(
            "sc_use_odom_gravity_override", "", restart_required);
        if (!gravity_override.empty()) {
            if (gravity_override != "true" && gravity_override != "false")
                throw std::invalid_argument("sc_use_odom_gravity_override must be true/false");
            m_localizer_config.sc_use_odom_gravity = gravity_override == "true";
        }
        m_localizer_config.sc_use_odom_gravity = this->declare_parameter<bool>(
            "sc_use_odom_gravity", m_localizer_config.sc_use_odom_gravity,
            restart_required);
        m_localizer_config.use_correlative_registration = this->declare_parameter<bool>(
            "use_correlative_registration", m_localizer_config.use_correlative_registration,
            restart_required);
        m_localizer_config.teaser_max_clique_time_s =
            this->declare_parameter<double>(
                "teaser_max_clique_time_s",
                m_localizer_config.teaser_max_clique_time_s,
                restart_required);
        m_localizer_config.teaser_solver_timeout_s =
            this->declare_parameter<double>(
                "teaser_solver_timeout_s",
                m_localizer_config.teaser_solver_timeout_s,
                restart_required);
        if (!std::isfinite(m_localizer_config.teaser_solver_timeout_s) ||
            m_localizer_config.teaser_solver_timeout_s < .05 ||
            m_localizer_config.teaser_solver_timeout_s > 30.)
            throw std::invalid_argument(
                "teaser_solver_timeout_s must be in [0.05,30] seconds");
        m_localizer_config.teaser_max_clique_threads =
            this->declare_parameter<int>(
                "teaser_max_clique_threads",
                m_localizer_config.teaser_max_clique_threads,
                restart_required);
        if (!std::isfinite(m_localizer_config.teaser_max_clique_time_s) ||
            m_localizer_config.teaser_max_clique_time_s < .01 ||
            m_localizer_config.teaser_max_clique_time_s > 30. ||
            m_localizer_config.teaser_max_clique_threads < 1 ||
            m_localizer_config.teaser_max_clique_threads > 2)
            throw std::invalid_argument(
                "TEASER clique time must be in [0.01,30] seconds; "
                "threads in [1,2]");
        m_localizer_config.global_coarse_first = this->declare_parameter<bool>(
            "global_coarse_first", m_localizer_config.global_coarse_first,
            restart_required);
        m_localizer_config.global_coarse_voxel_size = this->declare_parameter<double>(
            "global_coarse_voxel_size", m_localizer_config.global_coarse_voxel_size,
            restart_required);
        if (!std::isfinite(m_localizer_config.global_coarse_voxel_size) ||
            m_localizer_config.global_coarse_voxel_size <= 0.)
            throw std::invalid_argument("global_coarse_voxel_size must be finite and positive");
        m_localizer_config.global_support_distance = this->declare_parameter<double>(
            "global_support_distance", m_localizer_config.global_support_distance, restart_required);
        m_localizer_config.global_support_min_ratio = this->declare_parameter<double>(
            "global_support_min_ratio", m_localizer_config.global_support_min_ratio, restart_required);
        if (!std::isfinite(m_localizer_config.global_support_distance) || m_localizer_config.global_support_distance <= 0. ||
            !std::isfinite(m_localizer_config.global_support_min_ratio) || m_localizer_config.global_support_min_ratio <= 0. ||
            m_localizer_config.global_support_min_ratio > 1.)
            throw std::invalid_argument("global support distance must be positive, ratio in (0,1]");

        m_localizer_config.sc_dist_thresh = this->declare_parameter<double>(
            "sc_dist_thresh", m_localizer_config.sc_dist_thresh,
            runtime_configurable);
        m_localizer_config.sc_similar_candidates_thresh =
            this->declare_parameter<double>(
                "sc_similar_candidates_thresh",
                m_localizer_config.sc_similar_candidates_thresh,
                runtime_configurable);
        m_localizer_config.sc_max_candidates = this->declare_parameter<int>(
            "sc_max_candidates", m_localizer_config.sc_max_candidates,
            runtime_configurable);
        m_localizer_config.sc_enable_early_exit = this->declare_parameter<bool>(
            "sc_enable_early_exit", m_localizer_config.sc_enable_early_exit,
            runtime_configurable);

        if (m_localizer_config.sc_max_radius <= 0.0) {
            throw std::invalid_argument("sc_max_radius must be greater than 0");
        }
        if (m_localizer_config.sc_grid_resolution <= 0.0) {
            throw std::invalid_argument(
                "sc_grid_resolution must be greater than 0");
        }
        if (m_localizer_config.sc_dist_thresh <= 0.0) {
            throw std::invalid_argument("sc_dist_thresh must be greater than 0");
        }
        if (m_localizer_config.sc_similar_candidates_thresh < 0.0) {
            throw std::invalid_argument(
                "sc_similar_candidates_thresh must be non-negative");
        }
        if (m_localizer_config.sc_max_candidates < 1) {
            throw std::invalid_argument(
                "sc_max_candidates must be at least 1");
        }
    }

    rcl_interfaces::msg::SetParametersResult onParametersSet(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        ICPConfig updated_config = m_localizer_config;
        bool sc_config_changed = false;
        double updated_relocalization_timeout = relocalization_timeout_s_;
        bool relocalization_timeout_changed = false;

        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name == "sc_dist_thresh") {
                const double value = parameter.as_double();
                if (value <= 0.0) {
                    result.successful = false;
                    result.reason = "sc_dist_thresh must be greater than 0";
                    return result;
                }
                updated_config.sc_dist_thresh = value;
                sc_config_changed = true;
            } else if (name == "sc_similar_candidates_thresh") {
                const double value = parameter.as_double();
                if (value < 0.0) {
                    result.successful = false;
                    result.reason =
                        "sc_similar_candidates_thresh must be non-negative";
                    return result;
                }
                updated_config.sc_similar_candidates_thresh = value;
                sc_config_changed = true;
            } else if (name == "sc_max_candidates") {
                const int value = parameter.as_int();
                if (value < 1) {
                    result.successful = false;
                    result.reason = "sc_max_candidates must be at least 1";
                    return result;
                }
                updated_config.sc_max_candidates = value;
                sc_config_changed = true;
            } else if (name == "sc_enable_early_exit") {
                updated_config.sc_enable_early_exit = parameter.as_bool();
                sc_config_changed = true;
            } else if (name == "relocalization_timeout_s") {
                const double value = parameter.as_double();
                if (!std::isfinite(value) || value < 30. || value > 600.) {
                    result.successful = false;
                    result.reason =
                        "relocalization_timeout_s must be within [30, 600] seconds";
                    return result;
                }
                updated_relocalization_timeout = value;
                relocalization_timeout_changed = true;
            }
        }

        if (sc_config_changed) {
            m_localizer_config = updated_config;
            RCLCPP_INFO(
                this->get_logger(),
                "Scan Context ROS parameters updated; changes apply to the next global relocalization");
        }

        if (relocalization_timeout_changed) {
            relocalization_timeout_s_ = updated_relocalization_timeout;
            last_quality_ = 0.;
            RCLCPP_INFO(
                this->get_logger(),
                "Relocalization timeout updated to %.0f seconds; applies to the next match",
                relocalization_timeout_s_);
        }

        return result;
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        this->declare_parameter("default_map_path", "");
        this->declare_parameter("cloud_topic", "");
        this->declare_parameter("odom_topic", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        // Empty config still honors launch overrides (map/cloud/odom).
        YAML::Node config = config_path.empty() ? YAML::Node(YAML::NodeType::Map) : YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            declareScanContextParameters();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        if (config["cloud_topic"]) m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        if (config["odom_topic"]) m_config.odom_topic = config["odom_topic"].as<std::string>();
        if (config["map_frame"]) m_config.map_frame = config["map_frame"].as<std::string>();
        if (config["local_frame"]) m_config.local_frame = config["local_frame"].as<std::string>();
        if (config["update_hz"]) m_config.update_hz = config["update_hz"].as<double>();
        if (config["default_map_path"]) m_config.default_map_path = config["default_map_path"].as<std::string>();
        if (config["relocalization_timeout_s"])
            relocalization_timeout_s_ = config["relocalization_timeout_s"].as<double>();

        // ICP参数
        if (config["rough_scan_resolution"]) m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        if (config["rough_map_resolution"]) m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        if (config["rough_max_iteration"]) m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        if (config["rough_score_thresh"]) m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();
        if (config["refine_scan_resolution"]) m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        if (config["refine_map_resolution"]) m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        if (config["refine_max_iteration"]) m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        if (config["refine_score_thresh"]) m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        // GICP特有参数
        if (config["correspondence_randomness"]) m_localizer_config.correspondence_randomness = config["correspondence_randomness"].as<int>();
        if (config["maximum_optimizer_iterations"]) m_localizer_config.maximum_optimizer_iterations = config["maximum_optimizer_iterations"].as<int>();

        // 全局配准参数
        if (config["use_global_registration"]) m_localizer_config.use_global_registration = config["use_global_registration"].as<bool>();
        if (config["global_voxel_size"]) m_localizer_config.global_voxel_size = config["global_voxel_size"].as<double>();
        if (config["global_feature_radius"]) m_localizer_config.global_feature_radius = config["global_feature_radius"].as<double>();
        if (config["global_score_thresh"]) m_localizer_config.global_score_thresh = config["global_score_thresh"].as<double>();

        // Scan Context 参数
        if (config["use_scan_context"]) m_localizer_config.use_scan_context = config["use_scan_context"].as<bool>();
        if (config["scan_context_path"]) m_localizer_config.scan_context_path = config["scan_context_path"].as<std::string>();
        if (config["sc_dist_thresh"]) m_localizer_config.sc_dist_thresh = config["sc_dist_thresh"].as<double>();
        if (config["sc_max_radius"]) m_localizer_config.sc_max_radius = config["sc_max_radius"].as<double>();
        if (config["sc_grid_resolution"]) m_localizer_config.sc_grid_resolution = config["sc_grid_resolution"].as<double>();
        if (config["sc_similar_candidates_thresh"]) m_localizer_config.sc_similar_candidates_thresh = config["sc_similar_candidates_thresh"].as<double>();
        if (config["sc_max_candidates"]) m_localizer_config.sc_max_candidates = config["sc_max_candidates"].as<int>();
        if (config["sc_enable_early_exit"]) m_localizer_config.sc_enable_early_exit = config["sc_enable_early_exit"].as<bool>();
        if (config["sc_use_odom_gravity"]) m_localizer_config.sc_use_odom_gravity = config["sc_use_odom_gravity"].as<bool>();
        if (config["use_correlative_registration"]) m_localizer_config.use_correlative_registration = config["use_correlative_registration"].as<bool>();
        if (config["teaser_max_clique_time_s"])
            m_localizer_config.teaser_max_clique_time_s =
                config["teaser_max_clique_time_s"].as<double>();
        if (config["teaser_solver_timeout_s"])
            m_localizer_config.teaser_solver_timeout_s =
                config["teaser_solver_timeout_s"].as<double>();
        if (config["teaser_max_clique_threads"])
            m_localizer_config.teaser_max_clique_threads =
                config["teaser_max_clique_threads"].as<int>();
        if (config["global_coarse_first"]) m_localizer_config.global_coarse_first = config["global_coarse_first"].as<bool>();
        if (config["global_coarse_voxel_size"]) m_localizer_config.global_coarse_voxel_size = config["global_coarse_voxel_size"].as<double>();
        if (config["global_support_distance"]) m_localizer_config.global_support_distance = config["global_support_distance"].as<double>();
        if (config["global_support_min_ratio"]) m_localizer_config.global_support_min_ratio = config["global_support_min_ratio"].as<double>();
        // YAML supplies defaults. Explicit ROS parameter overrides take precedence.
        declareScanContextParameters();
        RCLCPP_INFO(
            this->get_logger(),
            "ICP effective config: rough(score=%.3f scan=%.3f map=%.3f iter=%d) "
            "refine(score=%.3f scan=%.3f map=%.3f iter=%d)",
            m_localizer_config.rough_score_thresh,
            m_localizer_config.rough_scan_resolution,
            m_localizer_config.rough_map_resolution,
            m_localizer_config.rough_max_iteration,
            m_localizer_config.refine_score_thresh,
            m_localizer_config.refine_scan_resolution,
            m_localizer_config.refine_map_resolution,
            m_localizer_config.refine_max_iteration);
        if (config["enable_ground_z_correction"]) m_localizer_config.enable_ground_z_correction = config["enable_ground_z_correction"].as<bool>();
        if (config["map_ground_z"]) m_localizer_config.map_ground_z = config["map_ground_z"].as<double>();
        if (config["max_ground_z_correction"]) m_localizer_config.max_ground_z_correction = config["max_ground_z_correction"].as<double>();

        // ������ 新增：点云保存参数
        if (config["save_clouds"]) m_config.save_clouds = config["save_clouds"].as<bool>();
        if (config["save_clouds_dir"]) m_config.save_clouds_dir = config["save_clouds_dir"].as<std::string>();

        // 创建保存目录
        if (m_config.save_clouds) {
            std::filesystem::create_directories(m_config.save_clouds_dir);
            RCLCPP_INFO(this->get_logger(), "������ 点云保存已启用，保存目录: %s", m_config.save_clouds_dir.c_str());
        }

        // 输出全局配准状态
        RCLCPP_INFO(this->get_logger(), "全局配准: %s", m_localizer_config.use_global_registration ? "启用" : "禁用");
        if (m_localizer_config.use_global_registration) {
            RCLCPP_INFO(this->get_logger(), "  - 体素大小: %.2f m", m_localizer_config.global_voxel_size);
            RCLCPP_INFO(this->get_logger(), "  - 特征半径: %.2f m", m_localizer_config.global_feature_radius);
            RCLCPP_INFO(this->get_logger(), "  - 评分阈值: %.2f", m_localizer_config.global_score_thresh);
        }

        // 输出 Scan Context 状态
        RCLCPP_INFO(this->get_logger(), "Scan Context: %s", m_localizer_config.use_scan_context ? "启用" : "禁用");
        if (m_localizer_config.use_scan_context) {
            RCLCPP_INFO(this->get_logger(), "  - 缓存路径: %s",
                m_localizer_config.scan_context_path.empty()
                    ? "<定位PCD>.sc.v2"
                    : m_localizer_config.scan_context_path.c_str());
            RCLCPP_INFO(this->get_logger(), "  - 匹配阈值: %.2f", m_localizer_config.sc_dist_thresh);
            RCLCPP_INFO(this->get_logger(), "  - 最大半径: %.2f m", m_localizer_config.sc_max_radius);
            RCLCPP_INFO(this->get_logger(), "  - 网格分辨率: %.2f m", m_localizer_config.sc_grid_resolution);
            RCLCPP_INFO(this->get_logger(), "  - 最大候选数: %d", m_localizer_config.sc_max_candidates);
            RCLCPP_INFO(this->get_logger(), "  - 提前退出: %s", m_localizer_config.sc_enable_early_exit ? "启用" : "禁用");
        }

        std::string override_map_path;
        std::string override_cloud_topic;
        std::string override_odom_topic;
        this->get_parameter("default_map_path", override_map_path);
        this->get_parameter("cloud_topic", override_cloud_topic);
        this->get_parameter("odom_topic", override_odom_topic);
        if (!override_map_path.empty()) m_config.default_map_path = override_map_path;
        if (!override_cloud_topic.empty()) m_config.cloud_topic = override_cloud_topic;
        if (!override_odom_topic.empty()) m_config.odom_topic = override_odom_topic;

        RCLCPP_INFO(this->get_logger(), "Localization map: %s", m_config.default_map_path.c_str());
    }

    double monotonicSeconds() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double dataAge() const {
        if (!m_state.message_received) return 1e9;
        return (now() - rclcpp::Time(m_state.last_message_time, get_clock()->get_clock_type())).seconds();
    }

    bool localizationValid() const {
        const double observation_age = (now().nanoseconds()-last_success_stamp_)/1e9;
        return tracking::valid(m_state.localize_success,
            monotonicSeconds() - last_success_, dataAge(), m_state.icp_failure_count,
            match_timeout_, request_pending_ || m_state.service_received) &&
            observation_age >= -.1 && observation_age <= std::min(5., match_timeout_);
    }

    struct DiagnosticState {
        std::string operation = "idle";
        MatchDiagnostics match;
        double operation_started = 0.;
        double stage_started = 0.;
    };

    void beginDiagnostics(const std::string &operation,
                          const std::string &stage)
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        diagnostics_ = DiagnosticState{};
        diagnostics_.operation = operation;
        diagnostics_.match.stage = stage;
        diagnostics_.operation_started = monotonicSeconds();
        diagnostics_.stage_started = diagnostics_.operation_started;
    }

    void updateDiagnostics(uint64_t generation, const std::string &operation,
                           const MatchDiagnostics &match)
    {
        if (generation != match_generation_.load()) return;
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        if (generation != match_generation_.load()) return;
        const double now_s = monotonicSeconds();
        if (diagnostics_.operation != operation) {
            diagnostics_.operation = operation;
            diagnostics_.operation_started = now_s;
        }
        if (diagnostics_.match.stage != match.stage)
            diagnostics_.stage_started = now_s;
        diagnostics_.match = match;
    }

    void finishDiagnostics(const std::string &reason_code = "")
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        if (diagnostics_.match.stage != "complete")
            diagnostics_.stage_started = monotonicSeconds();
        diagnostics_.match.stage = "complete";
        diagnostics_.match.reason_code = reason_code;
        ++diagnostics_.match.progress_sequence;
    }

    void publishQuality() {
        const double t = monotonicSeconds();
        if (t - last_quality_ < .2) return;
        last_quality_ = t;
        DiagnosticState diagnostic;
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            diagnostic = diagnostics_;
        }
        const auto number = [](double value) {
            return std::isfinite(value) ? value : -1.;
        };
        std_msgs::msg::String msg;
        std::ostringstream out;
        Eigen::Quaterniond offset(m_state.last_offset_r);
        const auto &translation = m_state.last_offset_t;
        out << std::setprecision(17);
        out << "{\"schema_version\":1,\"valid\":" << (localizationValid() ? "true" : "false")
            << ",\"instance_id\":" << std::quoted(instance_id_)
            << ",\"map_sha256\":" << std::quoted(map_sha256_)
            << ",\"map_frame\":" << std::quoted(m_config.map_frame)
            << ",\"odom_frame\":" << std::quoted(m_config.local_frame)
            << ",\"input_error\":" << std::quoted(input_error_)
            << ",\"initial_pose_required\":" << (!m_state.localize_success && initial_attempted_ && !candidate_pending_ ? "true" : "false")
            << ",\"verifying_initial_pose\":" << (candidate_pending_ ? "true" : "false")
            << ",\"match_in_progress\":" << (match_future_.valid() ? "true" : "false")
            << ",\"tracking_timeout_s\":" << std::min(5., match_timeout_)
            << ",\"relocalization_timeout_s\":" << relocalization_timeout_s_
            << ",\"operation\":" << std::quoted(diagnostic.operation)
            << ",\"stage\":" << std::quoted(diagnostic.match.stage)
            << ",\"reason_code\":" << std::quoted(diagnostic.match.reason_code)
            << ",\"operation_elapsed_s\":" <<
                (diagnostic.operation_started > 0. ? t-diagnostic.operation_started : -1.)
            << ",\"stage_elapsed_s\":" <<
                (diagnostic.stage_started > 0. ? t-diagnostic.stage_started : -1.)
            << ",\"progress_sequence\":" << diagnostic.match.progress_sequence
            << ",\"input_points\":" << diagnostic.match.input_points
            << ",\"rough_points\":" << diagnostic.match.rough_points
            << ",\"refine_points\":" << diagnostic.match.refine_points
            << ",\"candidate_count\":" << diagnostic.match.candidate_count
            << ",\"supported_candidates\":" << diagnostic.match.supported_candidates
            << ",\"map_support_ratio\":" << number(diagnostic.match.map_support_ratio)
            << ",\"rough_converged\":" << (diagnostic.match.rough_converged ? "true" : "false")
            << ",\"rough_fitness\":" << number(diagnostic.match.rough_fitness)
            << ",\"rough_threshold\":" << number(diagnostic.match.rough_threshold)
            << ",\"refine_converged\":" << (diagnostic.match.refine_converged ? "true" : "false")
            << ",\"refine_fitness\":" << number(diagnostic.match.refine_fitness)
            << ",\"refine_threshold\":" << number(diagnostic.match.refine_threshold)
            << ",\"inlier_ratio\":" << number(diagnostic.match.inlier_ratio)
            << ",\"rmse\":" << number(diagnostic.match.rmse)
            << ",\"map_from_odom\":{\"translation_m\":[" << translation.x() << "," << translation.y() << "," << translation.z()
            << "],\"rotation_xyzw\":[" << offset.x() << "," << offset.y() << "," << offset.z() << "," << offset.w() << "]}"
            << ",\"published_stamp_ns\":" << now().nanoseconds()
            << ",\"match_sequence\":" << match_sequence_
            << ",\"last_success_stamp_ns\":" << last_success_stamp_
            << ",\"match_age_s\":" << (last_success_ > 0 ? t-last_success_ : -1.)
            << ",\"data_age_s\":" << dataAge()
            << ",\"last_match_duration_s\":" << last_match_duration_s_
            << ",\"last_match_observation_age_s\":" <<
                last_match_observation_age_s_
            << ",\"last_match_reason\":" <<
                std::quoted(last_match_reason_)
            << ",\"fitness\":" << (std::isfinite(last_fitness_) ? last_fitness_ : -1.)
            << ",\"failures\":" << m_state.icp_failure_count
            << ",\"request_id\":" << request_id_
            << ",\"completed_request_id\":" << completed_request_id_
            << ",\"request_succeeded\":" << (request_succeeded_ ? "true" : "false")
            << ",\"manual_request_id\":" << manual_request_id_
            << ",\"manual_pose_stamp_ns\":" << manual_pose_stamp_ns_
            << ",\"completed_manual_request_id\":" << completed_manual_request_id_
            << ",\"completed_manual_pose_stamp_ns\":" << completed_manual_pose_stamp_ns_
            << ",\"manual_request_succeeded\":" << (manual_request_succeeded_ ? "true" : "false") << "}";
        msg.data = out.str();
        quality_pub_->publish(msg);
    }

    struct MatchWork {
        M4F prior = M4F::Identity(), guess = M4F::Identity();
        M3D odom_rotation = M3D::Identity();
        V3D odom_translation = V3D::Zero();
        CloudType::Ptr cloud;
        ICPConfig config;
        int64_t stamp = 0;
        uint64_t generation = 0;
        uint64_t manual_request_id = 0;
        int64_t manual_pose_stamp_ns = 0;
        std::string operation = "tracking";
        MatchDiagnostics diagnostics;
        bool global = false, manual = false, confirming = false, result = false;
        double fitness = -1., result_timeout_s = 120., elapsed_s = 0.;
    };

    void completeManualRequest(uint64_t request_id, int64_t pose_stamp_ns, bool succeeded) {
        // The generation check in finishMatch protects this path from an old
        // worker.  The identity check is kept as a second guard because a new
        // /initialpose may reuse the same ROS header stamp.
        if (!manual_request_pending_ || request_id != manual_request_id_ ||
            pose_stamp_ns != manual_pose_stamp_ns_) return;
        completed_manual_request_id_ = request_id;
        completed_manual_pose_stamp_ns_ = pose_stamp_ns;
        manual_request_succeeded_ = succeeded;
        manual_request_pending_ = false;
    }

    void cancelPendingManualRequest() {
        if (!manual_request_pending_) return;
        completeManualRequest(manual_request_id_, manual_pose_stamp_ns_, false);
    }

    void finishMatch(MatchWork work) {
        // Only the ROS executor owns navigation state. A worker from an old
        // odometry epoch or superseded request can never restore localization.
        if (work.generation != match_generation_.load()) return;
        automatic_initialization_generation_ = -1;
        last_fitness_ = work.fitness;
        last_match_duration_s_ = work.elapsed_s;
        const double observation_age = (now().nanoseconds()-work.stamp)/1e9;
        const bool acceptable = work.result && m_state.message_received &&
            tracking::acceptable(work.prior, work.guess, work.fitness,
                work.config.refine_score_thresh, max_correction_, max_rotation_,
                work.global || work.manual || (!m_state.localize_success && !work.confirming));
        const double allowed_age = work.global || work.manual ?
            work.result_timeout_s : std::min(5., match_timeout_);
        const bool age_valid = tracking::observationFresh(
            observation_age, dataAge(), allowed_age);
        const bool initialization = work.global || work.manual || work.confirming;
        const bool wait_for_fresh = acceptable && tracking::canAwaitFreshScan(
            observation_age, dataAge(), allowed_age, initialization);
        bool result = acceptable && (age_valid || wait_for_fresh);
        const bool provisional = result && tracking::needsFreshConfirmation(
            initialization, work.confirming, observation_age, age_valid);
        last_match_observation_age_s_ = observation_age;
        last_match_reason_ = !work.result ? "registration_failed" :
            !m_state.message_received ? "input_unavailable" :
            !acceptable ? "correction_rejected" :
            !age_valid ? "observation_expired" : "accepted";
        const M3D map_body_r = work.guess.block<3,3>(0,0).cast<double>();
        const V3D map_body_t = work.guess.block<3,1>(0,3).cast<double>();
        const M3D map_odom_r = map_body_r * work.odom_rotation.transpose();
        const V3D map_odom_t = map_body_t - map_odom_r * work.odom_translation;
        const double correction_m = (map_body_t - work.prior.block<3,1>(0,3).cast<double>()).norm();
        bool hold = false;
        if (result && !initialization && confirm_translation_ > 0. && correction_m >= confirm_translation_) {
            // Compare both matches at the NEW scan's body pose, allowing actual vehicle motion.
            const auto &p = pending_tracking_;
            const bool agrees = p && p->generation == work.generation && work.stamp > p->stamp &&
                (now().nanoseconds() - p->stamp) / 1e9 <= allowed_age &&
                (p->map_odom_r * work.odom_translation + p->map_odom_t - map_body_t).norm() <= confirm_translation_ * .5 &&
                Eigen::AngleAxisd(p->map_odom_r.transpose() * map_odom_r).angle() <= 3. * M_PI / 180.;
            hold = !agrees;
            last_match_reason_ = hold ? "tracking_correction_pending" : "tracking_correction_confirmed";
            if (hold)
                pending_tracking_ = TrackingCorrection{map_odom_r, map_odom_t, work.stamp, work.generation};
            RCLCPP_INFO(get_logger(), "Tracking correction %.3f m at stamp %ld: %s",
                correction_m, static_cast<long>(work.stamp), last_match_reason_.c_str());
        }
        if (!hold) pending_tracking_.reset();
        if (provisional)
            last_match_reason_ = "awaiting_fresh_confirmation";
        RCLCPP_INFO(
            get_logger(),
            "MATCH_WORK duration_s=%.3f observation_age_s=%.3f "
            "reason=%s global=%d confirming=%d",
            work.elapsed_s, observation_age, last_match_reason_.c_str(),
            work.global, work.confirming);
        updateDiagnostics(work.generation, work.operation, work.diagnostics);
        if (hold) {
            finishDiagnostics("WAITING_TRACKING_CONFIRMATION");
            last_quality_ = 0.;
            publishQuality();
            return; // Preserve the last accepted TF, its timestamp and quality expiry.
        }
        if (result) {
            if (provisional && !candidate_pending_)
                candidate_deadline_ = monotonicSeconds() + std::max(0., allowed_age-observation_age);
            m_state.last_offset_r = map_odom_r;
            m_state.last_offset_t = map_odom_t;
            m_state.localize_success = !provisional;
            candidate_pending_ = provisional;
            m_state.icp_failure_count = 0;
            last_success_ = provisional ? 0. : monotonicSeconds();
            last_success_stamp_ = provisional ? 0 : work.stamp;
            if (!provisional) ++match_sequence_;
            RCLCPP_INFO(get_logger(), "Map match %lu %s: fitness=%.5f", match_sequence_,
                provisional ? "candidate awaiting fresh scan" : "accepted", last_fitness_);
            if (provisional) {
                std::lock_guard<std::mutex> lock(diagnostics_mutex_);
                diagnostics_.match.stage = "verifying_fresh_scan";
                diagnostics_.match.reason_code = "WAITING_FRESH_CONFIRMATION";
                diagnostics_.stage_started = monotonicSeconds();
                ++diagnostics_.match.progress_sequence;
            } else {
                finishDiagnostics();
            }
            if (!provisional && m_use_pose_graph && m_pose_graph)
                m_pose_graph->addKeyFrame(work.stamp/1e9, work.guess, work.cloud);
        } else {
            std::string reason = work.diagnostics.reason_code;
            if (work.result && !age_valid) reason = "RESULT_STALE";
            else if (work.result && !acceptable) reason = "RESULT_REJECTED";
            else if (reason.empty()) reason = "MATCH_FAILED";
            finishDiagnostics(reason);
            ++m_state.icp_failure_count;
            if (candidate_pending_ && m_state.icp_failure_count >= 3) candidate_pending_ = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Map correction rejected; failures=%d, fitness=%.5f", m_state.icp_failure_count, last_fitness_);
        }
        if (request_pending_ && !candidate_pending_) {
            completed_request_id_ = request_id_;
            request_succeeded_ = result;
            request_pending_ = false;
        }
        if (work.manual_request_id != 0 && !candidate_pending_)
            completeManualRequest(
                work.manual_request_id, work.manual_pose_stamp_ns, result);
        last_quality_ = 0;
        publishQuality();
        publishMapCloud(m_state.last_message_time);
        publishMatchingResults(m_state.last_message_time);
    }

    void timerCB() {
        processBuffers();
        if (candidate_pending_ && monotonicSeconds() >= candidate_deadline_) {
            ++match_generation_;
            candidate_pending_ = false;
            m_state.localize_success = false;
            last_success_ = 0.; last_success_stamp_ = 0;
            if (request_pending_) {
                completed_request_id_ = request_id_;
                request_succeeded_ = false;
                request_pending_ = false;
            }
            cancelPendingManualRequest();
            finishDiagnostics("FRESH_INPUT_TIMEOUT");
            last_quality_ = 0.;
        }
        if (match_future_.valid() && match_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            finishMatch(match_future_.get());
        publishQuality();
        sendBroadCastTF();
        if (match_future_.valid() || !m_state.message_received || !m_state.map_loaded) return;
        const double t = monotonicSeconds();
        const int64_t stamp = rclcpp::Time(m_state.last_message_time).nanoseconds();
        const bool confirming_jump = pending_tracking_ && pending_tracking_->generation == match_generation_.load();
        if (dataAge() < -.1 || dataAge() > 1. || stamp == last_attempt_stamp_ ||
            (!confirming_jump && t-last_attempt_ < 1./std::max(.2, std::min(2., m_config.update_hz)))) return;
        const bool manual = m_state.service_received && m_state.skip_global_registration;
        const bool global = m_state.force_sc_relocalization ||
            (!m_state.localize_success && !manual && !initial_attempted_);
        if (!m_state.localize_success && !candidate_pending_ && !manual && !global) return;
        if (global && !request_pending_ && !initial_attempted_ &&
            initialization_window_.size() > 0 && initialization_window_.size() < 3) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for multi-frame initialization: %zu frames, %.3f s",
                initialization_window_.size(), initialization_window_.spanNs()/1e9);
            return;
        }
        if (!global && !manual &&
            m_state.last_cloud->size() < static_cast<std::size_t>(tracking_min_scan_points_)) {
            last_attempt_stamp_ = stamp;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Skipping sparse tracking scan: %zu points (minimum %d)",
                m_state.last_cloud->size(), tracking_min_scan_points_);
            return;
        }
        last_attempt_ = t; last_attempt_stamp_ = stamp;
        initial_attempted_ = true;
        MatchWork work;
        work.global = global; work.manual = manual; work.confirming = candidate_pending_;
        work.generation = match_generation_.load(); work.stamp = stamp;
        if (global && !request_pending_ && !manual && !candidate_pending_ && match_sequence_ == 0)
            automatic_initialization_generation_ = static_cast<int64_t>(work.generation);
        work.result_timeout_s = relocalization_timeout_s_;
        work.operation = manual ? "manual" :
            (global ? "global" : (work.confirming ? "verification" : "tracking"));
        if (manual_request_pending_) {
            work.manual_request_id = manual_request_id_;
            work.manual_pose_stamp_ns = manual_pose_stamp_ns_;
        }
        work.odom_rotation = m_state.last_r; work.odom_translation = m_state.last_t;
        work.cloud = m_state.last_cloud; work.config = m_localizer_config;
        if (global || manual) {
            if (auto window = initialization_window_.build(m_state.last_r, m_state.last_t, stamp))
                work.cloud = std::move(window);
            RCLCPP_INFO(get_logger(), "INITIALIZATION_WINDOW frames=%zu points=%zu span_s=%.3f",
                initialization_window_.size(), work.cloud->size(), initialization_window_.spanNs()/1e9);
        }
        work.guess = tracking::predict(m_state.last_offset_r, m_state.last_offset_t,
                                       m_state.last_r, m_state.last_t);
        if (manual) {
            work.guess.block<3,3>(0,0) = m_state.map_base_r.cast<float>();
            work.guess.block<3,1>(0,3) = m_state.map_base_t.cast<float>();
        }
        work.prior = work.guess;
        if (!global) { work.config.use_global_registration = false; work.config.use_scan_context = false; }
        m_state.service_received = false; m_state.force_sc_relocalization = false;
        m_state.skip_global_registration = false;
        beginDiagnostics(work.operation, "preprocessing");
        // At most ONE worker; no queue of old scans and no shared state mutation.
        // Incoming messages and quality/TF publication stay on the ROS executor.
        match_future_ = std::async(std::launch::async, [this, localizer=m_localizer, work]() mutable {
            const auto started = std::chrono::steady_clock::now();
            try {
                localizer->setCancellationCheck([this, generation=work.generation] {
                    return generation != match_generation_.load() || !rclcpp::ok();
                });
                localizer->cancellationCheckpoint();
                localizer->updateConfig(work.config);
                localizer->setDiagnosticsCallback(
                    [this, generation=work.generation, operation=work.operation]
                    (const MatchDiagnostics &diagnostics) {
                        updateDiagnostics(generation, operation, diagnostics);
                    });
                localizer->setInput(work.cloud);
                work.result = localizer->align(work.guess, work.global, true, work.odom_rotation.cast<float>());
                work.fitness = localizer->getFitnessScore();
                work.diagnostics = localizer->diagnostics();
            } catch (const ICPLocalizer::Cancelled &) {
                work.result = false;
                work.diagnostics.stage = "complete";
                work.diagnostics.reason_code = "REQUEST_CANCELLED";
            } catch (const std::exception &e) {
                work.result = false;
                work.diagnostics.stage = "complete";
                work.diagnostics.reason_code = "WORKER_EXCEPTION";
                std::cerr << "ICP worker exception: " << e.what() << std::endl;
            }
            work.elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            return work;
        });
    }

    static std::string mapDigest(const std::string &path) {
        std::ifstream input(path, std::ios::binary);
        if (!input) throw std::runtime_error("cannot hash loaded map");
        std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
        if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
            throw std::runtime_error("cannot initialize map digest");
        char block[65536];
        while (input.read(block, sizeof block) || input.gcount()) {
            if (EVP_DigestUpdate(context.get(), block, input.gcount()) != 1)
                throw std::runtime_error("cannot update map digest");
        }
        if (!input.eof()) throw std::runtime_error("cannot read map digest");
        unsigned char digest[EVP_MAX_MD_SIZE];
        unsigned int length = 0;
        if (EVP_DigestFinal_ex(context.get(), digest, &length) != 1)
            throw std::runtime_error("cannot finish map digest");
        std::ostringstream result;
        for (unsigned int i=0; i<length; ++i)
            result << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(digest[i]);
        return result.str();
    }

    void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        cloud_buf.push_back(msg);
        if (cloud_buf.size() > max_buffer_size) cloud_buf.pop_front();
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        odom_buf.push_back(msg);
        if (odom_buf.size() > max_buffer_size) odom_buf.pop_front();
    }

    void processBuffers()
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        static int sync_fail_count = 0;
        static rclcpp::Time last_warn_time = rclcpp::Clock().now();

        while (!cloud_buf.empty() && !odom_buf.empty())
        {
            auto cloud_msg = cloud_buf.front();
            auto odom_msg = odom_buf.front();

            double cloud_t = rclcpp::Time(cloud_msg->header.stamp).seconds();
            double odom_t = rclcpp::Time(odom_msg->header.stamp).seconds();
            double diff = std::fabs(cloud_t - odom_t);

            if (diff <= max_time_diff_s)
            {
                sync_fail_count = 0;
                cloud_buf.pop_front();
                odom_buf.pop_front();
                syncCB(std::const_pointer_cast<const sensor_msgs::msg::PointCloud2>(cloud_msg),
                       std::const_pointer_cast<const nav_msgs::msg::Odometry>(odom_msg));
            }
            else
            {
                sync_fail_count++;
                // 每5秒警告一次时间戳不同步问题
                rclcpp::Time now = rclcpp::Clock().now();
                if ((now - last_warn_time).seconds() > 5.0)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "时间戳不同步: 点云=%.3f, 里程计=%.3f, 差值=%.3f秒 (阈值=%.3f秒), 失败次数=%d",
                        cloud_t, odom_t, diff, max_time_diff_s, sync_fail_count);
                    last_warn_time = now;
                }
                if (cloud_t < odom_t) cloud_buf.pop_front();
                else odom_buf.pop_front();
            }
        }
    }

    void invalidateOdometry() {
        ++match_generation_;
        ++odom_epoch_;
        initialization_window_.clear();
        finishDiagnostics("ODOM_EPOCH_CHANGED");
        // The previous map->odom belongs to a different odometry epoch. Do not
        // reinterpret it as a new prior or silently initialize at the origin.
        m_state.localize_success = false;
        m_state.service_received = false;
        m_state.force_sc_relocalization = false;
        m_state.skip_global_registration = false;
        m_state.icp_failure_count = 0;
        initial_attempted_ = true;
        candidate_pending_ = false;
        last_success_ = 0;
        last_success_stamp_ = 0;
        if (request_pending_) {
            completed_request_id_ = request_id_;
            request_succeeded_ = false;
            request_pending_ = false;
        }
        cancelPendingManualRequest();
        last_quality_ = 0;
        RCLCPP_WARN(get_logger(), "Odometry epoch changed; select an initial pose or request global relocalization");
    }

    void rejectInput(const char *reason) {
        if (automatic_initialization_generation_ >= 0 &&
            static_cast<uint64_t>(automatic_initialization_generation_) == match_generation_.load()) {
            // A cancelled first worker is not a completed failed match. Epoch
            // changes and explicit requests have another generation and cannot retry here.
            initial_attempted_ = false;
            automatic_initialization_generation_ = -1;
            RCLCPP_INFO(get_logger(), "Initial match input invalidated; collecting fresh scan history");
        }
        ++match_generation_; // A result computed before a rejected input needs a new observation.
        initialization_window_.clear();
        m_state.message_received = false;
        input_error_ = reason;
        beginDiagnostics("input", "waiting_input");
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            diagnostics_.match.reason_code = "INPUT_REJECTED";
        }
        // If invalid input supersedes an in-flight worker, keep the same
        // request queued for the next valid observation instead of leaving it
        // pending forever with no work capable of completing it.
        if (manual_request_pending_ && !candidate_pending_) {
            m_state.service_received = true;
            m_state.skip_global_registration = true;
        }
        if (request_pending_ && !candidate_pending_)
            m_state.force_sc_relocalization = true;
        last_quality_ = 0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Localization input rejected: %s", reason);
    }

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        if (!tracking::framesValid(cloud_msg->header.frame_id, odom_msg->header.frame_id,
                                  odom_msg->child_frame_id, m_config.map_frame)) {
            rejectInput("cloud_odom_frame_mismatch");
            return;
        }
        if (!tracking::cloudLayoutValid(*cloud_msg)) {
            rejectInput("invalid_cloud_layout");
            return;
        }
        Eigen::Quaterniond odom_q(
            odom_msg->pose.pose.orientation.w,
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z
        );
        if (!odom_q.coeffs().allFinite() || odom_q.norm() < .9 || odom_q.norm() > 1.1) {
            rejectInput("invalid_odometry_quaternion");
            return;
        }
        const auto &position = odom_msg->pose.pose.position;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
            rejectInput("invalid_odometry_position");
            return;
        }
        CloudType::Ptr clean(new CloudType);
        try { pcl::fromROSMsg(*cloud_msg, *clean); }
        catch (const std::exception &) { rejectInput("invalid_cloud_conversion"); return; }
        clean->erase(std::remove_if(clean->begin(), clean->end(), [](const auto &point) {
            return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
        }), clean->end());
        if (clean->size() < 20) { rejectInput("insufficient_finite_points"); return; }
        clean->is_dense = true;
        const int64_t odom_stamp = rclcpp::Time(odom_msg->header.stamp).nanoseconds();
        const V3D next_position(position.x, position.y, position.z);
        const M3D next_rotation = odom_q.normalized().toRotationMatrix();
        const double dt = (odom_stamp - last_odom_stamp_) / 1e9;
        if (last_odom_stamp_ > 0 &&
            (odom_msg->header.frame_id != m_config.local_frame || odom_msg->child_frame_id != body_frame_ ||
             dt < -.1 || (next_position - m_state.last_t).norm() > std::max(2., 5.*std::max(0., dt)) ||
             Eigen::AngleAxisd(next_rotation * m_state.last_r.transpose()).angle() >
                 std::max(M_PI/3., 2.*M_PI*std::max(0., dt)))) {
            invalidateOdometry();
        } else if (last_odom_stamp_ > 0 && dt <= 0) {
            initialization_window_.clear();
            return; // Duplicate or slightly reordered input is not fresh evidence.
        }
        input_error_.clear();
        last_odom_stamp_ = odom_stamp;
        body_frame_ = odom_msg->child_frame_id;
        m_config.local_frame = odom_msg->header.frame_id;
        m_state.last_cloud = clean;
        m_state.last_r = odom_q.normalized().toRotationMatrix();
        m_state.last_t = V3D(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z
        );
        m_state.last_message_time = cloud_msg->header.stamp;
        initialization_window_.setContext(odom_msg->header.frame_id, body_frame_, odom_epoch_);
        const auto buffered = initialization_window_.push(
            clean, m_state.last_r, m_state.last_t, rclcpp::Time(cloud_msg->header.stamp).nanoseconds());
        if (!buffered.accepted)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Initialization history unavailable (%d); keeping full current scan",
                static_cast<int>(buffered.rejection));

        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void globalRelocalizeCB(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        if (!m_state.map_loaded) {
            response->success = false;
            response->message = "map is not loaded";
            return;
        }
        if (request_pending_) {
            response->success = false;
            response->message = "relocalization is already pending";
            return;
        }

        ++match_generation_;
        // A global request supersedes an unfinished manual prior.  Publish its
        // failed terminal identity together with the newly queued global work.
        cancelPendingManualRequest();
        candidate_pending_ = false;
        m_state.service_received = false;
        m_state.force_sc_relocalization = true;
        m_state.skip_global_registration = false;
        m_state.need_icp_match = true;
        m_state.icp_failure_count = 0;
        response->success = true;
        request_pending_ = true;
        request_succeeded_ = false;
        ++request_id_;
        beginDiagnostics("global", "waiting_input");
        last_quality_ = 0;
        response->message = "queued:" + std::to_string(request_id_) + ":" + instance_id_;
        publishQuality();
        RCLCPP_INFO(
            this->get_logger(),
            "������ 收到全局重定位请求：Scan Context + GICP，TEASER++ 兜底");
    }

    void cancelRelocalizeCB(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        const bool pending = request_pending_ || manual_request_pending_ ||
            m_state.service_received || m_state.force_sc_relocalization ||
            match_future_.valid();
        if (!pending) {
            response->success = false;
            response->message = "no relocalization is pending";
            return;
        }
        ++match_generation_;
        if (request_pending_) {
            completed_request_id_ = request_id_;
            request_succeeded_ = false;
            request_pending_ = false;
        }
        cancelPendingManualRequest();
        m_state.service_received = false;
        m_state.force_sc_relocalization = false;
        m_state.skip_global_registration = false;
        candidate_pending_ = false;
        // Cancelling a request queued before the first scan also cancels the
        // implicit startup attempt; a later scan must not restart it silently.
        initial_attempted_ = true;
        finishDiagnostics("REQUEST_CANCELLED");
        last_quality_ = 0.;
        publishQuality();
        response->success = true;
        response->message = "relocalization result cancelled";
        RCLCPP_WARN(
            get_logger(),
            "Relocalization result cancelled; an in-flight GICP call may finish before its worker is reclaimed");
    }

    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!m_state.map_loaded)
        {
            RCLCPP_WARN(this->get_logger(), "Map not loaded, cannot relocalize");
            return;
        }

        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        Eigen::Vector3d t(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);

        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        if (msg->header.frame_id != m_config.map_frame || !q.coeffs().allFinite() ||
            q.norm() < .5 || !t.allFinite()) return;
        ++match_generation_;
        if (request_pending_) {
            completed_request_id_ = request_id_; request_succeeded_ = false; request_pending_ = false;
        }
        // A newer manual prior always supersedes the previous one, even when
        // both publishers use a zero or repeated ROS header stamp.
        cancelPendingManualRequest();
        ++manual_request_id_;
        manual_pose_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        manual_request_pending_ = true;
        beginDiagnostics("manual", "waiting_input");
        candidate_pending_ = false;
        m_state.map_base_r = q.normalized().toRotationMatrix();
        m_state.localize_success = false;
        last_success_ = 0;
        last_success_stamp_ = 0;
        m_state.map_base_t = t;
        m_state.service_received = true;
        m_state.need_icp_match = true; // 收到initialpose后需要重新匹配
        m_state.icp_failure_count = 0; // 重置失败计数器

        // /initialpose 始终表示人工先验，只执行局部GICP，不再用“接近原点”猜测操作意图。
        m_state.skip_global_registration = true;
        m_state.force_sc_relocalization = false;
        last_quality_ = 0;
        publishQuality();
        RCLCPP_INFO(
            this->get_logger(),
            "������ 手动位姿模式：使用用户先验 (%.2f, %.2f, %.2f) 执行局部GICP",
            t.x(), t.y(), t.z());
    }

    void sendBroadCastTF()
    {
        if (!localizationValid()) return;
        // The alignment changes at matching rate; 100 Hz TF crowds out the
        // measured odometry writer on shared /tf readers under GUI load.
        const double publish_time = monotonicSeconds();
        if (publish_time - last_tf_publish_ < .05) return;
        last_tf_publish_ = publish_time;
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        // This is the current verified map/odom alignment, not a retimed scan pose.
        transformStamped.header.stamp = now();

        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;

        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1) return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (!map_cloud || map_cloud->size() < 1) return;

        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

    void publishMatchingResults(builtin_interfaces::msg::Time &time)
    {
        if (m_teaser_source_pub->get_subscription_count() + m_teaser_target_pub->get_subscription_count() +
            m_teaser_corr_pub->get_subscription_count() + m_rough_source_pub->get_subscription_count() +
            m_rough_target_pub->get_subscription_count() + m_refine_source_pub->get_subscription_count() +
            m_refine_target_pub->get_subscription_count() == 0) return;
        // 发布TEASER++的source和target点云
        if (m_localizer->hasTeaserResult())
        {
            CloudType::Ptr teaser_src = m_localizer->getTeaserSourceCloud();
            CloudType::Ptr teaser_tgt = m_localizer->getTeaserTargetCloud();
            M4F teaser_transform = m_localizer->getTeaserTransform();

            if (teaser_src && teaser_src->size() > 0)
            {
                // 将source变换到map frame，这样才能和target对齐可视化
                CloudType::Ptr teaser_src_transformed(new CloudType);
                pcl::transformPointCloud(*teaser_src, *teaser_src_transformed, teaser_transform);

                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*teaser_src_transformed, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_teaser_source_pub->publish(msg);
            }

            if (teaser_tgt && teaser_tgt->size() > 0)
            {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*teaser_tgt, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_teaser_target_pub->publish(msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "发布TEASER++点云: source=%zu, target=%zu",
                    teaser_src->size(), teaser_tgt->size());
            }

            // 发布对应关系连线
            auto correspondences = m_localizer->getTeaserCorrespondences();
            if (!correspondences.empty() && teaser_src && teaser_tgt)
            {
                visualization_msgs::msg::MarkerArray marker_array;

                // 创建连线
                for (size_t i = 0; i < correspondences.size(); ++i)
                {
                    auto [src_idx, tgt_idx] = correspondences[i];

                    if (src_idx < 0 || tgt_idx < 0 ||
                        static_cast<std::size_t>(src_idx) >= teaser_src->size() ||
                        static_cast<std::size_t>(tgt_idx) >= teaser_tgt->size())
                        continue;

                    // 计算对应点对之间的距离
                    const auto& src_pt = teaser_src->points[src_idx];
                    const auto& tgt_pt = teaser_tgt->points[tgt_idx];
                    float dist = std::sqrt(
                        std::pow(src_pt.x - tgt_pt.x, 2) +
                        std::pow(src_pt.y - tgt_pt.y, 2) +
                        std::pow(src_pt.z - tgt_pt.z, 2));

                    visualization_msgs::msg::Marker line_marker;
                    line_marker.header.frame_id = m_config.map_frame;
                    line_marker.header.stamp = time;
                    line_marker.ns = "teaser_correspondences";
                    line_marker.id = i;
                    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                    line_marker.action = visualization_msgs::msg::Marker::ADD;
                    line_marker.scale.x = 0.02;  // 线宽

                    // 根据距离设置颜色
                    if (dist < 1.0)
                    {
                        // 绿色：好的匹配
                        line_marker.color.r = 0.0;
                        line_marker.color.g = 1.0;
                        line_marker.color.b = 0.0;
                        line_marker.color.a = 0.8;
                    }
                    else if (dist < 2.0)
                    {
                        // 黄色：一般的匹配
                        line_marker.color.r = 1.0;
                        line_marker.color.g = 1.0;
                        line_marker.color.b = 0.0;
                        line_marker.color.a = 0.6;
                    }
                    else
                    {
                        // 红色：差的匹配
                        line_marker.color.r = 1.0;
                        line_marker.color.g = 0.0;
                        line_marker.color.b = 0.0;
                        line_marker.color.a = 0.3;
                    }

                    // 添加source点（配准后的位置）
                    geometry_msgs::msg::Point p_src;
                    Eigen::Vector4f src_vec(src_pt.x, src_pt.y, src_pt.z, 1.0f);
                    Eigen::Vector4f src_transformed = teaser_transform * src_vec;
                    p_src.x = src_transformed[0];
                    p_src.y = src_transformed[1];
                    p_src.z = src_transformed[2];

                    // 添加target点
                    geometry_msgs::msg::Point p_tgt;
                    p_tgt.x = tgt_pt.x;
                    p_tgt.y = tgt_pt.y;
                    p_tgt.z = tgt_pt.z;

                    line_marker.points.push_back(p_src);
                    line_marker.points.push_back(p_tgt);

                    marker_array.markers.push_back(line_marker);
                }

                m_teaser_corr_pub->publish(marker_array);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "发布TEASER++对应关系连线: %zu 对", correspondences.size());
            }
        }

        // 发布GICP粗匹配的source和target点云
        if (m_localizer->hasRoughResult())
        {
            CloudType::Ptr rough_src = m_localizer->getRoughSourceCloud();
            CloudType::Ptr rough_tgt = m_localizer->getRoughTargetCloud();
            M4F rough_transform = m_localizer->getRoughTransform();

            if (rough_src && rough_src->size() > 0)
            {
                // 将source变换到map frame
                CloudType::Ptr rough_src_transformed(new CloudType);
                pcl::transformPointCloud(*rough_src, *rough_src_transformed, rough_transform);

                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*rough_src_transformed, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_rough_source_pub->publish(msg);
            }

            if (rough_tgt && rough_tgt->size() > 0)
            {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*rough_tgt, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_rough_target_pub->publish(msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "发布GICP粗匹配点云: source=%zu, target=%zu",
                    rough_src->size(), rough_tgt->size());
            }
        }

        // 发布GICP精匹配的source和target点云
        if (m_localizer->hasRefineResult())
        {
            CloudType::Ptr refine_src = m_localizer->getRefineSourceCloud();
            CloudType::Ptr refine_tgt = m_localizer->getRefineTargetCloud();
            M4F refine_transform = m_localizer->getRefineTransform();

            if (refine_src && refine_src->size() > 0)
            {
                // 将source变换到map frame
                CloudType::Ptr refine_src_transformed(new CloudType);
                pcl::transformPointCloud(*refine_src, *refine_src_transformed, refine_transform);

                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*refine_src_transformed, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_refine_source_pub->publish(msg);
            }

            if (refine_tgt && refine_tgt->size() > 0)
            {
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*refine_tgt, msg);
                msg.header.frame_id = m_config.map_frame;
                msg.header.stamp = time;
                m_refine_target_pub->publish(msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "发布GICP精匹配点云: source=%zu, target=%zu",
                    refine_src->size(), refine_tgt->size());
            }
        }
    }

private:
    std::future<MatchWork> match_future_;
    std::atomic<uint64_t> match_generation_{0};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
    std::string map_sha256_, instance_id_, body_frame_, input_error_;
    int64_t last_odom_stamp_ = 0;
    int tracking_min_scan_points_ = 0;
    double confirm_translation_ = 0.;
    struct TrackingCorrection {
        M3D map_odom_r;
        V3D map_odom_t;
        int64_t stamp;
        uint64_t generation;
    };
    std::optional<TrackingCorrection> pending_tracking_;
    double match_timeout_ = 5., max_correction_ = .5, max_rotation_ = 15.;
    double relocalization_timeout_s_ = 120.;
    double last_attempt_ = 0, last_success_ = 0, last_quality_ = 0, last_fitness_ = -1.;
    double last_match_duration_s_ = 0.;
    double last_match_observation_age_s_ = -1.;
    double candidate_deadline_ = 0.;
    std::string last_match_reason_ = "not_attempted";
    int64_t last_attempt_stamp_ = -1, last_success_stamp_ = 0;
    uint64_t match_sequence_ = 0, request_id_ = 0, completed_request_id_ = 0;
    bool initial_attempted_ = false, request_pending_ = false, request_succeeded_ = false;
    int64_t automatic_initialization_generation_ = -1;
    double last_tf_publish_ = -1.;
    uint64_t manual_request_id_ = 0, completed_manual_request_id_ = 0;
    int64_t manual_pose_stamp_ns_ = 0, completed_manual_pose_stamp_ns_ = 0;
    bool manual_request_pending_ = false, manual_request_succeeded_ = false;
    bool candidate_pending_ = false;
    tracking::InitializationWindow initialization_window_;
    uint64_t odom_epoch_ = 0;
    std::mutex diagnostics_mutex_;
    DiagnosticState diagnostics_;
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    std::shared_ptr<PoseGraphManager> m_pose_graph; // Pose Graph Manager
    bool m_use_pose_graph = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_global_relocalize_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_cancel_relocalize_service;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        m_parameter_callback_handle;

    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buf;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buf;
    std::mutex buffer_mutex;
    const size_t max_buffer_size = 20;  // 增加缓冲区大小，保留更多定位日志
    const double max_time_diff_s;

    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;

    // 匹配过程可视化的发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_teaser_source_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_teaser_target_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_teaser_corr_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_rough_source_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_rough_target_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_refine_source_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_refine_target_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}

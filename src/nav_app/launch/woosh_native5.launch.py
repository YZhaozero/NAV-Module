"""Native .5 driver service and paired IMU; camera stays on the /foo bridge."""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _ensure_native_driver(context):
    # The existing user service owns acquisition; systemctl start is idempotent.
    # Replay must never start a physical radar.
    if (IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context) or
            not IfCondition(LaunchConfiguration("start_driver")).evaluate(context)):
        return []
    subprocess.run(["systemctl", "--user", "start", "nav-livox5.service", "nav-camera1.service"],
                   check=True, timeout=15)
    return [LogInfo(msg="Using native .5 lidar/IMU from nav-livox5.service")]


def generate_launch_description():
    share = get_package_share_directory("nav_app")
    fast = get_package_share_directory("fast_livo")
    sensor_filter = get_package_share_directory("nav_cloud_self_filter")
    simulated = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    defaults = dict(pointcloud_topic="/livox/lidar_192_168_128_5",
                    imu_topic="/livox/imu_192_168_128_5",
                    image_topic="/foo/camera_1/color/image_raw", camera_info_topic="",
                    img_en="1", rviz="false", use_sim_time="false", self_filter="true",
                    imu_rate_odom="false", start_driver="true",
                    params_file=os.path.join(share, "config", "woosh_native5.yaml"),
                    camera_params_file=os.path.join(share, "config", "woosh_camera1.yaml"))
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        OpaqueFunction(function=_ensure_native_driver),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sensor_filter, "launch", "livox5_self_filter.launch.py")), launch_arguments={
                "input_topic": LaunchConfiguration("pointcloud_topic"),
                "output_topic": "/nav/livox5/points_filtered",
                "enabled": LaunchConfiguration("self_filter"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(fast, "launch", "navigation.launch.py")), launch_arguments={
                **{key: LaunchConfiguration(key) for key in (
                    "imu_topic", "image_topic", "camera_info_topic", "img_en", "rviz",
                    "use_sim_time", "params_file", "camera_params_file", "imu_rate_odom")},
                "pointcloud_topic": "/nav/livox5/points_filtered",
                "body_frame": "woosh_livox5_imu",
            }.items()),
        Node(package="nav_app", executable="woosh_odometry", output="screen",
             condition=IfCondition(LaunchConfiguration("imu_rate_odom")),
             parameters=[{"use_sim_time": simulated, "imu_frame": "woosh_livox5_imu"}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="woosh_imu_to_base",
             parameters=[{"use_sim_time": simulated}], arguments=[
                 "--x", "-0.0690248982235242", "--y", "0", "--z", "-0.7384453692895854",
                 "--qx", "0", "--qy", "-0.25881904510252074", "--qz", "0", "--qw", "0.9659258262890683",
                 "--frame-id", "woosh_livox5_imu", "--child-frame-id", "base_link"]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="woosh_base_footprint",
             parameters=[{"use_sim_time": simulated}], arguments=[
                 "--frame-id", "base_link", "--child-frame-id", "base_footprint"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="woosh_scan_mount", parameters=[{"use_sim_time": simulated}], arguments=[
                 "--x", "0", "--y", "0", "--z", "0.329",
                 "--roll", "0", "--pitch", "0", "--yaw", "0",
                 "--frame-id", "base_footprint", "--child-frame-id", "laser_link"]),
        # Copy only this robot's calibrated camera transforms, not ROS 1 map/odom TF.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="woosh_camera1_mount", parameters=[{"use_sim_time": simulated}], arguments=[
                 "--x", "0.398", "--y", "0", "--z", "0.215",
                 "--qx", "0.011386218188704486", "--qy", "-0.005142377179311725",
                 "--qz", "-0.012441454847957095", "--qw", "0.9998445480141257",
                 "--frame-id", "base_footprint", "--child-frame-id", "camera_1_color_frame"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="woosh_camera1_optical", parameters=[{"use_sim_time": simulated}], arguments=[
                 "--x", "0", "--y", "0.007", "--z", "0",
                 "--qx", "0.4999999999999998", "--qy", "0.49999998660255174",
                 "--qz", "0.4999999999999998", "--qw", "0.5000000133974483",
                 "--frame-id", "camera_1_color_frame", "--child-frame-id", "camera_1_color_optical_frame"]),
    ])

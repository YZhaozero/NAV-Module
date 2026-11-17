"""WOOSH FAST-LIVO2 uses existing bridge data; never starts sensor/base drivers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("nav_app")
    fast = get_package_share_directory("fast_livo")
    simulated = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    defaults = dict(pointcloud_topic="/foo/livox/lidar_192_168_128_3",
                    imu_topic="/foo/livox/imu_192_168_128_3",
                    image_topic="/foo/camera_1/color/image_raw", camera_info_topic="",
                    img_en="1", rviz="false", use_sim_time="false", imu_rate_odom="false",
                    params_file=os.path.join(share, "config", "woosh_fast_livo.yaml"),
                    camera_params_file=os.path.join(share, "config", "woosh_camera1.yaml"))
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(fast, "launch", "navigation.launch.py")), launch_arguments={
                **{key: LaunchConfiguration(key) for key in (
                    "pointcloud_topic", "imu_topic", "image_topic", "camera_info_topic", "img_en", "rviz",
                    "use_sim_time", "params_file", "camera_params_file", "imu_rate_odom")},
                "body_frame": "woosh_livox3_imu",
            }.items()),
        Node(package="nav_app", executable="woosh_odometry", output="screen",
             condition=IfCondition(LaunchConfiguration("imu_rate_odom")),
             parameters=[{"use_sim_time": simulated}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="woosh_imu_to_base",
             parameters=[{"use_sim_time": simulated}], arguments=[
                 "--x", "-0.3733523804664971", "--y", "0.10465180361560904", "--z", "-0.330",
                 "--qx", "0", "--qy", "0", "--qz", "-0.3826834323650898", "--qw", "0.9238795325112867",
                 "--frame-id", "woosh_livox3_imu", "--child-frame-id", "base_link"]),
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

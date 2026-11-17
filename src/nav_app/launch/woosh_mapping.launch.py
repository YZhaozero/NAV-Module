"""WOOSH .3 mapping only: existing ROS 1 bridge supplies every sensor."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("nav_app")
    dlio = get_package_share_directory("direct_lidar_inertial_odometry")
    simulated = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("pointcloud_topic", default_value="/foo/livox/lidar_192_168_128_3"),
        DeclareLaunchArgument("imu_topic", default_value="/foo/livox/imu_192_168_128_3"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(package="nav_app", executable="woosh_imu", output="screen",
             parameters=[{"input_topic": LaunchConfiguration("imu_topic"), "use_sim_time": simulated}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(dlio, "launch", "dlio.launch.py")), launch_arguments={
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "imu_topic": "/nav/woosh/imu3_si",
                "params_file": os.path.join(share, "config", "woosh_dlio.yaml"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rviz": LaunchConfiguration("rviz"),
            }.items()),
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

# Copyright 2026 NAV contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Launch an Intel RealSense D435i with RGB, depth, and IMU enabled."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_launch = PythonLaunchDescriptionSource(
        [get_package_share_directory("realsense2_camera"), "/launch/rs_launch.py"]
    )

    forwarded_arguments = {
        "camera_name": LaunchConfiguration("camera_name"),
        "camera_namespace": LaunchConfiguration("camera_namespace"),
        "serial_no": LaunchConfiguration("serial_no"),
        "device_type": LaunchConfiguration("device_type"),
        "config_file": LaunchConfiguration("config_file"),
        "enable_color": LaunchConfiguration("enable_color"),
        "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
        "enable_depth": LaunchConfiguration("enable_depth"),
        "color_qos": LaunchConfiguration("image_qos"),
        "depth_qos": LaunchConfiguration("image_qos"),
        "enable_gyro": LaunchConfiguration("enable_gyro"),
        "enable_accel": LaunchConfiguration("enable_accel"),
        "unite_imu_method": LaunchConfiguration("unite_imu_method"),
        "enable_sync": LaunchConfiguration("enable_sync"),
        "align_depth.enable": LaunchConfiguration("align_depth.enable"),
        "pointcloud.enable": LaunchConfiguration("pointcloud.enable"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            # d435 matches D435 and D435i by device-name substring; keeps module1 working.
            DeclareLaunchArgument("device_type", default_value="d435"),
            DeclareLaunchArgument("serial_no", default_value="''"),
            DeclareLaunchArgument("config_file", default_value="''"),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("color_profile", default_value="0,0,0"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            # Explicit reliable, volatile KEEP_LAST(10); system defaults lost depth frames on the device.
            DeclareLaunchArgument("image_qos", default_value="DEFAULT"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("unite_imu_method", default_value="2"),
            DeclareLaunchArgument("enable_sync", default_value="true"),
            DeclareLaunchArgument("align_depth.enable", default_value="true"),
            DeclareLaunchArgument("pointcloud.enable", default_value="false"),
            IncludeLaunchDescription(
                camera_launch,
                launch_arguments=forwarded_arguments.items(),
            ),
        ]
    )

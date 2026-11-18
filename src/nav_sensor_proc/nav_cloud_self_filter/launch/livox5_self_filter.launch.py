"""Standalone WOOSH .5 self filter; never starts sensors or robot motion."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("nav_cloud_self_filter")
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/livox/lidar_192_168_128_5"),
        DeclareLaunchArgument("output_topic", default_value="/nav/livox5/points_filtered"),
        DeclareLaunchArgument("filter_params_file", default_value=os.path.join(share, "config", "woosh_livox5.yaml")),
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="nav_cloud_self_filter", executable="cloud_self_filter", name="nav_livox5_self_filter",
             output="screen", parameters=[LaunchConfiguration("filter_params_file"), {
                 "enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
             remappings=[("input", LaunchConfiguration("input_topic")),
                         ("output", LaunchConfiguration("output_topic"))]),
    ])

"""Standalone WOOSH localization and navigation, without sensor drivers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav = get_package_share_directory("nav_app")
    localizer = get_package_share_directory("localizer")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Navigation YAML paired with the PCD"),
        DeclareLaunchArgument("pcd", description="Localization PCD in the same map coordinates"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_base_control", default_value="false"),
        DeclareLaunchArgument("required_control_mode", default_value="0"),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav, "launch/woosh_fast_livo.launch.py")),
                                 launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time"),
                                     "imu_rate_odom": "true"}.items())]),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localizer, "launch/localizer_launch.py")),
                                 launch_arguments={"map": LaunchConfiguration("pcd"),
                                     "cloud_topic": "/fast_livo/cloud_body",
                                     "odom_topic": "/aft_mapped_to_init",
                                     "sc_use_odom_gravity": "true",
                                     "use_sim_time": LaunchConfiguration("use_sim_time")}.items())]),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav, "launch/woosh_nav2.launch.py")),
                                 launch_arguments={"map": LaunchConfiguration("map"),
                                     "odom_topic": "/nav/woosh/odom",
                                     "use_sim_time": LaunchConfiguration("use_sim_time"),
                                     "enable_base_control": LaunchConfiguration("enable_base_control"),
                                     "required_control_mode": LaunchConfiguration("required_control_mode")}.items())]),
    ])

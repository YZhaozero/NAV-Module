"""Explicitly start NAV's software motion boundary.

The base adapter stays disabled unless the operator supplies
``start_base_adapter:=true`` and configured base topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_base_adapter", default_value="false"),
        DeclareLaunchArgument("planner_topic", default_value="/nav/planner_cmd_vel"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("base_odom_topic", default_value="/base/odom"),
        DeclareLaunchArgument("enable_topic", default_value="/base/motion_enabled"),
        Node(
            package="nav_app",
            executable="motion_guard",
            name="nav_motion_guard",
            output="screen",
        ),
        Node(
            package="nav_app",
            executable="ros2_base_adapter",
            name="nav_ros2_base_adapter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_base_adapter")),
            parameters=[{
                "planner_topic": LaunchConfiguration("planner_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "base_odom_topic": LaunchConfiguration("base_odom_topic"),
                "enable_topic": LaunchConfiguration("enable_topic"),
            }],
        ),
    ])

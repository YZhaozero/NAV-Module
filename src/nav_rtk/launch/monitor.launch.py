from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("fix_topic", default_value="/rtk/fix"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="nav_rtk", executable="gnss_monitor", output="screen",
             parameters=[{"fix_topic": LaunchConfiguration("fix_topic"),
                          "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}])])

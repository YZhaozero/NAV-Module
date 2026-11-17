from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("baudrate", default_value="230400"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="nav_rtk", executable="rtk_node", name="rtk_node", output="screen",
             parameters=[str(Path(get_package_share_directory("nav_rtk"))/"config/rtk.yaml"), {
                 "port": LaunchConfiguration("port"),
                 "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}])])

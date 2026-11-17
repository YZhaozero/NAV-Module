from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("nav_app")
    config = os.path.join(package_share, "config", "nav.yaml")
    default_pointcloud = os.path.join(
        package_share, "maps", "dilo_map.pcd")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fullscreen", default_value="false",
                              description="Start fullscreen at the current screen resolution"),
        DeclareLaunchArgument(
            "pointcloud_map_path",
            default_value=default_pointcloud,
            description="PCD/PLY loaded when the navigation UI starts; empty disables it",
        ),
        DeclareLaunchArgument(
            "fullscreen_on_small_screen",
            default_value="true",
            description="Use fullscreen mode when either screen edge is below 700 px",
        ),
        DeclareLaunchArgument("phone_remote_enabled", default_value="false"),
        DeclareLaunchArgument("phone_remote_bind", default_value="0.0.0.0"),
        DeclareLaunchArgument("phone_remote_port", default_value="8765"),
        Node(
            package="nav_app",
            executable="qt_nav_node",
            name="nav_qt_nav",
            output="screen",
            parameters=[config, {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "fullscreen": ParameterValue(
                    LaunchConfiguration("fullscreen"), value_type=bool),
                "pointcloud_map_path": LaunchConfiguration(
                    "pointcloud_map_path"),
                "fullscreen_on_small_screen": LaunchConfiguration(
                    "fullscreen_on_small_screen"),
                "phone_remote_enabled": ParameterValue(
                    LaunchConfiguration("phone_remote_enabled"), value_type=bool),
                "phone_remote_bind": LaunchConfiguration("phone_remote_bind"),
                "phone_remote_port": ParameterValue(
                    LaunchConfiguration("phone_remote_port"), value_type=int),
            }],
        ),
    ])

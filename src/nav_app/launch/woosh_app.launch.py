"""WOOSH desktop profile; mapping starts only from the mapping page."""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_camera_receiver(context):
    subprocess.run(["systemctl", "--user", "start", "nav-camera1.service"],
                   check=True, timeout=15)
    return []


def generate_launch_description():
    share = get_package_share_directory("nav_app")
    return LaunchDescription([
        DeclareLaunchArgument("operating_mode", default_value="walking", choices=["walking", "robot"]),
        DeclareLaunchArgument("woosh_control_mode", default_value="0", choices=["0", "1", "2"]),
        DeclareLaunchArgument("fullscreen", default_value="false"),
        DeclareLaunchArgument("phone_remote_enabled", default_value="false"),
        OpaqueFunction(function=_start_camera_receiver),
        Node(package="nav_app", executable="qt_nav_node", name="nav_qt_nav", output="screen",
             parameters=[os.path.join(share, "config", "nav.yaml"), {
                 "use_sim_time": False,
                 "phone_remote_enabled": ParameterValue(
                     LaunchConfiguration("phone_remote_enabled"), value_type=bool),
                 "fullscreen": ParameterValue(LaunchConfiguration("fullscreen"), value_type=bool),
                 "camera_topic": "/foo/camera_1/color/image_raw",
                 "camera_info_topic": "/foo/camera_1/color/camera_info",
                 "camera_frame": "camera_1_color_optical_frame",
                 "map_frame": "map", "ground_height_m": 0.0,
                 "camera_height_m": 0.215,
                 "mapping_backend": "woosh_native5",
                 "robot_profile": "woosh",
                 "operating_mode": LaunchConfiguration("operating_mode"),
                 "woosh_control_mode": ParameterValue(
                     LaunchConfiguration("woosh_control_mode"), value_type=int),
                 "pointcloud_map_path": "",
                 "map_storage_dir": os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/nav_maps")),
                 "pointcloud_topic": "",
             }]),
    ])

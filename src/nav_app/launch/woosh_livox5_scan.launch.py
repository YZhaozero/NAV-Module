"""Use UDP for the native .5 scan process without changing other launch environments."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sensor_filter = get_package_share_directory("nav_cloud_self_filter")
    converter = get_package_share_directory("pointcloud_to_laserscan")
    return LaunchDescription([GroupAction(actions=[
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE",
                               os.path.join(sensor_filter, "config", "fastdds_udp.xml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(converter, "launch", "pointcloud_to_laserscan_launch.py"))),
    ])])

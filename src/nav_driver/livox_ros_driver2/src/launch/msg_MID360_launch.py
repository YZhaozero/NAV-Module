import os
from os.path import expanduser, isfile, join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_share = get_package_share_directory('livox_ros_driver2')
def _default_user_config_path():
    """机器专用配置优先（部署约定 §4）: MID360_CONFIG > ~/.config/nav > 安装目录公共默认。"""
    env_path = os.environ.get("MID360_CONFIG", "").strip()
    if env_path:
        path = expanduser(env_path)
        if not isfile(path):
            raise RuntimeError(f"环境变量 MID360_CONFIG 指向的文件不存在: {path}")
        return path
    device = expanduser("~/.config/nav/MID360_config.json")
    if isfile(device):
        return device
    return join(package_share, 'config', 'MID360_config.json')


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            "xfer_format": ParameterValue(
                LaunchConfiguration("xfer_format"), value_type=int),
            "multi_topic": ParameterValue(
                LaunchConfiguration("multi_topic"), value_type=int),
            "publish_freq": ParameterValue(
                LaunchConfiguration("publish_freq"), value_type=float),
            "frame_id": LaunchConfiguration("frame_id"),
            "user_config_path": LaunchConfiguration("user_config_path"),
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument("xfer_format", default_value="0"),
        DeclareLaunchArgument("multi_topic", default_value="1"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("frame_id", default_value="base_footprint"),
        DeclareLaunchArgument(
            "user_config_path", default_value=_default_user_config_path()),
        livox_driver,
    ])

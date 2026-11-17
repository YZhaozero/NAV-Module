"""Nav2 with an explicit owner for map -> odom and a selected odometry topic."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def _start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    sim, auto = value("use_sim_time").lower() == "true", value("autostart").lower() == "true"
    share = get_package_share_directory("nav2_bringup")
    configured = RewrittenYaml(source_file=value("params_file"), root_key="",
        param_rewrites={"use_sim_time": str(sim).lower(), "odom_topic": value("odom_topic"),
                        "base_frame_id": "base_link", "robot_base_frame": "base_link",
                        "scan_topic": value("scan_topic"),
                        "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": value("scan_topic"),
                        "local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic": value("scan_topic"),
                        "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": value("scan_topic")}, convert_types=True)
    common = {"use_sim_time": str(sim).lower(), "autostart": str(auto).lower(),
              # Humble evaluates this argument inside a PythonExpression.
              "use_composition": "False", "params_file": configured}
    if value("external_localization").lower() == "true":
        nodes = [
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[{"yaml_filename": value("map"), "use_sim_time": sim}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_map", parameters=[{"use_sim_time": sim,
                    "autostart": auto, "node_names": ["map_server"]}]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "navigation_launch.py")), launch_arguments=common.items())]
    else:
        nodes = [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "bringup_launch.py")),
            launch_arguments={**common, "map": value("map"), "rviz": "false"}.items())]
    if value("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": sim}],
            arguments=["-d", os.path.join(share, "rviz", "nav2_default_view.rviz")]))
    return [GroupAction(actions=[
        SetRemap(src="/cmd_vel", dst="/nav/planner_cmd_vel"),
        SetRemap(src="/cmd_vel_nav", dst="/nav/controller_cmd_vel"), *nodes])]



def generate_launch_description():
    share = get_package_share_directory("nav2_bringup")
    defaults = {"map": "", "use_sim_time": "false", "autostart": "true", "rviz": "false",
                "external_localization": "true", "odom_topic": "/odom", "scan_topic": "/scan",
                "params_file": os.path.join(share, "params", "nav2_params.yaml")}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              OpaqueFunction(function=_start)])

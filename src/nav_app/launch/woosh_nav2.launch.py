"""WOOSH Nav2: velocity stays isolated unless the SDK adapter is explicitly enabled."""
import os
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node


def start(context):
    value = lambda name: context.launch_configurations[name]
    simulated = value("use_sim_time").lower() == "true"
    autostart = value("autostart").lower() == "true"
    share = Path(get_package_share_directory("nav_app"))
    parameters = yaml.safe_load(Path(value("nav2_params_file")).read_text())
    overrides = yaml.safe_load((share / "config/woosh_nav2.yaml").read_text())
    for name in ("bt_navigator", "controller_server", "velocity_smoother"):
        overrides[name + ".ros__parameters.odom_topic"] = value("odom_topic")
    for area, layer in (("local", "obstacle_layer"), ("local", "voxel_layer"), ("global", "obstacle_layer")):
        overrides[f"{area}_costmap.{area}_costmap.ros__parameters.{layer}.scan.topic"] = value("scan_topic")
    overrides["amcl.ros__parameters.scan_topic"] = value("scan_topic")
    overrides["amcl.ros__parameters.base_frame_id"] = "base_link"
    for dotted, setting in overrides.items():
        keys = dotted.split(".")
        section = parameters
        for key in keys[:-1]:
            section = section.setdefault(key, {})
        section[keys[-1]] = setting
    for section in parameters.values():
        if "ros__parameters" in section:
            section["ros__parameters"]["use_sim_time"] = simulated
        for nested in section.values():
            if isinstance(nested, dict) and "ros__parameters" in nested:
                nested["ros__parameters"]["use_sim_time"] = simulated
    with tempfile.NamedTemporaryFile(mode="w", prefix="woosh-nav2-", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(parameters, stream)
        configured = stream.name
    specs = [
        ("nav2_controller", "controller_server", [("cmd_vel", "/nav/woosh/controller_cmd_vel")]),
        ("nav2_smoother", "smoother_server", []),
        ("nav2_planner", "planner_server", []),
        ("nav2_behaviors", "behavior_server", [("cmd_vel", "/nav/woosh/controller_cmd_vel")]),
        ("nav2_bt_navigator", "bt_navigator", []),
        ("nav2_waypoint_follower", "waypoint_follower", []),
        ("nav2_velocity_smoother", "velocity_smoother", [
            ("cmd_vel", "/nav/woosh/controller_cmd_vel"),
            ("cmd_vel_smoothed", "/nav/planner_cmd_vel")]),
    ]
    nodes = [Node(package=pkg, executable=name, name=name, output="screen",
                  parameters=[configured], remappings=remaps) for pkg, name, remaps in specs]
    localization_nodes = ["map_server"]
    if value("external_localization").lower() != "true":
        nodes.append(Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                          parameters=[configured], remappings=[("scan", value("scan_topic"))]))
        localization_nodes.append("amcl")
    nodes += [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": value("map"), "use_sim_time": simulated}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             parameters=[{"use_sim_time": simulated, "autostart": autostart, "node_names": localization_nodes}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[{"use_sim_time": simulated, "autostart": autostart,
                          "node_names": [name for pkg, name, remaps in specs]}]),
        Node(package="nav_app", executable="motion_guard", name="nav_motion_guard", output="screen"),
    ]
    if value("enable_base_control").lower() == "true":
        nodes.append(Node(package="nav_app", executable="woosh_base_adapter", output="screen",
                          parameters=[{"required_control_mode": int(value("required_control_mode"))}]))
    else:
        nodes.append(LogInfo(msg="WOOSH：仅定位与规划，底盘执行未启用；速度输出为 /nav/planner_cmd_vel"))
    if value("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(
            get_package_share_directory("nav2_bringup"), "rviz/nav2_default_view.rviz")],
            parameters=[{"use_sim_time": simulated}]))
    return nodes


def generate_launch_description():
    defaults = {"map": "", "use_sim_time": "false", "autostart": "true", "rviz": "false",
                "external_localization": "true", "odom_topic": "/dlio/odom_node/odom", "scan_topic": "/foo/scan",
                "enable_base_control": "false", "required_control_mode": "0",
                "nav2_params_file": os.path.join(get_package_share_directory("nav2_bringup"), "params/nav2_params.yaml")}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=start)])

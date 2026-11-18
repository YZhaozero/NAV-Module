"""Map and Nav2 route planner only; localization is supplied by the selected frontend."""
from pathlib import Path
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def start(context):
    value = context.launch_configurations
    simulated = value["use_sim_time"].lower() == "true"
    parameters = yaml.safe_load(Path(value["params_file"]).read_text())
    planner = parameters["planner_server"]["ros__parameters"]
    costmap = parameters["global_costmap"]["global_costmap"]["ros__parameters"]
    planner["use_sim_time"] = costmap["use_sim_time"] = simulated
    costmap["obstacle_layer"]["scan"]["topic"] = value["scan_topic"]
    if value["robot_profile"] == "woosh":
        overrides = yaml.safe_load((Path(get_package_share_directory("nav_app")) / "config/woosh_nav2.yaml").read_text())
        for key in ("footprint", "footprint_padding", "transform_tolerance"):
            costmap[key] = overrides["global_costmap.global_costmap.ros__parameters." + key]
    with tempfile.NamedTemporaryFile(mode="w", prefix="nav-walking-", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump({"planner_server": {"ros__parameters": planner},
                        "global_costmap": {"global_costmap": {"ros__parameters": costmap}}}, stream)
        configured = stream.name
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": value["map"], "use_sim_time": simulated}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[configured]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_walking", output="screen",
             parameters=[{"use_sim_time": simulated, "autostart": value["autostart"].lower() == "true",
                          "node_names": ["map_server", "planner_server"]}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("nav_app"))
    defaults = {"map": "", "scan_topic": "/scan", "robot_profile": "", "use_sim_time": "false",
                "autostart": "true", "params_file": str(share / "config/walking_nav.yaml")}
    return LaunchDescription([*[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
                              OpaqueFunction(function=start)])

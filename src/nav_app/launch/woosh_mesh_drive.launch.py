"""Launch the MeshNav planner and WOOSH guarded execution in one owned process group."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mesh = LaunchConfiguration("mesh")
    return LaunchDescription([
        DeclareLaunchArgument("mesh"),
        DeclareLaunchArgument("params"),
        DeclareLaunchArgument("control_mode", choices=["1", "2"]),
        Node(package="mbf_mesh_nav", executable="mbf_mesh_nav", output="screen",
             parameters=[LaunchConfiguration("params"), {"mesh_map.mesh_file": mesh,
                         "mesh_map.mesh_working_file": [mesh, ".woosh-octagon-v3.h5"]}]),
        Node(package="nav_app", executable="motion_guard", output="screen",
             parameters=[{"max_speed_mps": .10, "max_yaw_rps": .35}]),
        Node(package="nav_app", executable="woosh_base_adapter", output="screen",
             parameters=[{"required_control_mode": ParameterValue(LaunchConfiguration("control_mode"), value_type=int)}])
    ])

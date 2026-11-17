import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "sc_use_odom_gravity",
                default_value="",
                description="Verified map/odom vertical alignment; empty preserves YAML",
            ),
            DeclareLaunchArgument(
                "config_path", default_value=default_config_path,
                description="Localizer YAML configuration file",
            ),
            DeclareLaunchArgument(
                "map", default_value="",
                description="PCD localization map; empty uses localizer.yaml",
            ),
            DeclareLaunchArgument(
                "scan_context_path", default_value="",
                description="Optional explicit .sc.v2 cache bound to the PCD map",
            ),
            DeclareLaunchArgument(
                "cloud_topic", default_value="",
                description="Input cloud override; empty uses localizer.yaml",
            ),
            DeclareLaunchArgument(
                "odom_topic", default_value="",
                description="Odometry override; empty uses localizer.yaml",
            ),
            *[
                DeclareLaunchArgument(
                    name,
                    default_value="-1",
                    description=(
                        "Optional App startup override; -1 preserves YAML"),
                )
                for name in (
                    "rough_score_thresh_override",
                    "refine_score_thresh_override",
                    "rough_scan_resolution_override",
                    "refine_scan_resolution_override",
                    "rough_map_resolution_override",
                    "refine_map_resolution_override",
                    "rough_max_iteration_override",
                    "refine_max_iteration_override",
                )
            ],
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool),
                        "config_path": LaunchConfiguration("config_path"),
                        "sc_use_odom_gravity_override": ParameterValue(
                            LaunchConfiguration("sc_use_odom_gravity"), value_type=str),
                        "default_map_path": ParameterValue(
                            LaunchConfiguration("map"), value_type=str),
                        "scan_context_path_override": ParameterValue(
                            LaunchConfiguration("scan_context_path"), value_type=str),
                        "cloud_topic": ParameterValue(
                            LaunchConfiguration("cloud_topic"), value_type=str),
                        "odom_topic": ParameterValue(
                            LaunchConfiguration("odom_topic"), value_type=str),
                        "rough_score_thresh_override": ParameterValue(
                            LaunchConfiguration("rough_score_thresh_override"),
                            value_type=float),
                        "refine_score_thresh_override": ParameterValue(
                            LaunchConfiguration("refine_score_thresh_override"),
                            value_type=float),
                        "rough_scan_resolution_override": ParameterValue(
                            LaunchConfiguration("rough_scan_resolution_override"),
                            value_type=float),
                        "refine_scan_resolution_override": ParameterValue(
                            LaunchConfiguration("refine_scan_resolution_override"),
                            value_type=float),
                        "rough_map_resolution_override": ParameterValue(
                            LaunchConfiguration("rough_map_resolution_override"),
                            value_type=float),
                        "refine_map_resolution_override": ParameterValue(
                            LaunchConfiguration("refine_map_resolution_override"),
                            value_type=float),
                        "rough_max_iteration_override": ParameterValue(
                            LaunchConfiguration("rough_max_iteration_override"),
                            value_type=int),
                        "refine_max_iteration_override": ParameterValue(
                            LaunchConfiguration("refine_max_iteration_override"),
                            value_type=int),
                    }
                ],
            ),

        ]
    )

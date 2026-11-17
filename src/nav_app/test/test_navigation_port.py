"""Integration contracts for the robot-only navigation migration."""
from concurrent.futures import Future
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from nav_app.features.localization_health import LocalizationHealth
from nav_app.features.localization_tools import LocalizationRosAdapter
from nav_app.features.navigation_stack import NavigationStackConfig, build_navigation_launches
from nav_app.qt_nav_node import QtNavRosNode


def test_fast_livo_switches_all_dependent_cloud_and_odom_contracts():
    config = NavigationStackConfig(
        navigation_map="", localization_map="", odometry_backend="fast_livo",
        localization_backend="disabled", navigation_backend="scan_planner",
        fast_livo_vision=False,
        lidar_arguments=["launch", "livox_ros_driver2", "msg_MID360_launch.py", "xfer_format:=0"],
        camera_arguments=["launch", "realsense2_camera", "d435i.launch.py"])
    launches = build_navigation_launches(config)
    assert "xfer_format:=4" in launches["lidar"]
    assert "xfer_format:=0" not in launches["lidar"]
    assert launches["dlio"][:3] == ["launch", "fast_livo", "navigation.launch.py"]
    assert "img_en:=0" in launches["dlio"]
    assert "cloud_topic:=/fast_livo/cloud_body" in launches["laserscan"]
    assert "target_frame:=base_link" in launches["laserscan"]
    assert "cloud_topic:=/fast_livo/cloud_body" in launches["nav2"]
    assert "body_pose_topic:=/aft_mapped_to_init" in launches["nav2"]
    assert "cmd_vel_topic:=/cmd_vel" in launches["nav2"]
    assert not any("/nav/planner_cmd_vel" in arg for arg in launches["nav2"])


def test_external_localization_launch_excludes_amcl_and_isolates_cmd_vel():
    path = Path(__file__).parents[1] / "launch/navigation_external.launch.py"
    spec = importlib.util.spec_from_file_location("robot_external_navigation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        external_localization="true", use_sim_time="false", autostart="true",
        params_file=str(Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml"),
        map="/tmp/robot-map.yaml", odom_topic="/aft_mapped_to_init", scan_topic="/foo/scan", rviz="false")
    actions = module._start(context)
    from launch.actions import GroupAction
    from launch_ros.actions import SetRemap
    assert len(actions) == 1 and isinstance(actions[0], GroupAction)
    children = actions[0].get_sub_entities()
    assert sum(isinstance(action, SetRemap) for action in children) == 2
    assert sum(isinstance(action, Node) for action in children) == 2
    include = next(action for action in children if isinstance(action, IncludeLaunchDescription))
    source = include.launch_description_source
    source.get_launch_description(context)
    assert source.location.endswith("navigation_launch.py")


def test_quality_requires_recent_map_match_and_sensor_data():
    health = LocalizationHealth()
    row = dict(schema_version=1, valid=True, published_stamp_ns=10_000_000_000,
               match_age_s=.2, data_age_s=.1, match_sequence=1,
               request_id=1, completed_request_id=1, request_succeeded=True)
    health.update(json.dumps(row), received=20.)
    assert health.valid(10_100_000_000, now=20.1)
    assert not health.valid(10_100_000_000, now=22.)
    row["data_age_s"] = 2.
    health.update(json.dumps(row), received=20.)
    assert not health.valid(10_100_000_000, now=20.1)


def test_stale_localization_blocks_robot_goal_and_requests_cancel():
    cancelled = []
    node = SimpleNamespace(
        navigation_client=SimpleNamespace(server_is_ready=lambda: True),
        require_localizer_quality=True, _navigation_input_subscriptions=[],
        localization_tools=SimpleNamespace(health=LocalizationHealth()),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        navigation_route_ready=lambda: True,
        poll_navigation=lambda: None,
        navigation_active=True, cancel_navigation=lambda: cancelled.append(True), _route_wait_status=None)
    assert not QtNavRosNode.send_navigation_waypoints(node, [np.array([1., 2.])])
    assert "定位" in node.navigation_status
    QtNavRosNode._monitor_localization(node)
    assert cancelled == [True]
    assert "请求取消" in node.navigation_status


def test_existing_ui_exposes_fast_livo_without_enabling_uncalibrated_vision():
    from PyQt5.QtWidgets import QApplication, QScrollArea
    from nav_app.qt_nav_node import NavigationWindow
    app = QApplication.instance() or QApplication([])
    node = SimpleNamespace(navigation_status="ready", cancel_navigation=lambda: None,
                           send_navigation_waypoints=lambda points: bool(points))
    window = NavigationWindow(node)
    window.refresh_timer.stop()
    try:
        stack = window.navigation_stack_page
        assert isinstance(stack.parameter_tabs.widget(2), QScrollArea)
        assert stack.configuration().odometry_backend == "dlio"
        assert not stack.fast_livo_lidar_topic.isEnabled()
        stack.odometry_backend.setCurrentIndex(stack.odometry_backend.findData("fast_livo"))
        config = stack.configuration()
        assert config.odometry_backend == "fast_livo" and not config.fast_livo_vision
        assert stack.fast_livo_lidar_topic.isEnabled()
        page = window.mapping_page
        page.algorithm_combo.setCurrentIndex(page.algorithm_combo.findData("fast_livo"))
        assert "img_en:=0" in page.launch_arguments()
        assert page.map_topic.text() == "/fast_livo/map_preview"
        assert page.recover_button.isEnabled()
        assert window.unknown_policy_combo.currentData() is False
    finally:
        window.close()
        app.processEvents()


def test_navigation_stack_stop_clears_localizer_requirement():
    node = SimpleNamespace(
        require_localizer_quality=True, _navigation_input_subscriptions=[],
        _navigation_health_subscription=None,
        _navigation_readiness=SimpleNamespace(reset=lambda: None))
    QtNavRosNode.configure_navigation(node, None)
    assert node.require_localizer_quality is False


def test_map_recovery_cannot_submit_a_concurrent_save():
    from nav_app.features.mapping import MappingController
    controller = SimpleNamespace(_recovery_path=Path("/tmp/recovered.pcd"))
    success, message = MappingController.save_map(controller, "/tmp", "map", .05)
    assert not success and "恢复" in message


def test_malformed_quality_is_rejected_with_a_visible_reason():
    health = LocalizationHealth()
    health.update("not-json")
    assert not health.sample and health.error
    assert not health.valid(10_000_000_000)

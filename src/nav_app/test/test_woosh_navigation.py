"""WOOSH navigation reuses bridged sensors and keeps actuation opt-in."""
import json
import pytest

from nav_app.features.navigation_stack import NavigationStackConfig, build_navigation_launches, validate_navigation_stack
from nav_app.features.woosh_control import decode_command


def test_woosh_stack_uses_paired_input_without_starting_drivers(tmp_path):
    pcd = tmp_path / "room.pcd"; pcd.write_bytes(b"pcd")
    grid = tmp_path / "room.yaml"; grid.write_text("image: room.pgm\n")
    config = NavigationStackConfig(navigation_map=str(grid), localization_map=str(pcd), robot_profile="woosh",
        dlio_pointcloud_topic="/foo/livox/lidar_192_168_128_3", dlio_imu_topic="/foo/livox/imu_192_168_128_3")
    assert validate_navigation_stack(config)[0]
    launches = build_navigation_launches(config)
    assert set(launches) == {"dlio", "localizer", "nav2"}
    assert launches["dlio"][:3] == ["launch", "nav_app", "woosh_mapping.launch.py"]
    assert "imu_topic:=/foo/livox/imu_192_168_128_3" in launches["dlio"]
    assert "cloud_topic:=/dlio/odom_node/pointcloud/body" in launches["localizer"]
    assert "odom_topic:=/dlio/odom_node/scan_odom" in launches["localizer"]
    assert launches["nav2"][:3] == ["launch", "nav_app", "woosh_nav2.launch.py"]
    assert not any("enable_base_control:=true" in args for args in launches.values())
    config.odometry_backend = "fast_livo"
    config.fast_livo_lidar_topic = "/foo/livox/lidar_192_168_128_3"
    config.fast_livo_image_topic = "/foo/camera_1/color/image_raw"
    config.map_gravity_aligned = True
    assert validate_navigation_stack(config)[0]
    launches = build_navigation_launches(config)
    assert set(launches) == {"dlio", "localizer", "nav2"}
    assert launches["dlio"][:3] == ["launch", "nav_app", "woosh_fast_livo.launch.py"]
    assert "pointcloud_topic:=/foo/livox/lidar_192_168_128_3" in launches["dlio"]
    assert "image_topic:=/foo/camera_1/color/image_raw" in launches["dlio"]
    assert "cloud_topic:=/fast_livo/cloud_body" in launches["localizer"]
    assert "odom_topic:=/aft_mapped_to_init" in launches["localizer"]
    assert "sc_use_odom_gravity:=true" in launches["localizer"]
    assert "odom_topic:=/nav/woosh/odom" in launches["nav2"]
    assert "imu_rate_odom:=true" in launches["dlio"]
    assert not any("enable_base_control:=true" in args for args in launches.values())
    config.sensor_source = "bag"
    assert not validate_navigation_stack(config)[0]


def test_guard_command_preserves_axis_order_and_sequence():
    payload = dict(schema=1, frame_id="base_link", stamp_ns=1_000_000_000, sequence=5, velocity=[.1, 0., -.2])
    assert decode_command(json.dumps(payload), 1_100_000_000, 4) == (5, (.1, 0., -.2))


@pytest.mark.parametrize("changed", [
    {"frame_id": "odom"}, {"stamp_ns": 1}, {"sequence": 4},
    {"velocity": [.1, .1, 0.]}, {"velocity": [.6, 0., 0.]},
    {"velocity": [float("nan"), 0., 0.]}, {"velocity": [True, 0., 0.]},
])
def test_invalid_or_stale_external_guard_command_is_rejected(changed):
    payload = dict(schema=1, frame_id="base_link", stamp_ns=1_000_000_000, sequence=5, velocity=[.1, 0., -.2])
    payload.update(changed)
    with pytest.raises(ValueError):
        decode_command(json.dumps(payload), 1_100_000_000, 4)


def test_woosh_ui_seeds_bridge_topics_and_requires_a_map_bundle(tmp_path, monkeypatch):
    monkeypatch.setenv("NAV_ACCESS_SETTINGS", str(tmp_path / "access.ini"))
    from types import SimpleNamespace
    from PyQt5.QtWidgets import QApplication
    from nav_app.qt_nav_node import NavigationWindow
    app = QApplication.instance() or QApplication([])
    node = SimpleNamespace(robot_profile="woosh", navigation_status="ready",
                           cancel_navigation=lambda: None,
                           send_navigation_waypoints=lambda points: bool(points))
    window = NavigationWindow(node)
    window.refresh_timer.stop()
    try:
        config = window.navigation_stack_page.configuration()
        assert config.robot_profile == "woosh"
        assert window.lidar_multi_topic.isChecked()
        assert window.lidar_frame_id.text() == "base_footprint"
        assert window.lidar_xfer_format.currentData() == 0
        assert config.dlio_pointcloud_topic == "/foo/livox/lidar_192_168_128_3"
        assert config.odometry_backend == "fast_livo"
        assert config.fast_livo_lidar_topic == "/foo/livox/lidar_192_168_128_3"
        assert config.fast_livo_image_topic == "/foo/camera_1/color/image_raw"
        assert config.map_gravity_aligned
        assert config.localizer_cloud_topic == "/fast_livo/cloud_body"
        assert config.localizer_odom_topic == "/aft_mapped_to_init"
        assert config.scan_topic == "/foo/scan"
        assert not config.localization_map and not config.navigation_map
    finally:
        window.close()
        app.processEvents()


def test_navigation_health_follows_selected_bridge_lidar_and_keeps_stale_detection():
    from types import SimpleNamespace
    from nav_app import qt_nav_node as qt
    subscriptions, destroyed = [], []
    old_subscription = object()
    node = SimpleNamespace(
        _navigation_lidar_topic="/livox/lidar/pointcloud",
        _navigation_lidar_subscription=old_subscription,
        _last_lidar_message_time=0., _last_camera_message_time=0.,
        _navigation_input_subscriptions=[], _navigation_health_subscription=None, _on_navigation_scan_data=lambda msg: None,
        set_clock_mode=lambda _: None, operating_mode="robot",
        _navigation_readiness=SimpleNamespace(configure=lambda *args, **kwargs: None),
        destroy_subscription=destroyed.append)
    node._on_navigation_lidar_data = lambda msg: qt.QtNavRosNode._on_navigation_lidar_data(node, msg)
    node.create_subscription = lambda msg_type, topic, callback, qos: subscriptions.append((topic, callback)) or subscriptions[-1]
    config = NavigationStackConfig(navigation_map="room.yaml", localization_map="room.pcd",
        robot_profile="woosh", odometry_backend="fast_livo", fast_livo_lidar_topic="/foo/livox/lidar_192_168_128_3",
        dlio_pointcloud_topic="/foo/livox/lidar_192_168_128_3")
    node.tf_listener = SimpleNamespace(node=SimpleNamespace(create_subscription=node.create_subscription, destroy_subscription=node.destroy_subscription))
    node._on_fast_navigation_health = lambda msg: None
    qt.QtNavRosNode.configure_navigation(node, config)
    assert old_subscription in destroyed
    lidar_subscription = next(s for s in subscriptions if s[0] == config.fast_livo_lidar_topic)
    lidar_subscription[1](object())
    assert qt.QtNavRosNode.navigation_sensor_data_active(node, "lidar")
    qt.QtNavRosNode.configure_navigation(node, config)
    assert sum(topic == config.fast_livo_lidar_topic for topic, callback in subscriptions) == 1
    node._last_lidar_message_time = qt.time.monotonic() - 2.1
    assert not qt.QtNavRosNode.navigation_sensor_data_active(node, "lidar")
    config.odometry_backend = "dlio"
    config.dlio_pointcloud_topic = "/foo/other_lidar"
    qt.QtNavRosNode.configure_navigation(node, config)
    assert subscriptions[-1][0] == "/foo/other_lidar"
    assert node._last_lidar_message_time == 0.


def test_fast_livo_readiness_requires_current_odometry_and_tf():
    from types import SimpleNamespace
    import time
    from nav_app.qt_nav_node import QtNavRosNode
    node = SimpleNamespace(_odometry_backend="fast_livo",
        _odometry_received={"fast_livo": time.monotonic()}, base_frame="base_link",
        _navigation_tf_ready=lambda target, source: False)
    assert not QtNavRosNode.navigation_component_ready(node, "dlio")
    node._navigation_tf_ready = lambda target, source: (target, source) == ("odom", "base_link")
    assert QtNavRosNode.navigation_component_ready(node, "dlio")
    node._odometry_received["fast_livo"] -= 2.
    assert not QtNavRosNode.navigation_component_ready(node, "dlio")


def test_localizer_readiness_rejects_invalid_match_or_missing_tf():
    from types import SimpleNamespace
    import json
    from nav_app.qt_nav_node import QtNavRosNode
    from nav_app.features.localization_health import LocalizationHealth
    health = LocalizationHealth()
    node = SimpleNamespace(localization_tools=SimpleNamespace(health=health), map_frame="map",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
        _navigation_tf_ready=lambda target, source: True)
    row = dict(schema_version=1, valid=False, published_stamp_ns=10_000_000_000,
        match_age_s=.1, data_age_s=.1, match_sequence=1, request_id=0, completed_request_id=0)
    health.update(json.dumps(row))
    assert not QtNavRosNode.navigation_component_ready(node, "localizer")
    row["valid"] = True; health.update(json.dumps(row))
    assert QtNavRosNode.navigation_component_ready(node, "localizer")
    node._navigation_tf_ready = lambda target, source: False
    assert not QtNavRosNode.navigation_component_ready(node, "localizer")


def test_navigation_health_is_received_without_gui_odometry_processing():
    from types import SimpleNamespace
    from nav_msgs.msg import Odometry
    from nav_app.qt_nav_node import QtNavRosNode
    import time
    node=SimpleNamespace(_fast_navigation_odometry=True, base_frame="base_link", _odometry_backend="fast_livo",
        _odometry_received={"fast_livo":0.}, _navigation_tf_ready=lambda target,source:True,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    message=Odometry();message.header.frame_id="odom";message.child_frame_id="base_link"
    message.header.stamp.sec=9;message.header.stamp.nanosec=950_000_000
    QtNavRosNode._on_fast_navigation_health(node,message)
    assert QtNavRosNode.navigation_component_ready(node,"dlio")
    node._fast_navigation_odometry=False
    node._odometry_received["fast_livo"]=time.monotonic()-2.
    QtNavRosNode._on_fast_navigation_health(node,message)
    assert not QtNavRosNode.navigation_component_ready(node,"dlio")

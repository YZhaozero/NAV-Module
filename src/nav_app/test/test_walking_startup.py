"""Planning-only launch, UI selection and recoverable input interruptions."""
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from launch import LaunchContext
from nav_app.features.navigation_stack import NavigationStackConfig, NavigationStackController, build_navigation_launches
from nav_app.qt_nav_node import NavigationWindow, QtNavRosNode
from test_navigation_access import Process, Sensors, wait_for


def test_walking_launch_has_no_motion_nodes_and_keeps_chassis_geometry(monkeypatch):
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location("walking_launch", package / "launch/walking_planner.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(package))
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(map="room.yaml", scan_topic="/test/scan", robot_profile="woosh",
        use_sim_time="false", autostart="true", params_file=str(package / "config/walking_nav.yaml"))
    nodes = module.start(context)
    assert [node["executable"] for node in nodes] == ["map_server", "planner_server", "lifecycle_manager"]
    assert nodes[-1]["parameters"][0]["node_names"] == ["map_server", "planner_server"]
    configured = Path(nodes[1]["parameters"][0])
    try:
        params = yaml.safe_load(configured.read_text())
        costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        assert costmap["obstacle_layer"]["scan"]["topic"] == "/test/scan"
        expected = yaml.safe_load((package / "config/woosh_nav2.yaml").read_text())
        assert costmap["footprint"] == expected["global_costmap.global_costmap.ros__parameters.footprint"]
        assert costmap["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    finally:
        configured.unlink()


def test_walking_window_selects_planner_and_keeps_native5_inputs(tmp_path, monkeypatch, application):
    for key in ("NAV_ACCESS_SETTINGS", "NAV_TASK_STATE_DIR", "NAV_RUNTIME_DIR"):
        monkeypatch.setenv(key, str(tmp_path / key))
    node = SimpleNamespace(robot_profile="woosh", operating_mode="walking", navigation_status="ready",
                           map_storage_dir=str(tmp_path / "maps"), mapping_backend="woosh_native5")
    window = NavigationWindow(node)
    window.refresh_timer.stop();window.operations_page.timer.stop()
    try:
        window.navigation_access_page.load_preset("woosh_native5")
        window.navigation_access_page.save()
        config = window.navigation_stack_page.configuration()
        assert config.navigation_backend == "walking"
        assert config.localization_backend == "pointcloud_localizer"
        assert config.odometry_backend == "fast_livo" and config.scan_source == "convert"
        assert config.fast_livo_vision
        assert config.camera_driver_mode == "external"
        window.navigation_access_page.fields["fast_livo_vision"].setChecked(True)
        window.navigation_access_page.save()
        assert window.navigation_stack_page.configuration().fast_livo_vision
        launches = build_navigation_launches(config)
        assert launches["nav2"][:3] == ["launch", "nav_app", "walking_planner.launch.py"]
        assert "scan_topic:=/scan" in launches["nav2"]
        assert "woosh_native5.launch.py" in launches["dlio"]
        assert "start_driver:=false" in launches["dlio"]
    finally:
        window.close();application.processEvents()
        window.operations_page.task_coordinator.thread.join(timeout=5)


def test_walking_input_loss_preserves_processes_and_recovers(tmp_path, monkeypatch, application):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module, "RosLaunchProcess", Process)
    ready = dict(scan=False, external_odom=True, external_localization=True, nav2=True)
    sensors = Sensors()
    controller = NavigationStackController(sensors, component_ready=ready.__getitem__)
    grid = tmp_path / "map.yaml";grid.write_text("image: map.pgm")
    config = NavigationStackConfig(navigation_map=str(grid), localization_map="", navigation_backend="walking",
        odometry_backend="external", localization_backend="external", lidar_driver_mode="external",
        camera_driver_mode="off", scan_source="external", startup_interval_ms=0)
    try:
        assert controller.start(config)[0]
        controller._poll_external_inputs(time.monotonic()-1)
        assert controller.overall_state == "starting"
        assert not controller.processes["nav2"].calls
        ready["scan"] = True
        wait_for(application, lambda: controller.overall_state == "running")
        ready["scan"] = False;ready["nav2"] = False
        controller._monitor_sensor_health()
        assert controller.overall_state == "running"
        assert controller.states["nav2"] == "waiting"
        assert all(process.stops == 0 for process in controller.processes.values())
        ready["scan"] = True;ready["nav2"] = True
        controller._monitor_sensor_health()
        assert controller.states["nav2"] == "running"
        assert len(controller.processes["nav2"].calls) == 1
    finally:
        controller.shutdown()


def test_walking_localization_timeout_keeps_waiting_then_advances(monkeypatch, application):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module, "RosLaunchProcess", Process)
    ready = [False]
    controller = NavigationStackController(Sensors(), component_ready=lambda key: ready[0])
    controller.active_config = NavigationStackConfig(navigation_map="", localization_map="", navigation_backend="walking")
    controller.overall_state = "starting";controller._waiting_key = "localizer"
    controller._queue = ["nav2"]
    later = []
    monkeypatch.setattr(controller, "_later", lambda delay, callback: later.append(callback))
    try:
        controller._poll_component_ready("localizer", time.monotonic()-1)
        assert controller._queue == ["nav2"] and controller._waiting_key == "localizer"
        assert controller.overall_state == "starting"
        assert "重新定位" in controller.log_events[-1][1]
        ready[0] = True;later[-1]()
        assert controller.states["localizer"] == "running" and not controller._waiting_key
    finally:
        controller.shutdown()


@pytest.mark.parametrize("missing", ["scan", "dlio", "localizer"])
def test_planning_is_blocked_until_inputs_recover(missing):
    ready = dict(scan=True, dlio=True, localizer=True)
    action = SimpleNamespace(server_is_ready=lambda: True)
    node = SimpleNamespace(_navigation_readiness=SimpleNamespace(ready=lambda *args: True, reason=""),
        navigation_client=action, robot_client=action,
        _planning_inputs=tuple((key, key+" unavailable") for key in ready),
        navigation_component_ready=ready.__getitem__)
    ready[missing] = False
    assert not QtNavRosNode.navigation_route_ready(node)
    assert node._navigation_readiness.reason == missing+" unavailable"
    ready[missing] = True
    assert QtNavRosNode.navigation_route_ready(node)

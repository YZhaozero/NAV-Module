"""Configurable Nav2 frontend, persistence, readiness and ownership."""
import importlib.util
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from nav_app.qt_nav_node import NavigationWindow, QtNavRosNode
from nav_app.map_processing import GridMap, save_grid_map
from nav_app.features.navigation_stack import NavigationStackConfig, NavigationStackController, build_navigation_launches, validate_navigation_stack
from nav_app.ui.navigation_access import NavigationAccessPage


@pytest.fixture
def window(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_ACCESS_SETTINGS", str(tmp_path / "access.ini"))
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path / "tasks"))
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    node = SimpleNamespace(robot_profile="woosh", navigation_status="等待准备",
        map_storage_dir=str(tmp_path / "maps"), mapping_backend="woosh_fast_livo")
    view = NavigationWindow(node)
    view.refresh_timer.stop();view.operations_page.timer.stop()
    yield view
    view.close();application.processEvents()
    view.operations_page.task_coordinator.thread.join(timeout=5)


@pytest.mark.parametrize("preset,owned", [
    ("woosh", {"dlio", "localizer", "nav2"}),
    ("woosh_native5", {"lidar", "dlio", "laserscan", "localizer", "nav2"}),
    ("external", {"nav2"}),
    ("local", {"lidar", "camera", "dlio", "laserscan", "localizer", "nav2"}),
])
def test_saved_preset_controls_owned_launches_and_keeps_nav2(window, tmp_path, preset, owned):
    page = window.navigation_access_page
    page.load_preset(preset)
    page.fields["navigation_engine"].setCurrentIndex(page.fields["navigation_engine"].findData("nav2"))
    page.save()
    grid, _ = save_grid_map(tmp_path / "room.yaml", GridMap(np.zeros((10,10),dtype=np.int8), .05, np.zeros(3)))
    cloud = tmp_path / "room.pcd";cloud.write_text("PCD")
    stack = window.navigation_stack_page
    stack.navigation_map.setText(str(grid));stack.localization_map.setText(str(cloud));stack.scan_context_map.clear()
    config = stack.configuration()
    assert validate_navigation_stack(config)[0]
    launches = build_navigation_launches(config)
    assert set(launches) == owned
    assert ("tracking_confirm_translation_m:=0.10" in launches.get("localizer", [])) == (preset == "woosh_native5")
    assert config.navigation_backend == "nav2"
    assert not any("enable_base_control:=true" in args for args in launches.values())
    if preset == "woosh_native5":
        assert config.lidar_driver_mode == "nav"
        assert launches["lidar"] == ["systemctl", "--user", "start", "nav-livox5.service"]
        assert config.woosh_fast_livo_profile == "native5"
        assert launches["dlio"][:3] == ["launch", "nav_app", "woosh_native5.launch.py"]
        for argument in ["pointcloud_topic:=/livox/lidar_192_168_128_5",
                         "imu_topic:=/livox/imu_192_168_128_5",
                         "image_topic:=/foo/camera_1/color/image_raw", "imu_rate_odom:=true", "start_driver:=false"]:
            assert argument in launches["dlio"]
        assert config.fast_livo_vision and config.localization_backend == "pointcloud_localizer"
        assert "odom_topic:=/nav/woosh/odom" in launches["nav2"]
        assert "scan_topic:=/scan" in launches["nav2"]
        assert config.scan_source == "convert" and config.laser_target_frame == "base_footprint"
        assert config.min_height == .05 and config.max_height == 1.5
        for argument in ["cloud_topic:=/nav/livox5/points_filtered", "scan_topic:=/scan",
                         "target_frame:=base_footprint", "min_height:=0.05", "max_height:=1.5", "scan_time:=0.1"]:
            assert argument in launches["laserscan"]
    if preset == "external":
        stack.localization_map.clear()
        assert validate_navigation_stack(stack.configuration())[0]
        assert "odom_topic:=/foo/odom" in launches["nav2"]
        assert "scan_topic:=/foo/scan" in launches["nav2"]
        assert "external_localization:=false" in launches["nav2"]


def test_draft_does_not_change_runtime_and_saved_values_reload(window):
    page=window.navigation_access_page
    before=window.navigation_stack_page.configuration()
    page.load_preset("external")
    assert window.navigation_stack_page.configuration().odometry_backend==before.odometry_backend
    page.save()
    loaded=NavigationAccessPage(window.navigation_stack_page,lambda:None,"woosh")
    assert loaded.values()==page.values()
    assert window.navigation_stack_page.configuration().odometry_backend=="external"
    loaded.close()


def test_runtime_blocks_editing_settings(window):
    page=window.navigation_access_page
    window.navigation_stack_controller.overall_state_changed.emit("running")
    assert not page.save_button.isEnabled() and not page.scroll.isEnabled()
    window.navigation_stack_controller.overall_state_changed.emit("stopped")
    assert page.save_button.isEnabled()


def test_external_scan_can_be_combined_with_local_odometry(window):
    page=window.navigation_access_page
    page.load_preset("woosh");page.save()
    config=window.navigation_stack_page.configuration()
    config.navigation_map="map.yaml";config.localization_map="map.pcd"
    launches=build_navigation_launches(config)
    assert "dlio" in launches and "laserscan" not in launches
    assert config.scan_topic=="/foo/scan"
    assert "image_topic:=/foo/camera_1/color/image_raw" in launches["dlio"]


class Process(QObject):
    log_received=pyqtSignal(str)
    state_changed=pyqtSignal(str)
    def __init__(self,label,parent=None):
        super().__init__(parent);self.running=False;self.calls=[];self.stops=0
    @property
    def active(self):return self.running
    def start_launch(self,args):
        self.calls.append(args);self.running=True
        QTimer.singleShot(0,lambda:self.state_changed.emit("running"))
        return True
    def stop_launch(self):
        self.stops+=1;self.running=False;self.state_changed.emit("stopped")
    def shutdown(self):self.running=False


class Sensors(QObject):
    log_received=pyqtSignal(str,str)
    state_changed=pyqtSignal(str,str)
    def __init__(self):
        super().__init__();self.processes={k:Process(k) for k in ("lidar","camera")}
    def start(self,key,args):raise AssertionError("External driver must not start")
    def stop(self,key):self.processes[key].stop_launch()


def wait_for(app,condition):
    end=time.monotonic()+3
    while not condition() and time.monotonic()<end:
        app.processEvents();time.sleep(.01)
    assert condition()


def test_external_inputs_gate_nav2_and_stop_does_not_touch_shared_drivers(tmp_path, monkeypatch, application):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module,"RosLaunchProcess",Process)
    sensors=Sensors()
    for process in sensors.processes.values():process.running=True
    ready={"scan":False,"external_odom":False,"external_localization":False,"nav2":True}
    controller=NavigationStackController(sensors,sensor_active=lambda key:False,component_ready=ready.__getitem__)
    grid=tmp_path/"room.yaml";grid.write_text("image: room.pgm")
    config=NavigationStackConfig(navigation_map=str(grid),localization_map="",robot_profile="woosh",
        odometry_backend="external",localization_backend="external",lidar_driver_mode="external",
        camera_driver_mode="off",scan_source="external",external_odom_topic="/foo/odom",scan_topic="/foo/scan",startup_interval_ms=0)
    try:
        assert controller.start(config)[0]
        assert not controller.processes["nav2"].calls
        assert any("/foo/scan" in text for key,text in controller.log_events)
        ready.update(scan=True,external_odom=True,external_localization=True)
        wait_for(application,lambda:controller.overall_state=="running")
        assert controller.processes["nav2"].calls
        controller._monitor_sensor_health()
        assert controller.overall_state=="running"
        ready["external_odom"]=False
        controller._monitor_sensor_health()
        wait_for(application,lambda:controller.overall_state=="error")
        assert all(p.running and p.stops==0 for p in sensors.processes.values())
        assert controller.processes["nav2"].stops==1
    finally:controller.shutdown()


@pytest.mark.parametrize("scan_topic,external", [("/foo/scan", "false"), ("/scan", "true")])
def test_woosh_amcl_lifecycle_and_scan_parameters_are_connected(tmp_path, monkeypatch,scan_topic,external):
    from launch import LaunchContext
    from ament_index_python.packages import get_package_share_directory
    path=Path(__file__).parents[1]/"launch/woosh_nav2.launch.py"
    spec=importlib.util.spec_from_file_location("access_woosh_nav2",path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nav2=get_package_share_directory("nav2_bringup")
    monkeypatch.setattr(module,"get_package_share_directory",lambda name:str(Path(__file__).parents[1]) if name=="nav_app" else get_package_share_directory(name))
    nodes=[]
    def node(**kwargs):nodes.append(kwargs);return kwargs
    monkeypatch.setattr(module,"Node",node)
    context=LaunchContext()
    context.launch_configurations.update(use_sim_time="false",autostart="true",map="room.yaml",rviz="false",
        external_localization=external,enable_base_control="false",required_control_mode="0",odom_topic="/foo/odom",scan_topic=scan_topic,
        nav2_params_file=str(Path(nav2)/"params/nav2_params.yaml"))
    module.start(context)
    localization=next(n for n in nodes if n.get("name")=="lifecycle_manager_map")
    assert localization["parameters"][0]["node_names"] == (["map_server"] if external=="true" else ["map_server","amcl"])
    navigation=next(n for n in nodes if n.get("name")=="lifecycle_manager_navigation")
    assert "amcl" not in navigation["parameters"][0]["node_names"]
    assert any(n.get("name")=="amcl" for n in nodes) == (external=="false")
    controller=next(n for n in nodes if n.get("name")=="controller_server")
    configured=Path(controller["parameters"][0]);params=yaml.safe_load(configured.read_text())
    try:
        assert params["amcl"]["ros__parameters"]["scan_topic"]==scan_topic
        for area,layer in [("local","obstacle_layer"),("local","voxel_layer"),("global","obstacle_layer")]:
            assert params[area+"_costmap"][area+"_costmap"]["ros__parameters"][layer]["scan"]["topic"] == scan_topic
        assert params["amcl"]["ros__parameters"]["base_frame_id"]=="base_link"
        assert params["controller_server"]["ros__parameters"]["odom_topic"]=="/foo/odom"
        assert not any(n.get("executable")=="woosh_base_adapter" for n in nodes)
    finally:configured.unlink()


def test_amcl_initial_pose_is_not_a_motion_goal():
    from builtin_interfaces.msg import Time
    sent=[]
    node=SimpleNamespace(amcl_initial_pose_ready=lambda:True,
        amcl_initial_pose_pub=SimpleNamespace(publish=sent.append),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10))))
    accepted,message=QtNavRosNode.publish_amcl_initial_pose(node,[1.,2.],math.pi/2,"map")
    assert accepted and len(sent)==1
    assert sent[0].header.frame_id=="map"
    assert sent[0].pose.pose.position.x==1. and sent[0].pose.pose.position.y==2.
    assert sent[0].pose.pose.orientation.z==pytest.approx(math.sqrt(.5))


def test_optional_preview_camera_failure_does_not_block_external_navigation(tmp_path, monkeypatch, application):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module,"RosLaunchProcess",Process)
    sensors=Sensors()
    calls=[]
    sensors.start=lambda key,args:calls.append(key) or False
    controller=NavigationStackController(sensors,sensor_active=lambda key:False,component_ready=lambda key:True)
    grid=tmp_path/"room.yaml";grid.write_text("image: room.pgm")
    config=NavigationStackConfig(navigation_map=str(grid),localization_map="",odometry_backend="external",
        localization_backend="external",lidar_driver_mode="external",camera_driver_mode="nav",
        camera_arguments=["launch","realsense2_camera","d435i.launch.py"],scan_source="external",startup_interval_ms=0)
    try:
        assert controller.start(config)[0]
        wait_for(application,lambda:controller.overall_state=="running")
        assert calls==["camera"]
        assert controller.states["camera"]=="error"
        controller._monitor_sensor_health()
        assert controller.overall_state=="running"
    finally:controller.shutdown()


def test_external_odometry_requires_fresh_messages_and_tf():
    node=SimpleNamespace(_odometry_received={"external":time.monotonic()-5},base_frame="base_link",map_frame="map",
        _navigation_tf_ready=lambda target,source:True)
    assert not QtNavRosNode.navigation_component_ready(node,"external_odom")
    node._odometry_received["external"]=time.monotonic()
    assert QtNavRosNode.navigation_component_ready(node,"external_odom")
    node._navigation_tf_ready=lambda target,source:False
    assert not QtNavRosNode.navigation_component_ready(node,"external_odom")
    assert not QtNavRosNode.navigation_component_ready(node,"external_localization")


def test_missing_external_input_timeout_marks_unstarted_nav2_error(tmp_path, monkeypatch, application):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module,"RosLaunchProcess",Process)
    sensors=Sensors()
    for process in sensors.processes.values():process.running=True
    controller=NavigationStackController(sensors,component_ready=lambda key:False)
    grid=tmp_path/"room.yaml";grid.write_text("image: room.pgm")
    config=NavigationStackConfig(navigation_map=str(grid),localization_map="",odometry_backend="external",
        localization_backend="external",lidar_driver_mode="external",camera_driver_mode="off",scan_source="external")
    try:
        assert controller.start(config)[0]
        controller._poll_external_inputs(time.monotonic()-1)
        assert controller.overall_state=="error"
        assert controller.states["nav2"]=="error"
        assert not controller.processes["nav2"].calls
        assert all(process.running and process.stops==0 for process in sensors.processes.values())
    finally:controller.shutdown()


def test_native5_draft_persistence_and_mount_switch_clear_only_old_livo_override(window):
    page=window.navigation_access_page;stack=window.navigation_stack_page
    stack.fast_livo_params.setText("previous-livox3.yaml")
    page.load_preset("woosh_native5")
    assert stack.configuration().woosh_fast_livo_profile == "livox3"
    assert stack.fast_livo_params.text() == "previous-livox3.yaml"
    page.save()
    assert stack.configuration().woosh_fast_livo_profile == "native5"
    assert not stack.fast_livo_params.text()
    stack.fast_livo_params.setText("custom-livox5.yaml")
    stack.min_height.setValue(.1)
    page.save()
    assert stack.min_height.value() == .1
    assert stack.fast_livo_params.text() == "custom-livox5.yaml"
    loaded=NavigationAccessPage(stack,lambda:None,"woosh")
    assert loaded.values() == page.values()
    loaded.close()
    page.load_preset("woosh");page.save()
    assert stack.configuration().woosh_fast_livo_profile == "livox3"
    assert stack.dlio_imu_topic.text() == "/foo/livox/imu_192_168_128_3"
    assert not stack.fast_livo_params.text()


def test_legacy_saved_woosh_settings_keep_livox3_mount(window):
    page=window.navigation_access_page
    legacy=page.preset("woosh");legacy.pop("woosh_fast_livo_profile")
    page.settings.setValue("profiles/woosh",legacy);page.settings.sync()
    loaded=NavigationAccessPage(window.navigation_stack_page,lambda:None,"woosh")
    assert loaded.values()["woosh_fast_livo_profile"] == "livox3"
    loaded.close()


def test_native5_cannot_silently_reuse_dlio_mount(window,tmp_path):
    page=window.navigation_access_page;page.load_preset("woosh_native5");page.save()
    config=window.navigation_stack_page.configuration()
    config.navigation_map=str(tmp_path/"room.yaml");config.localization_map=str(tmp_path/"room.pcd")
    config.odometry_backend="dlio"
    valid,message=validate_navigation_stack(config)
    assert not valid and "DLIO" in message


def test_livox3_keeps_external_scan_when_native5_uses_its_own_scan(window):
    page=window.navigation_access_page
    page.load_preset("woosh_native5");page.save()
    assert window.navigation_stack_page.configuration().scan_topic == "/scan"
    assert window.navigation_stack_page.configuration().laser_cloud_topic == "/nav/livox5/points_filtered"
    page.load_preset("woosh");page.save()
    config=window.navigation_stack_page.configuration()
    assert config.scan_source == "external" and config.scan_topic == "/foo/scan"


@pytest.mark.parametrize("outcome", ["data", "already_online", "service_error", "missing_systemctl", "stop", "timeout", "data_timeout"])
def test_native5_service_precedes_algorithms_and_failures_stop_startup(tmp_path, monkeypatch, application, outcome):
    from nav_app.features import navigation_stack as module
    monkeypatch.setattr(module, "RosLaunchProcess", Process)
    commands = tmp_path / "bin";commands.mkdir()
    executable = commands / "systemctl"
    record = tmp_path / "service-arguments"
    executable.write_text(
        "#!/usr/bin/python3\nimport sys, time\nfrom pathlib import Path\n"
        f"Path({str(record)!r}).write_text(' '.join(sys.argv[1:]))\n"
        + ("if 'start' in sys.argv: time.sleep(10)\n" if outcome in ("stop", "timeout") else "")
        + ("sys.exit(7)\n" if outcome == "service_error" else ""))
    executable.chmod(0o755)
    if outcome == "missing_systemctl":executable.unlink()
    monkeypatch.setenv("PATH", str(commands))
    live = {"lidar": outcome == "already_online", "camera": True}
    controller = NavigationStackController(Sensors(), sensor_active=live.__getitem__, component_ready=lambda key: True)
    grid = tmp_path / "map.yaml";grid.write_text("image: map.pgm")
    cloud = tmp_path / "map.pcd";cloud.write_text("PCD")
    config = NavigationStackConfig(robot_profile="woosh", woosh_fast_livo_profile="native5",
        lidar_driver_mode="nav", camera_driver_mode="external", scan_source="convert",
        odometry_backend="fast_livo", fast_livo_lidar_topic="/livox/lidar_192_168_128_5",
        dlio_imu_topic="/livox/imu_192_168_128_5", navigation_map=str(grid),
        localization_map=str(cloud), startup_interval_ms=0)
    service_deadline = []
    later = controller._later
    def schedule(delay, callback):
        if delay == 15000:service_deadline.append(callback)
        else:later(delay, callback)
    monkeypatch.setattr(controller, "_later", schedule)
    try:
        assert validate_navigation_stack(config)[0]  # Service does not need generic Livox launch arguments.
        assert controller.start(config)[0]
        if outcome == "already_online":
            wait_for(application, lambda:controller.overall_state == "running")
            assert record.read_text() == "--user start nav-livox5.service"
        elif outcome in ("service_error", "missing_systemctl"):
            wait_for(application, lambda:controller.overall_state == "error")
            assert not any(p.calls for p in controller.processes.values())
            assert any("服务" in text and ("失败" in text or "无法启动" in text) for _,text in controller.log_events)
        elif outcome in ("stop", "timeout"):
            wait_for(application, record.exists)
            finished = []
            controller._lidar_service_start.finished.connect(lambda *_:finished.append(True))
            if outcome == "stop":controller.stop()
            else:
                # Invoke only the service deadline callback, without waiting 15 seconds.
                service_deadline[0]()
            wait_for(application, lambda:bool(finished))
            application.processEvents()
            wait_for(application, lambda: controller.overall_state == ("stopped" if outcome == "stop" else "error"))
            assert not any(p.calls for p in controller.processes.values())
        else:
            wait_for(application, lambda:controller.states["lidar"] == "waiting")
            assert record.read_text() == "--user start nav-livox5.service"
            assert not any(p.calls for p in controller.processes.values())
            if outcome == "data_timeout":
                controller._sensor_wait_deadline = 0
                controller._poll_sensor_data("lidar")
                wait_for(application, lambda: controller.overall_state == "error")
                assert not any(p.calls for p in controller.processes.values())
            else:
                live["lidar"] = True
                wait_for(application, lambda:controller.overall_state == "running")
                assert all(p.calls for p in controller.processes.values())
        controller.stop()
        if outcome in ("already_online", "data"):
            wait_for(application, lambda: controller.overall_state == "stopped")
            assert record.read_text() == "--user stop nav-livox5.service"
        assert not any(p.stops for p in controller.sensor_controller.processes.values())
    finally:
        controller.shutdown()
        application.processEvents()


@pytest.mark.parametrize("source,simulated,mode", [("bag", True, "nav"), ("live", True, "nav"), ("live", False, "external")])
def test_native5_replay_and_external_mode_do_not_acquire_hardware(source, simulated, mode):
    config = NavigationStackConfig(navigation_map="", localization_map="", robot_profile="woosh", woosh_fast_livo_profile="native5",
        lidar_driver_mode=mode, camera_driver_mode="external", odometry_backend="fast_livo",
        sensor_source=source, use_sim_time=simulated)
    assert "lidar" not in build_navigation_launches(config)


def test_woosh_defaults_to_meshnav_and_launches_without_scan(window, tmp_path, monkeypatch):
    from ament_index_python import packages

    real_share = packages.get_package_share_directory
    monkeypatch.setattr(packages, "get_package_prefix", lambda name: str(tmp_path))
    page = window.navigation_access_page
    page.load_preset("woosh_native5")
    stack = window.navigation_stack_page
    cloud = tmp_path / "office.pcd"
    cloud.write_text("PCD")
    mesh = cloud.with_suffix(".mesh.ply")
    mesh.write_text("ply")
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: str(tmp_path) if name == "nav_app" else real_share(name))
    stack.localization_map.setText(str(cloud))
    stack.scan_context_map.clear()
    page.save()
    config = stack.configuration()

    assert page.settings.value("profiles/woosh")["navigation_engine"] == "meshnav"
    assert config.navigation_backend == "meshnav" and config.woosh_control_mode == 0
    assert validate_navigation_stack(config)[0]
    launches = build_navigation_launches(config)
    assert "laserscan" not in launches
    assert launches["nav2"][:3] == ["run", "mbf_mesh_nav", "mbf_mesh_nav"]
    assert f"mesh_map.mesh_file:={mesh}" in launches["nav2"]
    stack.set_operating_mode("robot", 1)
    auto = stack.configuration()
    assert auto.navigation_backend == "meshnav" and validate_navigation_stack(auto)[0]
    auto_launch = build_navigation_launches(auto)
    assert "laserscan" not in auto_launch
    assert auto.mesh_cloud_topic == "/nav/livox5/points_filtered"
    assert auto_launch["nav2"][:3] == ["launch", "nav_app", "woosh_mesh_drive.launch.py"]
    assert f"mesh:={mesh}" in auto_launch["nav2"] and "control_mode:=1" in auto_launch["nav2"]

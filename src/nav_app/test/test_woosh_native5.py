"""Native .5 mount, sensor pairing, driver ownership and desktop defaults."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from launch import LaunchContext
from scipy.spatial.transform import Rotation

from nav_app.features.mapping import MAPPING_BACKENDS, MappingController
from nav_app.qt_nav_node import NavigationWindow

ROOT = Path(__file__).resolve().parents[3]


def test_native5_extrinsics_and_units_preserve_base_and_camera_geometry():
    config = ROOT / "src/nav_app/config"
    params = yaml.safe_load((config / "woosh_native5.yaml").read_text())["/**"]["ros__parameters"]
    old = yaml.safe_load((config / "woosh_fast_livo.yaml").read_text())["/**"]["ros__parameters"]
    ext = params["extrin_calib"]
    rotation = np.array(ext["extrinsic_R"]).reshape(3, 3)
    translation = np.array(ext["extrinsic_T"])
    mount = Rotation.from_euler("y", 30, degrees=True).as_matrix()
    sensor_point = np.array([1.2, -.4, 2.5])
    base_point = mount @ sensor_point + [.429, 0., .605]
    np.testing.assert_allclose(rotation @ base_point + translation, sensor_point, atol=1e-12)
    # A level base produces the measured raw sensor gravity direction.
    np.testing.assert_allclose(rotation @ [0., 0., 1.], [-.5, 0., np.sqrt(3)/2], atol=1e-12)
    assert ext["Rcl"] == old["extrin_calib"]["Rcl"]
    assert ext["Pcl"] == old["extrin_calib"]["Pcl"]
    assert params["common"]["imu_acc_scale"] == 9.80665
    assert params["common"]["imu_frame"] == "livox_frame"
    assert params["common"]["lid_topic"] == "/livox/lidar_192_168_128_5"
    assert params["common"]["imu_topic"] == "/livox/imu_192_168_128_5"
    assert params["common"]["img_topic"] == "/foo/camera_1/color/image_raw"
    assert params["preprocess"]["lidar_type"] == 8
    assert params["navigation"]["input_holdback_s"] == .4


@pytest.mark.parametrize("simulated,start_driver,expected_calls", [("false", "true", 1), ("true", "true", 0), ("false", "false", 0)])
def test_native5_driver_service_is_reused_and_replay_does_not_start_hardware(monkeypatch, simulated, start_driver, expected_calls):
    spec = importlib.util.spec_from_file_location("native5_launch", ROOT / "src/nav_app/launch/woosh_native5.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(module.subprocess, "run", lambda *args, **kwargs: calls.append((args, kwargs)))
    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = simulated
    context.launch_configurations["start_driver"] = start_driver
    module._ensure_native_driver(context)
    assert len(calls) == expected_calls
    if expected_calls:
        assert calls[0] == ((["systemctl", "--user", "start", "nav-livox5.service", "nav-camera1.service"],),
                            {"check": True, "timeout": 15})


def test_native5_page_and_command_use_paired_native_topics(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path / "tasks"))
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    node = SimpleNamespace(navigation_status="ready", map_storage_dir=str(tmp_path), latest_pointcloud=None,
        send_navigation_waypoints=lambda _: True, cancel_navigation=lambda: None, mapping_backend="woosh_native5")
    window = NavigationWindow(node)
    window.refresh_timer.stop()
    window.operations_page.timer.stop()
    try:
        page = window.mapping_page
        backend = page.controller.backend
        assert backend.key == "woosh_native5"
        assert page.pointcloud_combo.currentText() == "/livox/lidar_192_168_128_5"
        assert page.imu_combo.currentText() == "/livox/imu_192_168_128_5"
        assert page.image_topic.text() == "/foo/camera_1/color/image_raw"
        assert page.vision_enabled.isChecked()
        command = page.launch_arguments()
        assert command[:3] == ["launch", "nav_app", "woosh_native5.launch.py"]
        assert "img_en:=1" in command
        assert "pointcloud_topic:=/livox/lidar_192_168_128_5" in command
        assert "imu_topic:=/livox/imu_192_168_128_5" in command
        assert backend.save_service == "/fast_livo/save_map"
        page.algorithm_combo.setCurrentIndex(page.algorithm_combo.findData("woosh_fast_livo"))
        assert page.pointcloud_combo.currentText() == "/foo/livox/lidar_192_168_128_3"
        assert page.image_topic.text() == "/foo/camera_1/color/image_raw"
        assert MAPPING_BACKENDS["woosh_dlio"].save_service == "/save_pcd"
    finally:
        window.operations_page.shutdown()
        window.operations_page.task_coordinator.thread.join(timeout=5)
        window.close()
        application.processEvents()


def test_native5_navigation_uses_native_body_frame_and_keeps_mapping_default(monkeypatch):
    from launch.actions import DeclareLaunchArgument, LogInfo
    from launch.utilities import perform_substitutions
    spec=importlib.util.spec_from_file_location("native5_navigation_launch",ROOT/"src/nav_app/launch/woosh_native5.launch.py")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=[];includes=[]
    monkeypatch.setattr(module,"get_package_share_directory",lambda name:str(ROOT/"src"/name))
    monkeypatch.setattr(module,"Node",lambda **kwargs:nodes.append(kwargs) or LogInfo(msg="node"))
    monkeypatch.setattr(module,"IncludeLaunchDescription",lambda source,**kwargs:includes.append(dict(kwargs["launch_arguments"])) or LogInfo(msg="include"))
    description=module.generate_launch_description()
    context=LaunchContext();context.launch_configurations.update(use_sim_time="false",imu_rate_odom="true")
    adapter=next(node for node in nodes if node.get("executable")=="woosh_odometry")
    assert adapter["condition"].evaluate(context)
    assert adapter["parameters"][0]["imu_frame"] == "woosh_livox5_imu"
    estimator=next(include for include in includes if "body_frame" in include)
    assert estimator["body_frame"] == "woosh_livox5_imu"
    assert estimator["pointcloud_topic"] == "/nav/livox5/points_filtered"
    assert estimator["imu_rate_odom"].perform(context) == "true"
    default=next(action for action in description.entities if isinstance(action,DeclareLaunchArgument) and action.name=="imu_rate_odom")
    assert perform_substitutions(context,default.default_value) == "false"

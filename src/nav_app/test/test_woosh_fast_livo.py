"""The WOOSH cloud is already in base, while the filter state is the raw IMU frame."""
from pathlib import Path
from types import SimpleNamespace
import math
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from nav_app.features.mapping import MAPPING_BACKENDS, MappingController
from nav_app.qt_nav_node import NavigationWindow

ROOT = Path(__file__).resolve().parents[3]


def test_woosh_fast_livo_extrinsics_preserve_physical_geometry():
    params = yaml.safe_load((ROOT / "src/nav_app/config/woosh_fast_livo.yaml").read_text())["/**"]["ros__parameters"]
    ext = params["extrin_calib"]
    rotation = np.array(ext["extrinsic_R"]).reshape(3, 3)
    offset = np.array(ext["extrinsic_T"])
    r_bi = Rotation.from_euler("z", 45, degrees=True).as_matrix()
    t_bi = np.array([.338, .190, .330])
    sensor_point = np.array([1.2, -.4, 2.5])
    base_point = r_bi @ sensor_point + t_bi
    np.testing.assert_allclose(rotation @ base_point + offset, sensor_point, atol=1e-12)
    r_bc = Rotation.from_quat([.011386218188704486, -.005142377179311725,
                              -.012441454847957095, .9998445480141257]).as_matrix()
    r_co = Rotation.from_quat([.4999999999999998, .49999998660255174,
                              .4999999999999998, .5000000133974483]).as_matrix()
    r_bo = r_bc @ r_co
    t_bo = np.array([.398, 0, .215]) + r_bc @ np.array([0, .007, 0])
    camera_point = np.array([.1, .2, 3.])
    np.testing.assert_allclose(np.array(ext["Rcl"]).reshape(3, 3) @ (r_bo @ camera_point + t_bo)
                              + ext["Pcl"], camera_point, atol=1e-12)
    assert params["time_offset"]["img_time_offset"] == 0.
    assert params["preprocess"]["lidar_type"] == 8
    assert params["common"]["imu_topic"] == "/foo/livox/imu_192_168_128_3"
    assert params["common"]["imu_acc_scale"] == 9.80665
    assert params["common"]["imu_frame"] == "livox_frame"


def test_woosh_fast_livo_uses_native_cloud_and_paired_sensors():
    backend = MAPPING_BACKENDS["woosh_fast_livo"]
    assert backend.pointcloud_type == "sensor_msgs/msg/PointCloud2"
    controller = SimpleNamespace(backend=backend)
    arguments = MappingController.build_launch_arguments(controller,
        backend.default_pointcloud_topic, backend.default_imu_topic, False, False,
        "/foo/camera_1/color/image_raw", vision_enabled=True)
    assert arguments[:3] == ["launch", "nav_app", "woosh_fast_livo.launch.py"]
    assert "img_en:=1" in arguments
    assert "pointcloud_topic:=/foo/livox/lidar_192_168_128_3" in arguments
    assert backend.save_service == "/fast_livo/save_map"
    assert MAPPING_BACKENDS["woosh_dlio"].save_service == "/save_pcd"


def test_woosh_fast_livo_page_seeds_camera_and_keeps_dlio(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path / "tasks"))
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    node = SimpleNamespace(navigation_status="ready", map_storage_dir=str(tmp_path), latest_pointcloud=None,
        send_navigation_waypoints=lambda _: True, cancel_navigation=lambda: None, mapping_backend="woosh_fast_livo")
    window = NavigationWindow(node)
    window.refresh_timer.stop(); window.operations_page.timer.stop()
    try:
        page = window.mapping_page
        assert page.controller.backend.key == "woosh_fast_livo"
        assert page.vision_enabled.isChecked() and page.vision_enabled.isEnabled()
        assert page.image_topic.text() == "/foo/camera_1/color/image_raw"
        assert page.pointcloud_combo.currentText() == "/foo/livox/lidar_192_168_128_3"
        page.algorithm_combo.setCurrentIndex(page.algorithm_combo.findData("woosh_dlio"))
        assert page.controller.backend.key == "woosh_dlio"
        assert not page.vision_enabled.isEnabled()
    finally:
        window.operations_page.shutdown(); window.operations_page.task_coordinator.thread.join(timeout=5)
        window.close(); application.processEvents()

"""RTK trajectory map UI with synthetic positions and map assets."""

import time
from types import SimpleNamespace as NS

import numpy as np
from PyQt5.QtWidgets import QApplication

from nav_app.features.trajectory_test import TrajectoryTest
from nav_app.map_processing import GridMap, save_grid_map, save_pcd
from nav_app.ui import web_map
from nav_app.ui.trajectory_page import TrajectoryPage


def wait(app, condition):
    deadline = time.monotonic() + 5
    while not condition() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(.005)
    assert condition()


def fix():
    return dict(
        stamp=1000., received_at=1000.1,
        coordinate_system="WGS84", timestamp_source="receiver_utc",
        valid_position=True, fix_quality=4,
        longitude=116.4, latitude=39.9,
    )


def test_web_document_uses_nav_objects():
    page = web_map.document()
    assert "window.NAVBaidu" in page
    assert "window.NAVTrajectory" in page
    assert "window.GS" not in page


def test_page_loads_selected_grid_and_cloud(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    recorder = TrajectoryTest(tmp_path / "trajectory")
    node = NS(
        trajectory_test=recorder,
        rtk_live=NS(observation=None, observation_received_at=None),
        outdoor=NS(sensor_replay=False, calibration={}),
        map_services=NS(browser_key=""),
        phone_remote_port=8765,
        _odometry_backend="fast_livo",
    )
    occupancy = np.zeros((4, 5), dtype=np.int8)
    grid_path, _ = save_grid_map(
        tmp_path / "map.yaml",
        GridMap(occupancy, .1, np.array([1., 2., 0.]), frame_id="map"))
    cloud_path = save_pcd(
        tmp_path / "map.pcd",
        np.array([[1., 2., 0.], [2., 3., 0.]], dtype=np.float32))
    scene = NS(
        title="测试地图", summary="地图包完整", manifest=tmp_path / "map.scene.json",
        frame="map", grid=str(grid_path), cloud=str(cloud_path))
    library = NS(scenes=[scene], refresh=lambda *, preserve_edits=False: None)
    window = NS(node=node, mapping_controller=NS(backend=NS(key="fast_livo")))
    operations = NS(window=window, library=library)
    monkeypatch.setattr(web_map.WebMap, "ensure", lambda *args: True)
    page = TrajectoryPage(operations)
    page.timer.stop()
    try:
        wait(app, lambda: not page.worker.busy)
        page.view.setCurrentIndex(1)
        page.context.setCurrentIndex(0)
        page.load_selected_map()
        wait(app, lambda: not page.worker.busy)
        assert np.array_equal(page.canvas.grid[0], occupancy)
        assert page.canvas.grid[2] == .1
        assert page.canvas.cloud is None
        assert "已加载地图：测试地图" in page.status.text()
        page.layer.setCurrentIndex(1)
        page.refresh(force=True)
        assert page.canvas.grid is None
        assert len(page.canvas.cloud) == 2
    finally:
        recorder.close()
        page.shutdown()

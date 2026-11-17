"""Preview and conversion must select the same edited points in the same frame."""
from types import SimpleNamespace

import numpy as np
import pytest
from PyQt5.QtCore import QPointF
from nav_app.qt_nav_node import MapPanel, NavigationWindow
from nav_app.map_processing import PointCloudMap, pointcloud_to_grid


@pytest.fixture
def window(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path / "tasks"))
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    node = SimpleNamespace(navigation_status="ready", map_storage_dir=str(tmp_path),
        latest_pointcloud=None, send_navigation_waypoints=lambda _points: True,
        cancel_navigation=lambda: None)
    view = NavigationWindow(node)
    view.refresh_timer.stop()
    view.operations_page.timer.stop()
    yield view
    view.operations_page.shutdown()
    view.operations_page.task_coordinator.thread.join(timeout=5)
    view.close()
    application.processEvents()
    assert not view.operations_page.task_coordinator.thread.is_alive()


def test_height_preview_is_nondestructive_and_conversion_uses_edits(window):
    window.convert_coordinates_combo.setCurrentIndex(window.convert_coordinates_combo.findData("original"))
    points = np.array([[0, 0, 0], [1, 0, .5], [2, 0, 1], [3, 0, 1.5], [4, 0, 2]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.z_min_spin.setValue(.5)
    window.z_max_spin.setValue(1.5)
    assert window.editor_map_panel.height_slice_mask().tolist() == [False, True, True, True, False]
    assert np.array_equal(window.local_cloud.points, points)
    # An edit removes one obstacle and adds another before conversion.
    edited = np.array([[0, 0, 0], [1, 0, .5], [3, 0, 1.5], [4, 0, 2], [1, 1, .8]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(edited.copy()))
    assert window.editor_map_panel.height_slice_mask().tolist() == [False, True, True, False, True]
    window.grid_resolution_spin.setValue(.2)
    expected = pointcloud_to_grid(edited, resolution=.2, z_min=.5, z_max=1.5, inflation_radius=0.0,
        free_points=edited[np.abs(edited[:, 2]) <= .06])
    window.convert_pointcloud()
    assert np.array_equal(window.local_grid.occupancy, expected.occupancy)
    assert np.array_equal(window.local_grid.origin, expected.origin)
    assert np.array_equal(window.local_cloud.points, edited)


def test_sloped_cloud_stays_in_original_coordinates(window):
    x, y = np.meshgrid(np.linspace(-3, 3, 16), np.linspace(-2, 2, 12))
    points = np.column_stack((x.ravel(), y.ravel(), .4*x.ravel())).astype(np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.z_min_spin.setValue(-.1)
    window.z_max_spin.setValue(.1)
    window.convert_coordinates_combo.setCurrentIndex(window.convert_coordinates_combo.findData("original"))
    transformed = window.editor_map_panel.transform_cloud_points(points)
    expected = (transformed[:, 2] >= -.1) & (transformed[:, 2] <= .1)
    assert not window.conversion_uses_display_coordinates()
    assert np.array_equal(transformed, points)
    assert np.array_equal(window.editor_map_panel.height_slice_mask(), expected)
    assert np.array_equal(window.local_cloud.points, points)
    window.height_preview_check.setChecked(False)
    assert window.editor_map_panel._height_slice is None
    assert np.array_equal(window.local_cloud.points, points)


def test_plane_selection_updates_limits_from_the_visible_surface(window):
    window.convert_coordinates_combo.setCurrentIndex(window.convert_coordinates_combo.findData("original"))
    points = np.array([[-2, -1, .5], [0, 1, 1.2], [2, -1, 2]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    panel = window.editor_map_panel
    x, y, _depth = panel._project_cloud_points(panel._cloud_points)
    height = panel._pick_cloud_height(QPointF(float(x[1]), float(y[1])))
    assert height == pytest.approx(1.2)
    window.pick_height_plane("upper")
    panel.height_selected.emit(height)
    assert window.z_max_spin.value() == pytest.approx(1.2)
    assert np.array_equal(window.local_cloud.points, points)


def test_dense_cloud_keeps_more_samples_and_uses_visible_point_size(application):
    panel = MapPanel(cloud_3d=True)
    points = np.random.default_rng(7).normal(size=(280_000, 3)).astype(np.float32)
    panel.set_pointcloud(points)
    assert len(panel._cloud_points) == len(points)
    panel.set_cloud_point_size(3)
    assert panel._cloud_point_size == 3
    assert np.unique(panel._cloud_colors, axis=0).shape[0] > 20
    panel.close()


def test_pcd_loading_defaults_to_pseudocolor_but_preserves_real_rgb(window, tmp_path, monkeypatch):
    from nav_app import qt_nav_node
    from nav_app.map_processing import save_pcd
    source = tmp_path / "intensity.pcd"
    source.write_text("VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n0 0 0 10\n1 0 1 20\n2 0 2 30\n")
    monkeypatch.setattr(qt_nav_node.QFileDialog, "getOpenFileName", lambda *args: (str(source), ""))
    window.load_pointcloud_dialog()
    assert window.cloud_color_combo.currentData() == "height"
    assert not window.height_preview_check.isChecked()
    assert window.editor_map_panel._height_slice is None
    assert window.editor_map_panel._cloud_source_color_source == "intensity"
    colors = window.editor_map_panel._cloud_colors
    assert np.any(colors[:, 0] != colors[:, 1])
    window.cloud_color_combo.setCurrentIndex(window.cloud_color_combo.findData("intensity"))
    colors = window.editor_map_panel._cloud_colors
    assert np.unique(colors, axis=0).shape[0] == 3
    assert np.any(colors[:, 0] != colors[:, 2])
    assert "强度伪彩色" in window.editor_map_panel.cloud_display_description()
    rgb = np.array([[240, 10, 20], [30, 220, 40], [20, 40, 230]], dtype=np.uint8)
    source = save_pcd(tmp_path / "rgb.pcd", window.local_cloud.points, rgb)
    window.load_pointcloud_dialog()
    assert window.cloud_color_combo.currentData() == "height"
    assert np.array_equal(window.editor_map_panel._cloud_source_colors, rgb)
    window.cloud_color_combo.setCurrentIndex(window.cloud_color_combo.findData("rgb"))
    assert np.array_equal(window.editor_map_panel._cloud_colors, rgb)


def test_grid_conversion_has_no_hidden_expansion_and_metric_radius_is_not_rounded_up(window):
    point = np.array([[0., 0., 1.]], dtype=np.float32)
    window.convert_coordinates_combo.setCurrentIndex(window.convert_coordinates_combo.findData("original"))
    window._set_local_cloud(PointCloudMap(np.vstack((point, [[.4, .4, 0.]]))), reset_original=True)
    window.grid_resolution_spin.setValue(.05)
    window.z_min_spin.setValue(.5); window.z_max_spin.setValue(1.5)
    assert window.grid_inflation_spin.value() == 0
    window.convert_pointcloud()
    assert np.count_nonzero(window.local_grid.occupancy == 100) == 1
    assert "每格 5 cm" in window.map_status.text()
    expanded = pointcloud_to_grid(point, resolution=.05, z_min=.5, z_max=1.5, inflation_radius=.06)
    assert np.count_nonzero(expanded.occupancy == 100) == 5
    assert "新输出每格 5 cm" in window.grid_settings_label.text()


def test_grid_ruler_tracks_screen_zoom_without_changing_real_resolution(application):
    panel = MapPanel()
    panel.resize(800, 600)
    panel.set_map(np.zeros((100, 100), dtype=np.int8), np.zeros(2), .05, 0.)
    first_meters, first_pixels, resolution = panel.grid_scale_info()
    panel.zoom_view(2.)
    second_meters, second_pixels, second_resolution = panel.grid_scale_info()
    assert second_pixels / second_meters == pytest.approx(2 * first_pixels / first_meters)
    assert resolution == second_resolution == .05
    assert panel._grid_shape == (100, 100)
    panel.close()


def test_cloud_render_cache_updates_for_view_slice_and_new_data(application, monkeypatch):
    panel = MapPanel(cloud_3d=True)
    panel.resize(800, 480)
    panel.set_display_mode("cloud")
    points = np.random.default_rng(8).normal(size=(130000, 3)).astype(np.float32)
    panel.set_pointcloud(points)
    assert panel._cloud_pixmap is None  # No unused top-down raster for a 3-D panel.
    projections = []
    project = panel._project_cloud_points
    monkeypatch.setattr(panel, "_project_cloud_points", lambda xyz: projections.append(len(xyz)) or project(xyz))
    panel.grab()
    count = projections.count(len(points))
    first = panel._cloud_render_image.cacheKey()
    panel.grab()
    assert projections.count(len(points)) == count
    assert panel._cloud_render_image.cacheKey() == first
    panel.zoom_view(1.2)
    panel.grab()
    assert panel._cloud_render_image.cacheKey() != first
    second = panel._cloud_render_image.cacheKey()
    panel.set_height_slice((-.2, .8), display_coordinates=False)
    panel.grab()
    assert panel._cloud_render_image.cacheKey() != second
    second = panel._cloud_render_image.cacheKey()
    panel.set_pointcloud(points + 1)
    panel.grab()
    assert panel._cloud_render_image.cacheKey() != second
    panel._view_drag_button = 1  # Qt.LeftButton
    panel.grab()
    assert any(10000 < n <= 20000 for n in projections)
    drag_key = panel._cloud_render_image.cacheKey()
    panel._view_drag_button = 0
    panel.grab()
    assert panel._cloud_render_image.cacheKey() != drag_key
    panel.close()


def test_finish_mapping_waits_for_save_and_process_exit(window, monkeypatch, tmp_path):
    from PyQt5.QtTest import QSignalSpy
    page = window.mapping_page
    page.navigation_requested.disconnect()
    generated = QSignalSpy(page.navigation_requested)
    controller = page.controller
    controller.process = SimpleNamespace(active=True, shutdown=lambda: None)
    ros = controller.ros
    ros.save_attempt_id = "attempt-1"
    ros.save_status = "saved"
    monkeypatch.setattr(controller, "save_map", lambda *args: (True, "saving"))
    stops = []
    monkeypatch.setattr(controller, "stop", lambda: stops.append(True))
    page._handle_state("running")
    page.finish_mapping()
    assert not page.finish_button.isEnabled()
    page.refresh()
    assert not stops and not generated
    source = str(tmp_path / "map.pcd")
    ros.last_save_result = dict(attempt_id="attempt-1", success=True, path=source)
    ros.save_revision += 1
    page.refresh()
    assert stops == [True] and not generated
    controller.process.active = False
    page._handle_state("stopped")
    page.refresh()
    page.refresh()
    assert list(generated) == [[source, "nav_map", "observed"]]


def test_finish_mapping_save_failure_keeps_mapper_running(window, monkeypatch):
    from PyQt5.QtTest import QSignalSpy
    page = window.mapping_page
    page.navigation_requested.disconnect()
    generated = QSignalSpy(page.navigation_requested)
    controller = page.controller
    controller.process = SimpleNamespace(active=True, shutdown=lambda: None)
    ros = controller.ros
    ros.save_attempt_id = "failed-attempt"
    monkeypatch.setattr(controller, "save_map", lambda *args: (True, "saving"))
    stops = []
    monkeypatch.setattr(controller, "stop", lambda: stops.append(True))
    page._handle_state("running")
    page.finish_mapping()
    ros.last_save_result = dict(attempt_id="failed-attempt", success=False, path=None)
    ros.save_status = "保存失败"
    ros.save_revision += 1
    page.refresh()
    assert controller.process.active and not stops and not generated
    assert page.finish_button.isEnabled()
    assert "保存失败" in page.navigation_status.text()


def test_home_mapping_action_starts_once_and_preserves_running_session(window, monkeypatch):
    calls = []
    monkeypatch.setattr(window.mapping_page, "activate", lambda: None)
    monkeypatch.setattr(window.mapping_page, "start_mapping", lambda: (calls.append(True) or True, "启动中"))
    window.mapping_controller.process = SimpleNamespace(active=False, shutdown=lambda: None)
    window.open_mapping_button.click()
    assert window.pages.currentWidget() is window.mapping_workspace
    assert calls == [True]
    window.mapping_controller.process.active = True
    window.open_mapping_button.click()
    assert calls == [True]


def test_mapping_workspace_and_configuration_share_one_session(window, monkeypatch, application):
    settings, workspace = window.mapping_page, window.mapping_workspace
    starts = []
    monkeypatch.setattr(settings, "activate", lambda: None)
    monkeypatch.setattr(settings, "start_mapping", lambda: starts.append(True))
    window.mapping_controller.process = SimpleNamespace(active=True, shutdown=lambda: None)
    window.resize(1280, 720)
    window.show_mapping()
    window.show()
    application.processEvents()
    assert settings.map_panel.isVisible()
    assert workspace.map_panel is settings.map_panel
    assert workspace.isAncestorOf(settings.start_button)
    assert workspace.isAncestorOf(settings.finish_button)
    assert not settings.isAncestorOf(settings.map_panel)
    workspace.settings_button.click()
    application.processEvents()
    assert window.pages.currentWidget() is settings
    assert not settings.map_panel.isVisible()
    assert settings.algorithm_combo.isVisible()
    settings.map_name.setText("shared-name")
    settings.header.back_button.click()
    application.processEvents()
    assert window.pages.currentWidget() is workspace
    assert settings.map_name.text() == "shared-name"
    assert settings.map_panel.isVisible()
    assert not starts


def test_mapping_preview_catches_up_after_configuration(window, monkeypatch, application):
    page = window.mapping_page
    ros = page.controller.ros
    window.resize(1280, 720)
    monkeypatch.setattr(page, "activate", lambda: None)
    window.show_mapping_settings()
    window.show()
    application.processEvents()
    revision = page.last_cloud_revision
    ros.cloud = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float32)
    ros.cloud_revision += 1
    page.refresh()
    assert page.last_cloud_revision == revision
    window.show_mapping()
    application.processEvents()
    page._last_cloud_refresh = 0.
    page.refresh()
    assert page.last_cloud_revision == ros.cloud_revision
    assert len(page.map_panel._cloud_points) == 2


@pytest.mark.parametrize("width,height", [(1280, 720), (800, 480), (480, 800)])
def test_mapping_workspace_keeps_primary_actions_visible(window, application, width, height):
    from PyQt5.QtCore import QPoint
    window.show_mapping()
    window.resize(width, height)
    window.show()
    application.processEvents()
    application.processEvents()
    assert (window.width(), window.height()) == (width, height)
    for button in (window.mapping_workspace.settings_button,
                   window.mapping_page.start_button, window.mapping_page.finish_button):
        assert button.isVisible()
        top_left = button.mapTo(window, QPoint(0, 0))
        bottom_right = button.mapTo(window, button.rect().bottomRight())
        assert window.rect().contains(top_left)
        assert window.rect().contains(bottom_right)
    assert window.mapping_page.map_panel.height() >= 120


def test_manual_ground_produces_three_states_and_pgm_round_trip(window, tmp_path):
    import cv2
    from nav_app.map_processing import load_grid_map, save_grid_map
    points = np.array([[0., 0., -.29], [.5, 0., -.29], [.5, 0., .5], [1., 0., .5]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.floor_z_spin.setValue(-.29)
    window.convert_pointcloud()
    grid = window.local_grid
    assert set(np.unique(grid.occupancy)) == {-1, 0, 100}
    index = np.floor((points[:, :2] - grid.origin[:2]) / grid.resolution).astype(int)
    assert grid.occupancy[index[0, 1], index[0, 0]] == 0
    assert grid.occupancy[index[1, 1], index[1, 0]] == 100
    yaml_path, image_path = save_grid_map(tmp_path / 'three-state.yaml', grid)
    assert set(np.unique(cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE))) == {0, 205, 254}
    assert np.array_equal(load_grid_map(yaml_path).occupancy, grid.occupancy)
    assert '白色地面' in window.map_status.text()
    assert '灰色未知' in window.map_status.text()
    assert np.array_equal(window.local_cloud.points, points)


def test_ground_pick_sets_manual_height_without_leveling(window):
    points = np.array([[0., 0., -.29], [1., 0., .4]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.height_preview_check.setChecked(True)
    window.height_only_check.setChecked(True)
    window.pick_height_plane('ground')
    assert not window.height_only_check.isChecked()
    window.editor_map_panel.height_selected.emit(-.29)
    assert window.floor_z_spin.value() == pytest.approx(-.29)
    assert np.array_equal(window.editor_map_panel.transform_cloud_points(points), points)
    assert window.editor_map_panel._cloud_alignment == 'original'
    assert not hasattr(window, 'cloud_alignment_combo')


def test_wrong_ground_height_reports_no_observations_instead_of_filling_free(window):
    window._set_local_cloud(PointCloudMap(np.array([[0., 0., .7]], dtype=np.float32)), reset_original=True)
    window.convert_pointcloud()
    assert '没有实测点' in window.map_status.text()
    assert window.local_grid is None


def test_overlapping_floor_and_obstacle_bands_are_reported(window):
    window._set_local_cloud(PointCloudMap(np.array([[0., 0., 0.], [1., 0., .7]], dtype=np.float32)), reset_original=True)
    window.z_min_spin.setValue(.05)
    window.convert_pointcloud()
    assert '地面层上界须低于障碍高度下界' in window.map_status.text()
    assert window.local_grid is None


def test_ground_uses_same_manual_axis_transform_as_obstacles(window):
    points = np.array([[0., -.3, 0.], [1., .7, 0.]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.cloud_axis_combo.setCurrentIndex(window.cloud_axis_combo.findData('XZY'))
    window.floor_z_spin.setValue(-.3)
    window.convert_pointcloud()
    expected = pointcloud_to_grid(points[:, [0, 2, 1]], free_points=points[:1, [0, 2, 1]],
                                  z_min=.2, z_max=1.5, inflation_radius=0.)
    assert np.array_equal(window.local_grid.occupancy, expected.occupancy)
    assert np.array_equal(window.local_cloud.points, points)

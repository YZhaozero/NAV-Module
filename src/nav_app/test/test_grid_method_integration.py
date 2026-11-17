"""Method selection reaches both map generation entry points; SC stays shared."""
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from PyQt5.QtTest import QSignalSpy
from nav_app.map_processing import GridMap, PointCloudMap, save_pcd
from nav_app.qt_nav_node import NavigationWindow
from nav_app.features import navigation_bundle, terrain_grid
from nav_app.features.task_coordinator import normalize_task_payload, job_id


@pytest.fixture
def window(tmp_path, monkeypatch, application):
    monkeypatch.setenv('NAV_TASK_STATE_DIR', str(tmp_path/'tasks'))
    monkeypatch.setenv('NAV_RUNTIME_DIR', str(tmp_path/'runtime'))
    node = SimpleNamespace(navigation_status='ready', map_storage_dir=str(tmp_path/'maps'),
        latest_pointcloud=None, send_navigation_waypoints=lambda _: True, cancel_navigation=lambda: None)
    view = NavigationWindow(node)
    view.refresh_timer.stop(); view.operations_page.timer.stop()
    yield view
    view.operations_page.shutdown()
    view.operations_page.task_coordinator.thread.join(timeout=6)
    view.close(); application.processEvents()
    assert not view.operations_page.task_coordinator.thread.is_alive()


def test_daily_workspace_and_settings_selectors_stay_in_sync(window):
    settings = window.mapping_page.grid_method_combo
    daily = window.mapping_workspace.grid_method_combo
    assert settings.currentData() == daily.currentData() == 'observed'
    daily.setCurrentIndex(daily.findData('terrain'))
    assert settings.currentData() == 'terrain'
    settings.setCurrentIndex(settings.findData('observed'))
    assert daily.currentData() == 'observed'
    assert '原有投影' in window.convert_button.text()
    assert '局部高程' in window.terrain_convert_button.text()


@pytest.mark.parametrize('method', ['observed', 'terrain'])
def test_finish_snapshots_method_before_async_save_completes(window, monkeypatch, tmp_path, method):
    page = window.mapping_page; controller = page.controller
    page.navigation_requested.disconnect(); emitted = QSignalSpy(page.navigation_requested)
    controller.process = SimpleNamespace(active=True, shutdown=lambda: None)
    ros = controller.ros; ros.save_attempt_id = 'attempt'; ros.save_status = 'saved'
    monkeypatch.setattr(controller, 'save_map', lambda *args: (True, 'saving'))
    monkeypatch.setattr(controller, 'stop', lambda: None)
    page.grid_method_combo.setCurrentIndex(page.grid_method_combo.findData(method))
    page._handle_state('running'); page.finish_mapping()
    page.grid_method_combo.setCurrentIndex(page.grid_method_combo.findData('observed' if method == 'terrain' else 'terrain'))
    source = str(tmp_path/'saved.pcd')
    ros.last_save_result = dict(attempt_id='attempt', success=True, path=source)
    ros.save_revision += 1; page.refresh()
    assert not emitted
    controller.process.active = False; page._handle_state('stopped'); page.refresh()
    assert list(emitted) == [[source, 'nav_map', method]]


def test_automatic_submission_includes_method_and_distinguishable_title(window, monkeypatch):
    submitted = []
    monkeypatch.setattr(window.operations_page, 'submit_map_task', lambda *args: submitted.append(args) or 'task-id')
    window.operations_page.prepare_mapping_navigation('/data/map.pcd', 'room', 'terrain')
    assert submitted[0][1]['grid_method'] == 'terrain'
    assert submitted[0][1]['title'] == 'room · 地形分析'
    assert window.mapping_page.navigation_job_id == 'task-id'


def test_method_changes_job_identity_and_unknown_method_is_rejected(tmp_path):
    source = tmp_path/'map.pcd'; source.write_bytes(b'point cloud')
    payload = dict(cloud=str(source), directory=str(tmp_path), title='same')
    legacy = normalize_task_payload('navigation', payload)
    terrain = normalize_task_payload('navigation', dict(payload, grid_method='terrain'))
    assert legacy['grid_method'] == 'observed'
    assert job_id('navigation', legacy) != job_id('navigation', terrain)
    with pytest.raises(ValueError, match='不支持'):
        normalize_task_payload('navigation', dict(payload, grid_method='invalid'))


def test_worker_forwards_grid_method_without_changing_sc_requirement(tmp_path, monkeypatch):
    from nav_app.task_worker import execute
    from nav_app.features import task_storage
    monkeypatch.setattr(task_storage, 'task_storage_reason', lambda *args: '')
    calls = []
    monkeypatch.setattr(navigation_bundle, 'build', lambda *args, **kwargs: calls.append(kwargs) or dict(scene='new.scene.json'))
    spec = dict(kind='navigation', payload=dict(cloud='map.pcd', directory=str(tmp_path), title='room', grid_method='terrain'),
                work_directory=str(tmp_path/'work'))
    execute(spec, lambda _: None, lambda: False)
    assert calls[0]['grid_method'] == 'terrain'
    assert calls[0]['require_index'] is True


def test_two_bundle_methods_keep_same_source_and_sc_and_separate_grid_checkpoint(tmp_path, monkeypatch):
    monkeypatch.setenv('NAV_RUNTIME_DIR', str(tmp_path/'runtime'))
    x, y = np.meshgrid(np.linspace(0, 3, 61), np.linspace(0, 2, 41))
    points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    source = save_pcd(tmp_path/'source.pcd', points); original = source.read_bytes()
    sc = tmp_path/'source.pcd.sc.v2'; sc.write_bytes(b'original-sc')
    calls = []
    def fake_terrain(points, **kwargs):
        calls.append(points.copy())
        return GridMap(np.zeros((20, 30), dtype=np.int8), .05, np.zeros(3)), {}, dict(surface_resolution_m=.2)
    monkeypatch.setattr(terrain_grid, 'build_terrain_grid', fake_terrain)
    index_sources = []
    def index(source, log):
        index_sources.append(source)
        return str(sc)
    old = navigation_bundle.build(source, tmp_path/'maps', 'room', 0., index_builder=index, work_directory=tmp_path/'work')
    new = navigation_bundle.build(source, tmp_path/'maps', 'room', 0., index_builder=index, work_directory=tmp_path/'work', grid_method='terrain')
    assert len(calls) == 1 and index_sources == [source, source]
    old_scene = json.loads(Path(old['scene']).read_text()); new_scene = json.loads(Path(new['scene']).read_text())
    assert old['scene'] != new['scene'] and old_scene['grid'] != new_scene['grid']
    assert old_scene['scan_context'] == new_scene['scan_context']
    assert old_scene['generated']['grid_method'] == 'observed'
    assert new_scene['generated']['grid_method'] == 'terrain'
    assert sc.read_bytes() == b'original-sc' and source.read_bytes() == original
    navigation_bundle.build(source, tmp_path/'maps', 'room', 0., index_builder=index, work_directory=tmp_path/'work', grid_method='terrain')
    assert len(calls) == 1, 'same-method checkpoint should reuse the completed terrain grid'


def test_new_conversion_uses_complete_edited_transformed_cloud_in_background(window, monkeypatch, application):
    entered, release = threading.Event(), threading.Event(); received = []
    points = np.array([[0., -.3, 0.], [1., .7, 0.], [2., 1., .5]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.cloud_axis_combo.setCurrentIndex(window.cloud_axis_combo.findData('XZY'))
    window.z_min_spin.setValue(10.); window.z_max_spin.setValue(11.)
    def calculate(points, **kwargs):
        received.append((points.copy(), kwargs)); entered.set()
        assert release.wait(3)
        return GridMap(np.array([[-1, 0, 100]], dtype=np.int8), .05, np.zeros(3)), {}, dict(
            ground_reference_z_m=-.3, occupied_cells=1, free_cells=1, unknown_cells=1)
    monkeypatch.setattr(terrain_grid, 'build_terrain_grid', calculate)
    window.terrain_convert_button.click()
    assert entered.wait(2) and window.operations_page.library.worker.busy
    assert not window.convert_button.isEnabled()
    np.testing.assert_array_equal(received[0][0], points[:, [0, 2, 1]])
    np.testing.assert_array_equal(window.local_cloud.points, points)
    release.set()
    deadline = time.monotonic()+3
    while window.operations_page.library.worker.busy and time.monotonic() < deadline:
        application.processEvents(); time.sleep(.01)
    assert window.convert_button.isEnabled() and window.terrain_convert_button.isEnabled()
    assert window.local_grid.occupancy.tolist() == [[-1, 0, 100]]
    assert '局部高程通行性转换完成' in window.map_status.text()


def test_failed_new_method_reports_error_and_does_not_replace_previous_grid(window, monkeypatch, application):
    window._set_local_cloud(PointCloudMap(np.array([[0., 0., 0.]], dtype=np.float32)), reset_original=True)
    original = GridMap(np.zeros((2, 3), dtype=np.int8), .05, np.zeros(3))
    window._set_local_grid(original, reset_original=True)
    def calculate(*args, **kwargs):
        raise ValueError('extractor unavailable')
    monkeypatch.setattr(terrain_grid, 'build_terrain_grid', calculate)
    window.terrain_convert_button.click()
    deadline = time.monotonic()+3
    while window.operations_page.library.worker.busy and time.monotonic() < deadline:
        application.processEvents(); time.sleep(.01)
    assert window.local_grid is original
    assert 'extractor unavailable' in window.map_status.text()
    assert window.terrain_convert_button.isEnabled()


def test_cancelled_extractor_is_reaped(tmp_path):
    helper = tmp_path/'extractor'; pid_file = tmp_path/'child.pid'
    helper.write_text('#!/usr/bin/python3\nimport os,time\nfrom pathlib import Path\nPath('+repr(str(pid_file))+').write_text(str(os.getpid()))\ntime.sleep(30)\n')
    helper.chmod(0o700)
    started = time.monotonic()
    with pytest.raises(navigation_bundle.NavigationBuildCancelled):
        terrain_grid.build_terrain_grid(np.zeros((30, 3)), extractor=helper,
            cancel=lambda: time.monotonic()-started > .3)
    pid = int(pid_file.read_text())
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)

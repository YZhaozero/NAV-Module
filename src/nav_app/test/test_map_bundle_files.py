"""Map bundles bind explicit assets, preview safely, and open full editor data."""
import json
import time

import numpy as np
import pytest

from nav_app.map_processing import GridMap, save_grid_map, save_pcd
from test_height_preview import window
from test_localization_tools import write_scan_context_cache


@pytest.fixture
def assets(tmp_path):
    source = tmp_path / 'external'
    source.mkdir()
    cloud = save_pcd(source / 'points.pcd', np.array([[0, 0, 0], [1, 2, .5], [2, 1, 1]], dtype=np.float32))
    grid, _ = save_grid_map(source / 'occupancy.yaml', GridMap(
        np.array([[0, -1], [100, 0]], dtype=np.int8), .04, np.array([2., -1., .2])))
    sc = source / 'independent-index.sc.v2'
    write_scan_context_cache(sc, cloud)
    return cloud, sc, grid


def wait_for_worker(page, application):
    deadline = time.monotonic() + 5
    while (page.worker.busy or page.preview_worker.busy) and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(.005)
    application.processEvents()
    assert not page.worker.busy and not page.preview_worker.busy


def test_register_and_edit_explicit_assets_preserves_bundle_metadata(window, assets, application):
    page = window.operations_page.library
    cloud, sc, grid = assets
    page.bundle_title.setText('显式文件地图包')
    page.cloud_file.setText(str(cloud))
    page.sc_file.setText(str(sc))
    page.grid_file.setText(str(grid))
    page.coordinates_verified.setChecked(True)
    page.register()
    wait_for_worker(page, application)
    packages = list(window.map_storage_dir.glob('*.scene.json'))
    assert len(packages) == 1
    document = json.loads(packages[0].read_text())
    assert (packages[0].parent / document['scan_context']).resolve() == sc
    assert document['scan_context_verified'] and document['coordinates_verified']
    assert page.selected().global_ready
    document['destinations'] = [dict(name='门口', xy=[1, 2])]
    document['viewer_url'] = 'http://127.0.0.1:8080/'
    document['cloud'], document['grid'] = str(cloud), str(grid)
    document['generated'] = dict(gravity_aligned=True, source_sha256='metadata-to-preserve')
    packages[0].write_text(json.dumps(document))
    page.refresh()
    page.bundle_title.setText('改名后')
    page.register()
    wait_for_worker(page, application)
    updated = json.loads(packages[0].read_text())
    assert updated['title'] == '改名后'
    assert updated['destinations'] == document['destinations']
    assert updated['viewer_url'] == document['viewer_url']
    assert updated['generated'] == document['generated']
    assert len(list(window.map_storage_dir.glob('*.scene.json'))) == 1


def test_mismatched_sc_cannot_be_bound_to_another_pcd(window, assets, application):
    page = window.operations_page.library
    cloud, sc, grid = assets
    other = save_pcd(cloud.with_name('other.pcd'), np.array([[99, 10, 1]], dtype=np.float32))
    page.bundle_title.setText('错误组合')
    page.cloud_file.setText(str(other))
    page.sc_file.setText(str(sc))
    page.grid_file.setText(str(grid))
    page.register()
    wait_for_worker(page, application)
    assert '不匹配' in page.status.text()
    assert not list(window.map_storage_dir.glob('*.scene.json'))


def test_cloud_preview_does_not_replace_edits_and_processing_loads_full_cloud(window, tmp_path, application):
    page = window.operations_page.library
    points = np.random.default_rng(3).normal(size=(310001, 3)).astype(np.float32)
    cloud = save_pcd(tmp_path / 'large.pcd', points)
    page.cloud_file.setText(str(cloud))
    page.open_asset('cloud')
    wait_for_worker(page, application)
    assert 0 < len(page.cloud_preview._cloud_points) < len(points)
    assert window.local_cloud is None
    assert page.preview_tabs.currentWidget() is page.cloud_preview
    page.open_asset('cloud', edit=True)
    wait_for_worker(page, application)
    assert len(window.local_cloud.points) == len(points)
    assert np.array_equal(window.local_cloud.points, points)
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is window.map_tools_page
    assert window.editor_map_panel.display_mode == 'cloud'


def test_grid_preview_and_editor_keep_resolution_origin_and_cells(window, assets, application):
    page = window.operations_page.library
    _cloud, _sc, grid = assets
    page.grid_file.setText(str(grid))
    page.open_asset('grid')
    wait_for_worker(page, application)
    assert page.preview_tabs.currentWidget() is page.grid_preview
    assert page.grid_preview._grid_resolution == pytest.approx(.04)
    assert window.local_grid is None
    page.open_asset('grid', edit=True)
    wait_for_worker(page, application)
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is window.map_tools_page
    assert window.editor_map_panel.display_mode == 'grid'
    assert window.local_grid.resolution == pytest.approx(.04)
    assert np.allclose(window.local_grid.origin, [2, -1, .2])
    assert window.local_grid.occupancy.tolist() == [[0, -1], [100, 0]]


@pytest.mark.parametrize('content', ['[invalid', '- not-a-grid'])
def test_invalid_yaml_reports_error_and_releases_worker(window, tmp_path, application, content):
    page = window.operations_page.library
    path = tmp_path / 'broken.yaml'
    path.write_text(content)
    page.grid_file.setText(str(path))
    page.open_asset('grid')
    wait_for_worker(page, application)
    assert '预览读取失败' in page.status.text()
    assert window.local_grid is None


def test_changing_bound_map_paths_requires_coordinate_confirmation_again(window, assets):
    page = window.operations_page.library
    cloud, _sc, grid = assets
    page.coordinates_verified.setChecked(True)
    page.cloud_file.setText(str(cloud))
    assert not page.coordinates_verified.isChecked()
    page.coordinates_verified.setChecked(True)
    page.grid_file.setText(str(grid))
    assert not page.coordinates_verified.isChecked()



def test_compact_preview_scrolls_fully_into_view(window, assets, application):
    from PyQt5.QtCore import QPoint
    page = window.operations_page.library
    window.resize(800, 480)
    window.show_map_library()
    window.show()
    application.processEvents()
    application.processEvents()
    page.grid_file.setText(str(assets[2]))
    page.open_asset('grid')
    wait_for_worker(page, application)
    viewport = page.asset_scroll.viewport()
    top = page.preview_tabs.mapTo(viewport, QPoint(0, 0)).y()
    bottom = page.preview_tabs.mapTo(viewport, page.preview_tabs.rect().bottomRight()).y()
    assert top >= 0
    assert bottom < viewport.height()



def test_select_bundle_and_switch_tabs_load_without_view_button(window, assets, application):
    cloud, sc, grid = assets
    manifest = window.map_storage_dir / 'automatic.scene.json'
    manifest.write_text(json.dumps(dict(version=2, title='Automatic preview', cloud=str(cloud),
        grid=str(grid), scan_context=str(sc), coordinates_verified=True, scan_context_verified=True)))
    window.resize(1280, 720)
    window.show_map_library()
    window.show()
    application.processEvents()
    page = window.operations_page.library
    page.maps.setCurrentRow(0)
    wait_for_worker(page, application)
    assert len(page.cloud_preview._cloud_points) == 3
    page.preview_tabs.setCurrentWidget(page.grid_preview)
    wait_for_worker(page, application)
    assert page.grid_preview._grid_pixmap is not None
    assert page.grid_preview._grid_resolution == pytest.approx(.04)
    assert window.local_cloud is None and window.local_grid is None



def test_switching_back_reuses_loaded_preview(window, assets, application, monkeypatch):
    from nav_app import map_processing
    cloud, sc, grid = assets
    (window.map_storage_dir/'cached.scene.json').write_text(json.dumps(dict(version=2,
        title='Cached preview',cloud=str(cloud),grid=str(grid),scan_context=str(sc))))
    original = map_processing.load_pointcloud
    reads = []
    monkeypatch.setattr(map_processing, 'load_pointcloud',
        lambda *args, **kw: reads.append(str(args[0])) or original(*args, **kw))
    window.show_map_library()
    window.show()
    application.processEvents()
    page = window.operations_page.library
    wait_for_worker(page, application)
    page.preview_tabs.setCurrentWidget(page.grid_preview)
    wait_for_worker(page, application)
    page.preview_tabs.setCurrentWidget(page.cloud_preview)
    wait_for_worker(page, application)
    assert reads == [str(cloud)]
    assert len(page.cloud_preview._cloud_points) == 3
    assert '正在加载' not in page.status.text()


def test_fast_bundle_selection_only_displays_latest_file(window, assets, application, monkeypatch):
    from pathlib import Path
    import threading
    from nav_app import map_processing
    first, _sc, grid = assets
    second = save_pcd(first.with_name('second.pcd'), np.array([[8, 1, .1], [9, 2, .2]], dtype=np.float32))
    for title, cloud in [('first', first), ('second', second)]:
        (window.map_storage_dir/(title+'.scene.json')).write_text(json.dumps(dict(version=2,
            title=title,cloud=str(cloud),grid=str(grid))))
    original = map_processing.load_pointcloud
    entered, release = threading.Event(), threading.Event()
    def delayed(path, **kwargs):
        if Path(path) == first:
            entered.set()
            assert release.wait(timeout=5)
        return original(path, **kwargs)
    monkeypatch.setattr(map_processing, 'load_pointcloud', delayed)
    page = window.operations_page.library
    page.refresh()
    page.maps.setCurrentRow(next(i for i,s in enumerate(page.scenes) if s.cloud == str(first)))
    window.show_map_library()
    window.show()
    application.processEvents()
    try:
        assert entered.wait(timeout=1)
        page.maps.setCurrentRow(next(i for i,s in enumerate(page.scenes) if s.cloud == str(second)))
    finally:
        release.set()
        wait_for_worker(page, application)
    assert page.cloud_file.text() == str(second)
    assert page._preview_loaded['cloud'] == str(second)
    assert np.array_equal(page.cloud_preview._cloud_source_points,
                          np.array([[8, 1, .1], [9, 2, .2]], dtype=np.float32))


def test_preview_failure_clears_old_map_and_displays_filename(window, tmp_path, application):
    page = window.operations_page.library
    page.cloud_preview.set_pointcloud(np.array([[0, 0, 0]], dtype=np.float32))
    missing = tmp_path/'missing.pcd'
    page.cloud_file.setText(str(missing))
    page.open_asset('cloud')
    wait_for_worker(page, application)
    assert len(page.cloud_preview._cloud_points) == 0
    assert '预览读取失败' in page.status.text()
    assert 'missing.pcd' in page.status.text()


def test_choosing_grid_file_opens_preview_without_extra_view_click(window, assets, application, monkeypatch):
    from nav_app.ui import operations
    page = window.operations_page.library
    monkeypatch.setattr(operations.QFileDialog, 'getOpenFileName', lambda *args: (str(assets[2]), ''))
    page.choose_asset('grid')
    wait_for_worker(page, application)
    assert page.grid_file.text() == str(assets[2])
    assert page.preview_tabs.currentWidget() is page.grid_preview
    assert page.grid_preview._grid_pixmap is not None
    assert page.grid_preview._grid_resolution == pytest.approx(.04)



def test_return_to_library_keeps_unfinished_new_bundle_fields(window, assets, application):
    cloud, _sc, grid = assets
    (window.map_storage_dir/'existing.scene.json').write_text(json.dumps(dict(version=2,
        title='Existing',cloud=str(cloud),grid=str(grid))))
    window.show_map_library()
    window.show()
    application.processEvents()
    page = window.operations_page.library
    wait_for_worker(page, application)
    page.new_bundle()
    page.bundle_title.setText('未保存草稿')
    page.grid_file.setText(str(grid))
    window.show_system_home()
    application.processEvents()
    window.show_map_library()
    application.processEvents()
    wait_for_worker(page, application)
    assert page.bundle_title.text() == '未保存草稿'
    assert page.cloud_file.text() == ''
    assert page.grid_file.text() == str(grid)
    assert page.maps.currentRow() == -1

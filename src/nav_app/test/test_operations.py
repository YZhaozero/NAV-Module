"""Integration boundaries between the new tools and the existing navigation UI."""
import json
from pathlib import Path

import pytest
from PyQt5.QtGui import QCloseEvent

from nav_app.qt_nav_node import NavigationWindow
from nav_app.ros_launch_process import RosLaunchProcess
from nav_app.features.rosbag import PlaybackRequest


@pytest.fixture
def window(tmp_path, application):
    class Node:
        navigation_status = 'ready'
        map_storage_dir = str(tmp_path)
        def send_navigation_waypoints(self, points): return bool(points)
        def cancel_navigation(self): pass
        def set_clock_mode(self, enabled): self.simulated = enabled
    view = NavigationWindow(Node())
    view.refresh_timer.stop()
    view.operations_page.timer.stop()
    yield view
    view.close()
    application.processEvents()


def scene(root, verified):
    (root / 'map.pcd').write_text('placeholder')
    (root / 'map.yaml').write_text('placeholder')
    path = root / 'map.scene.json'
    path.write_text(json.dumps(dict(version=2, title='room', frame='map',
        cloud='map.pcd', grid='map.yaml', coordinates_verified=verified)))
    return path


def test_unverified_bundle_never_changes_navigation_maps(window, tmp_path):
    scene(tmp_path, False)
    page = window.operations_page.library
    page.refresh(); page.maps.setCurrentRow(0)
    old = window.navigation_stack_page.localization_map.text()
    page.load()
    assert window.navigation_stack_page.localization_map.text() == old
    assert '核对' in page.status.text()


def test_verified_bundle_selects_both_navigation_assets(window, tmp_path, monkeypatch):
    scene(tmp_path, True)
    import numpy as np
    from nav_app.map_processing import GridMap, save_grid_map
    grid = GridMap(np.zeros((12, 20), dtype=np.int16), .05, np.array([1., 2., .3]))
    save_grid_map(tmp_path / 'map.yaml', grid)
    page = window.operations_page.library
    page.refresh(); page.maps.setCurrentRow(0)
    loaded = []
    monkeypatch.setattr(window, 'load_navigation_localization_map', lambda value: loaded.append(value) or True)
    page.load()
    assert loaded == [str(tmp_path / 'map.pcd')]
    assert window.navigation_stack_page.navigation_map.text() == str(tmp_path / 'map.yaml')
    np.testing.assert_array_equal(window.local_grid.occupancy, grid.occupancy)
    np.testing.assert_allclose(window.local_grid.origin, grid.origin)
    assert window.node.grid_frame == "map"
    window.node.localization_tools = window.localization_tools_controller
    window.node.map_navigation_state = lambda: (None, None, 0.)
    window.node.navigation_route_ready = lambda: False
    window.show_relocalization()
    assert window.relocalization_page.scene_key is not None
    assert window.relocalization_page.map._grid_pixmap is not None
    assert window.relocalization_page.map._grid_shape == (12, 20)


def test_global_retry_while_waiting_for_localization(window, monkeypatch):
    window.pages.setCurrentWidget(window.relocalization_page)
    window.local_grid = object()
    window.node.map_navigation_state = lambda: (None, None, 0.)
    window.node.navigation_route_ready = lambda: False
    monkeypatch.setattr(window, "navigation_localization_backend", lambda: "pointcloud_localizer")
    monkeypatch.setattr(window, "_relocalization_map_identity", lambda: ("map", "a" * 64))
    controller = window.localization_tools_controller
    ready = True
    monkeypatch.setattr(controller, "initial_pose_readiness", lambda *_: (ready, ""))
    monkeypatch.setattr(controller, "relocalization_timeout_readiness", lambda _: (True, ""))
    for state, ready, expected in (
        ("starting", True, True), ("starting", False, False),
        ("running", True, True), ("stopped", True, False),
    ):
        window.navigation_stack_controller.overall_state = state
        window._refresh_relocalization()
        assert window.relocalization_page.automatic.isEnabled() == expected


def test_cloud_localizer_map_arrow_requires_fresh_confirmed_pose():
    from types import SimpleNamespace
    import numpy as np
    from nav_app.qt_nav_node import QtNavRosNode

    ready = {"localizer": True, "dlio": True}
    path = np.array([[1., 2., 0.]])
    node = SimpleNamespace(
        grid_frame="map", base_frame="base_link", require_localizer_quality=True,
        path_in_frame=lambda _: path, lookup_matrix=lambda *_: np.eye(4),
        navigation_component_ready=ready.__getitem__,
    )
    for localizer, odometry in ((False, True), (True, False), (True, True)):
        ready.update(localizer=localizer, dlio=odometry)
        route, xy, _ = QtNavRosNode.map_navigation_state(node)
        np.testing.assert_array_equal(route, path[:, :2])
        assert (xy is not None) == (localizer and odometry)


def test_processing_map_blocks_recording_and_algorithms(window):
    operations = window.operations_page
    operations.library.worker.busy = True
    try:
        assert not operations.admission('recording')[0]
        assert not operations.algorithm_admission('mapping')[0]
    finally:
        operations.library.worker.busy = False


def test_replay_uses_clock_and_pauses_until_algorithm_subscribes(window, tmp_path, monkeypatch):
    operations = window.operations_page
    calls = []
    monkeypatch.setattr(operations.controller, 'start_playback', lambda request: calls.append(request) or (True, 'playing'))
    request = PlaybackRequest(tmp_path, (window.mapping_page.pointcloud_combo.currentText(), '/livox/imu'), start_paused=True)
    operations.replay(request, 'mapping')
    assert calls[0].start_paused
    assert window.node.simulated is True
    assert operations.pending_replay == 'mapping'
    operations.pending_replay = None


def test_close_waits_for_recorder_and_closes_after_receipt(window, application, monkeypatch):
    process = window.operations_page.controller.record_process
    active = [True]
    monkeypatch.setattr(RosLaunchProcess, 'active', property(lambda value: value is process and active[0]))
    stopped, closed = [], []
    monkeypatch.setattr(process, 'stop_launch', lambda: stopped.append(True))
    monkeypatch.setattr(window, 'close', lambda: closed.append(True))
    event = QCloseEvent()
    window.closeEvent(event)
    assert not event.isAccepted() and stopped == [True]
    assert window._closing_after_record
    active[0] = False
    process.state_changed.emit('stopped')
    application.processEvents()
    assert closed == [True]
    window.operations_page.shutdown()


def test_diagnostics_snapshot_is_json_and_has_current_process_state(window):
    value = window.operations_page.snapshot(resources=True)
    assert json.loads(json.dumps(value))['navigation'] == window.navigation_stack_controller.overall_state
    assert 'resources' in value and 'log_store' in value


def test_rtk_controls_report_recording_conflict_without_opening_serial(window, monkeypatch):
    operations = window.operations_page
    process = operations.controller.record_process
    monkeypatch.setattr(RosLaunchProcess, 'active', property(lambda value: value is process))
    operations.rtk.port.setCurrentText('/dev/pts/999')
    operations.rtk.connect_receiver()
    assert '录包' in operations.rtk.status.text()
    assert '录包' in operations.rtk.adapter.debug_busy_reason()
    monkeypatch.undo()


def test_replay_waits_for_real_algorithm_subscriptions_before_resume(window, monkeypatch):
    from types import SimpleNamespace
    operations = window.operations_page
    process = operations.controller.play_process
    monkeypatch.setattr(RosLaunchProcess, 'active', property(lambda value: value is process))
    operations.pending_replay = 'resume'
    operations.replay_deadline = float('inf')
    operations.replay_topics = ('/livox/lidar', '/livox/imu')
    operations.replay_node = 'laserMapping'
    subscriptions = []
    monkeypatch.setattr(window.node, 'get_subscriptions_info_by_topic', lambda topic: subscriptions, raising=False)
    resumed = []
    monkeypatch.setattr(operations.controller, 'resume_playback', lambda: resumed.append(True) or (True, 'resumed'))
    operations.refresh()
    assert not resumed
    subscriptions.append(SimpleNamespace(node_name='laserMapping'))
    operations.refresh()
    assert resumed == [True] and operations.pending_replay is None
    monkeypatch.undo()


def test_catalogue_audits_pending_recording_without_loading_it(window, tmp_path, monkeypatch):
    import yaml
    from nav_app.features.bag_library import scan_bags
    bag = tmp_path / 'capture'
    bag.mkdir()
    (bag / 'part.db3').write_bytes(b'recorded data')
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
        storage_identifier='sqlite3', message_count=0, duration=dict(nanoseconds=0), topics_with_message_count=[]))))
    (bag / 'nav_manifest.json').write_text(json.dumps(dict(state='stored', storage_flush_confirmed=True)))
    operations = window.operations_page
    page = operations.recordings
    rows, notices = scan_bags([tmp_path])
    page.loaded(((rows, notices), 'available disk'), '')
    page.files.setCurrentItem(page.files.topLevelItem(0))
    calls = []
    monkeypatch.setattr(operations.audit_process, 'start_launch', lambda args: calls.append(args) or True)
    assert operations.bags.metadata is None
    assert not page.load_button.isEnabled()
    page.audit_button.click()
    assert calls == [['run', 'nav_app', 'audit_bag', str(bag), '--finalize']]
    assert operations.bags.metadata is None


def test_recording_image_choice_reaches_supervised_recorder(window, monkeypatch):
    page = window.operations_page.bags
    choices = []
    monkeypatch.setattr(page.controller, 'start_recording',
        lambda *args, **options: choices.append(options['image_storage']) or (False, 'test only'))
    page.image_storage.setCurrentIndex(page.image_storage.findData('jpeg95'))
    page._start_recording()
    page.image_storage.setCurrentIndex(page.image_storage.findData('raw'))
    page._start_recording()
    assert choices == ['jpeg95', 'raw']


@pytest.mark.parametrize('task', ['recording_delete', 'diagnostics_export'])
def test_close_waits_for_confirmed_file_operations(window, task):
    operations = window.operations_page
    operations.recordings.deleting = task == 'recording_delete'
    operations.diagnostics_page.exporting = task == 'diagnostics_export'
    event = QCloseEvent()
    window.closeEvent(event)
    assert not event.isAccepted()
    expected = (window.recordings_workspace_page if task == 'recording_delete'
                else window.diagnostics_workspace_page)
    assert window.pages.currentWidget() is expected
    operations.recordings.deleting = False
    operations.diagnostics_page.exporting = False


@pytest.mark.parametrize('size', [(800, 480), (480, 800), (1280, 900), (1920, 1080), (1850, 1053)])
def test_all_workspace_back_buttons_share_top_right_position(window, application, size, monkeypatch):
    from PyQt5.QtCore import QPoint
    from PyQt5.QtWidgets import QPushButton
    from nav_app.features import device_diagnostics
    monkeypatch.setattr(device_diagnostics, 'collect', lambda _: {})
    window.resize(*size)
    window.show()
    application.processEvents()
    assert (window.width(), window.height()) == size
    pages = [window.mapping_page, window.navigation_stack_page, window.sensor_tools_page,
             window.sensor_monitor_page, *window.task_workspaces]
    for page in pages:
        window.pages.setCurrentWidget(page)
        application.processEvents()
        returns = [button for button in page.findChildren(QPushButton)
                   if button.property('pageBack') and button.isVisible()]
        assert len(returns) == 1
        button = returns[0]
        position = button.mapTo(page, QPoint(0, 0))
        margins = page.layout().contentsMargins()
        assert position.y() == margins.top()
        assert page.width()-position.x()-button.width() == margins.right()
        assert button.width() == 88
    window.pages.setCurrentWidget(window.active_page)
    application.processEvents()
    button = window.active_page.exit_button
    inset_x, inset_y = (8, 6) if min(size) < 700 else (22, 18)
    assert button.y() == inset_y
    assert window.active_page.width()-button.x()-button.width() == inset_x


def test_nested_tool_tabs_have_one_return_and_no_duplicate_back(window, application, monkeypatch):
    from PyQt5.QtWidgets import QPushButton
    from nav_app.features import device_diagnostics
    monkeypatch.setattr(device_diagnostics, 'collect', lambda _: {})
    window.show()
    operations = window.operations_page
    for index in range(window.map_pages.count()):
        window.pages.setCurrentWidget(window.map_workspace_page)
        window.map_pages.setCurrentIndex(index)
        application.processEvents()
        returns = [b for b in window.map_workspace_page.findChildren(QPushButton)
                   if b.property('pageBack') and b.isVisible()]
        assert returns == [window.map_workspace_page.header.back_button]
    window.show_recordings()
    for inner in range(operations.bags.tabs.count()):
        operations.bags.tabs.setCurrentIndex(inner)
        application.processEvents()
        assert not [b for b in operations.bags.findChildren(QPushButton) if b.property('pageBack')]
    window.recordings_workspace_page.header.back_button.click()
    assert window.pages.currentWidget() is window.system_home_page


def test_return_preserves_task_ownership_and_sensor_preview_returns_to_parent(window, monkeypatch):
    stopped, cancelled = [], []
    monkeypatch.setattr(window.mapping_controller.process, 'stop_launch', lambda: stopped.append(True))
    monkeypatch.setattr(window.node, 'cancel_navigation', lambda: cancelled.append(True))
    window.show_mapping()
    window.mapping_page.header.back_button.click()
    assert window.pages.currentWidget() is window.mapping_workspace
    window.mapping_workspace.header.back_button.click()
    assert window.pages.currentWidget() is window.system_home_page
    assert not stopped and not cancelled
    window.show_sensor_monitor()
    window.sensor_monitor_header.back_button.click()
    assert window.pages.currentWidget() is window.sensor_tools_page
    window.sensor_tools_header.back_button.click()
    assert window.pages.currentWidget() is window.system_home_page


def test_navigation_shortcut_is_button_and_keeps_back_route(window):
    operations = window.operations_page
    button = operations.bags.mapping_view
    assert button.objectName() == 'workspaceButton' and not button.styleSheet()
    button.click()
    assert window.pages.currentWidget() is window.mapping_workspace
    window.mapping_workspace.header.back_button.click()
    assert window.pages.currentWidget() is window.system_home_page


def test_fullscreen_can_be_toggled_and_escape_restores_window(window, application):
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest
    window.show()
    application.processEvents()
    window.fullscreen_button.click()
    application.processEvents()
    assert window.isFullScreen()
    assert window.fullscreen_button.text() == '退出全屏'
    QTest.keyClick(window, Qt.Key_Escape)
    application.processEvents()
    assert not window.isFullScreen()
    assert window.fullscreen_button.text() == '全屏'
    window.show_recordings()
    QTest.keyClick(window, Qt.Key_F11)
    application.processEvents()
    assert window.isFullScreen()
    QTest.keyClick(window, Qt.Key_F11)
    application.processEvents()
    assert not window.isFullScreen()


def test_rtk_discovery_is_visible_without_selecting_or_opening_a_port(monkeypatch):
    from nav_app.ui import operations
    ports = ['/dev/ttyTHS1', '/dev/ttyTHS2']
    monkeypatch.setattr(operations, 'list_rtk_serial_ports', lambda: ports.copy())
    page = operations.RtkPage(object(), lambda: '')
    try:
        assert [page.port.itemText(i) for i in range(page.port.count())] == ports
        assert page.port.currentIndex() == -1
        assert not page.port.isEditable()
        assert '2' in page.port.placeholderText()
        assert '/dev/ttyTHS1' in page.status.text()
        assert not page.process.active
        page.port.setCurrentIndex(1)
        page.search()
        assert page.port.currentText() == '/dev/ttyTHS2'
        ports.clear()
        page.search()
        assert page.port.currentIndex() == -1
        assert '未发现' in page.status.text()
        page.manual_port.setChecked(True)
        page.port.setCurrentText('/dev/pts/7')
        assert page.port.currentText() == '/dev/pts/7'
        assert not page.process.active
    finally:
        page.shutdown()


def test_peer_button_roles_share_font_and_palette_across_pages(window, application):
    from PyQt5.QtGui import QPalette
    operations = window.operations_page
    operations.rtk.timer.stop()
    peers = [
        [window.open_navigation_stack_button, window.open_sensor_tools_button,
         window.open_mapping_button, window.open_map_tools_button, window.open_operations_button,
         operations.recordings.load_button, operations.bags.mapping_view],
        [window.start_button, window.mapping_page.start_button, operations.logs.export_selected,
         operations.diagnostics_page.json_button, operations.diagnostics_page.bundle_button,
         operations.rtk.monitor.command_panel.send_button],
    ]
    for width, height in [(800, 480), (1280, 900), (480, 800), (800, 480)]:
        window.resize(width, height)
        window.show()
        application.processEvents()
        for group in peers:
            for button in group:
                button.setEnabled(True)
                button.ensurePolished()
            application.processEvents()
            first = group[0]
            assert first.font().pixelSize() == (11 if min(width, height) < 700 else 13)
            for button in group[1:]:
                assert button.palette().color(QPalette.Button) == first.palette().color(QPalette.Button)
                assert button.palette().color(QPalette.ButtonText) == first.palette().color(QPalette.ButtonText)
                assert button.font().pixelSize() == first.font().pixelSize()

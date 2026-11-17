"""System routes and automatic navigation startup without dispatching motion."""
from types import SimpleNamespace
from pathlib import Path
import time

import numpy as np
import pytest
from PyQt5.QtCore import QPoint, Qt, QSettings

from nav_app.map_processing import PointCloudMap, GridMap, save_grid_map
from nav_app.qt_nav_node import NavigationWindow


@pytest.fixture
def window(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path / "tasks"))
    node = SimpleNamespace(navigation_status="等待准备", map_storage_dir=str(tmp_path / "maps"),
                           mapping_backend="woosh_dlio")
    view = NavigationWindow(node)
    view.refresh_timer.stop()
    view.operations_page.timer.stop()
    yield view
    view.close()
    application.processEvents()


def test_home_routes_and_return_buttons(window, application):
    home = window.system_home_page
    assert window.pages.currentWidget() is home
    assert not window.mapping_controller.process.active
    assert window.navigation_stack_controller.overall_state == "stopped"
    home.navigation_button.click()
    assert window.pages.currentWidget() is window.setup_page
    window.setup_home_button.click()
    assert window.pages.currentWidget() is home
    home.settings_button.click()
    assert window.pages.currentWidget() is window.system_settings_page
    window.system_settings_page.buttons["sensors"].click()
    assert window.pages.currentWidget() is window.sensor_tools_page
    window.sensor_tools_header.back_button.click()
    assert window.pages.currentWidget() is home
    home.maps_button.click()
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is window.operations_page.library
    window.map_workspace_page.header.back_button.click()
    assert window.pages.currentWidget() is home
    home.phone_button.click()
    assert window.pages.currentWidget() is window.phone_workspace_page
    assert not window.mapping_controller.process.active
    assert window.navigation_stack_controller.overall_state == "stopped"


def test_rtk_lives_in_sensors_and_keeps_exclusive_debug_guard(window):
    rtk = window.operations_page.rtk
    assert window.sensor_pages.indexOf(rtk) >= 0
    assert not hasattr(window.operations_page, "tabs")
    assert window.sensor_tools_page.isAncestorOf(rtk)
    before = rtk.adapter
    try:
        rtk.adapter = SimpleNamespace(available=True, debug_blocks_tasks=True)
        accepted, reason = window.operations_page.algorithm_admission("mapping")
        assert not accepted and "RTK" in reason
    finally:
        rtk.adapter = before


def test_unsaved_map_is_named_without_claiming_a_file(window):
    window.local_cloud = PointCloudMap(np.zeros((1, 3)))
    window._refresh_system_home()
    assert window.system_home_page.map_name.text() == "当前地图（尚未保存）"
    assert window.system_home_page.state.text() == "WOOSH · 待命"


@pytest.mark.parametrize("size", [(800, 480), (480, 800), (1280, 900)])
def test_home_actions_fit_the_screen(window, application, size):
    window.resize(*size)
    window.show()
    application.processEvents()
    home = window.system_home_page
    for button in (home.mapping_button, home.navigation_button, home.maps_button,
                   home.settings_button, home.phone_button, home.device_button):
        assert button.isVisible()
        assert home.rect().contains(button.mapTo(home, QPoint(0, 0)))
        assert home.rect().contains(button.mapTo(home, button.rect().bottomRight()))
    assert window.width() == size[0] and window.height() == size[1]


def test_maps_are_grouped_and_progress_keeps_the_same_task_owner(window):
    operations = window.operations_page
    owner = operations.task_coordinator
    assert [window.map_pages.tabText(i) for i in range(window.map_pages.count())] == [
        "地图包", "地图处理", "处理进度"]
    window.show_map_tools()
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is window.map_tools_page
    assert window.map_workspace_page.isAncestorOf(window.load_cloud_button)
    operations.show_mapping_progress()
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is operations.map_tasks
    assert operations.task_coordinator is owner


def test_trajectory_is_in_sensor_data_and_tools_have_independent_routes(window, monkeypatch):
    operations = window.operations_page
    assert window.sensor_monitor_tabs.indexOf(operations.trajectory) == 3
    assert window.sensor_monitor_page.isAncestorOf(operations.trajectory)
    stopped = []
    monkeypatch.setattr(operations.trajectory, "shutdown", lambda: stopped.append(True))
    window.show_sensor_monitor()
    window.sensor_monitor_tabs.setCurrentWidget(operations.trajectory)
    window.sensor_monitor_header.back_button.click()
    assert window.pages.currentWidget() is window.sensor_tools_page
    assert not stopped
    window.show_recordings()
    assert window.pages.currentWidget() is window.recordings_workspace_page
    window.show_logs()
    assert window.pages.currentWidget() is window.logs_workspace_page
    assert window.logs_workspace_page.isAncestorOf(operations.logs)
    window.show_phone_remote()
    assert window.pages.currentWidget() is window.phone_workspace_page
    assert not hasattr(operations, "tabs")


def test_settings_routes_are_separate_from_map_and_sensor_workspaces(window, monkeypatch):
    from nav_app.features import device_diagnostics
    monkeypatch.setattr(device_diagnostics, "collect", lambda _: {})
    routes = {
        "maps": window.map_workspace_page,
        "recordings": window.recordings_workspace_page,
        "logs": window.logs_workspace_page,
        "diagnostics": window.diagnostics_workspace_page,
        "phone": window.phone_workspace_page,
    }
    for key, destination in routes.items():
        window.show_system_settings()
        window.system_settings_page.buttons[key].click()
        assert window.pages.currentWidget() is destination
        destination.header.back_button.click()
        assert window.pages.currentWidget() is window.system_home_page
    assert window.system_settings_page.scroll.property("gsTouchScroll")


@pytest.mark.parametrize("size", [(800, 480), (480, 800), (1280, 900)])
def test_grouped_settings_actions_are_reachable(window, application, size):
    window.resize(*size)
    window.show_system_settings()
    window.show()
    application.processEvents()
    page = window.system_settings_page
    for button in page.buttons.values():
        page.scroll.ensureWidgetVisible(button, 4, 4)
        application.processEvents()
        viewport = page.scroll.viewport()
        assert viewport.rect().contains(button.mapTo(viewport, button.rect().center()))
    assert page.exit_button.isVisible() and page.fullscreen_button.isVisible()
    assert window.width() == size[0] and window.height() == size[1]


def test_split_workspaces_shutdown_each_rtk_instance_once(window, monkeypatch):
    rtk = window.operations_page.rtk
    original = rtk.shutdown
    calls = []
    monkeypatch.setattr(rtk, "shutdown", lambda: (calls.append("rtk"), original()))
    window.operations_page.shutdown()
    assert calls == ["rtk"]


def test_log_workspace_loads_packages_on_entry_and_reentry(window, application, tmp_path, monkeypatch):
    from nav_app.features.log_store import LogStore
    from nav_app.ui import log_management

    page = window.operations_page.logs
    root = tmp_path / "logs"
    for count in (1, 2):
        store = LogStore(root)
        try:
            store.write("device", "startup", "NAV started")
            assert store.flush()
            if count == 1:
                assert store.finish()
            monkeypatch.setattr(log_management, "log_store", lambda: store)
            window.show_system_settings()
            window.system_settings_page.buttons["logs"].click()
            deadline = time.monotonic() + 4.
            while page.worker.busy and time.monotonic() < deadline:
                application.processEvents()
                time.sleep(.01)
            assert not page.worker.busy
            assert window.pages.currentWidget() is window.logs_workspace_page
            assert page.location.text() == "日志包目录：" + str(root)
            assert page.files.count() == count
            assert f"{count} 个日志包" in page.status.text()
            assert page.rows[0]["active"] is (count == 2)
            page.files.setCurrentRow(0)
            assert page.export_selected.isEnabled() is (count == 1)
            assert page.delete_selected.isEnabled() is (count == 1)
            if count == 2:
                page.files.setCurrentRow(1)
                assert page.export_selected.isEnabled()
                assert page.delete_selected.isEnabled()
            window.logs_workspace_page.header.back_button.click()
            assert window.pages.currentWidget() is window.system_home_page
        finally:
            assert store.finish()


def test_map_tab_reloads_inventory_without_discarding_draft(window, application):
    page = window.operations_page.library
    root = Path(window.map_storage_dir)
    root.mkdir(parents=True, exist_ok=True)
    window.show()
    for count in (1, 2):
        (root / f"room-{count}.pcd").write_text("placeholder")
        window.show_map_tools()
        window.map_pages.setCurrentWidget(page)
        application.processEvents()
        assert page.maps.count() == count
        if count == 1:
            page.bundle_title.setText("尚未保存的地图包")
            page.cloud_file.setText("draft.pcd")
        else:
            assert page.bundle_title.text() == "尚未保存的地图包"
            assert page.cloud_file.text() == "draft.pcd"
        window.map_workspace_page.header.back_button.click()
    window.show_map_library()
    assert page.maps.count() == 2
    assert page.bundle_title.text() == "尚未保存的地图包"


def test_recording_topics_load_and_update_without_resetting_choices(window, application, monkeypatch):
    from nav_app.features.rosbag import BagTopic, RosbagController

    controller = window.operations_page.controller
    page = window.operations_page.bags
    topics = [BagTopic("/livox/imu1", "sensor_msgs/msg/Imu")]
    monkeypatch.setattr(controller, "available_topics", lambda: tuple(topics))
    recording = [False]
    monkeypatch.setattr(RosbagController, "recording", property(lambda value: value is controller and recording[0]))
    window.show()
    try:
        for count in (1, 2):
            window.show_system_settings()
            window.system_settings_page.buttons["recordings"].click()
            application.processEvents()
            items = {page.record_topics.topLevelItem(i).text(1): page.record_topics.topLevelItem(i)
                     for i in range(page.record_topics.topLevelItemCount())}
            assert items[f"/livox/imu{count}"].text(3) == "已发现"
            if count == 1:
                assert items["/livox/imu1"].checkState(0) == Qt.Checked
                items["/livox/imu1"].setCheckState(0, Qt.Unchecked)
                topics.append(BagTopic("/livox/imu2", "sensor_msgs/msg/Imu"))
            else:
                assert items["/livox/imu1"].checkState(0) == Qt.Unchecked
                assert items["/livox/imu2"].checkState(0) == Qt.Checked
            window.show_system_home()
        from PyQt5.QtWidgets import QPushButton
        next(button for button in page.findChildren(QPushButton)
             if button.text() == "刷新并选择传感器基础话题").click()
        assert all(page.record_topics.topLevelItem(i).checkState(0) == Qt.Checked
                   for i in range(page.record_topics.topLevelItemCount())
                   if page.record_topics.topLevelItem(i).text(1) in ("/livox/imu1", "/livox/imu2"))
        window.show_recordings()
        page.tabs.setCurrentIndex(1)
        topics.append(BagTopic("/livox/imu3", "sensor_msgs/msg/Imu"))
        page.tabs.setCurrentIndex(0)
        assert any(page.record_topics.topLevelItem(i).text(1) == "/livox/imu3"
                   for i in range(page.record_topics.topLevelItemCount()))
        window.show_system_home()
        recording[0] = True
        topics.append(BagTopic("/livox/imu4", "sensor_msgs/msg/Imu"))
        window.show_recordings()
        assert all(page.record_topics.topLevelItem(i).text(1) != "/livox/imu4"
                   for i in range(page.record_topics.topLevelItemCount()))
    finally:
        recording[0] = False


def test_recording_inventory_refreshes_on_tab_entry_and_reentry(window, application, tmp_path):
    import yaml

    root = tmp_path / "records"
    page = window.operations_page.recordings
    window.operations_page.bags.record_directory.setText(str(root))
    page.settings = QSettings(str(tmp_path / "bag-settings.ini"), QSettings.IniFormat)
    window.show()
    for count in (1, 2):
        bag = root / f"capture-{count}"
        bag.mkdir(parents=True)
        metadata = dict(storage_identifier="sqlite3", message_count=1,
                        duration=dict(nanoseconds=1_000_000_000),
                        starting_time=dict(nanoseconds=1_000_000_000),
                        topics_with_message_count=[dict(topic_metadata=dict(name="/imu", type="sensor_msgs/msg/Imu"), message_count=1)])
        (bag / "metadata.yaml").write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=metadata)))
        (bag / "part.db3").write_bytes(b"test payload")
        window.show_recordings()
        window.operations_page.bags.tabs.setCurrentWidget(page)
        deadline = time.monotonic() + 4.
        while page.worker.busy and time.monotonic() < deadline:
            application.processEvents()
            time.sleep(.01)
        assert not page.worker.busy
        assert page.files.topLevelItemCount() == count
        assert {entry.path.name for entry in page.rows} == {f"capture-{i}" for i in range(1, count+1)}
        window.show_system_home()


def test_trajectory_reloads_maps_and_history_preserving_selected_session(window, application, tmp_path):
    page = window.operations_page.trajectory
    root = tmp_path / "trajectory"
    page.recorder = SimpleNamespace(root=root, snapshot=lambda: dict(
        active=False, manifest={}, samples={}, error="", dropped=0, session=""))
    maps = Path(window.map_storage_dir)
    maps.mkdir(parents=True, exist_ok=True)
    page.view.setCurrentIndex(1)
    window.show()
    for count in (1, 2):
        (maps / f"room-{count}.pcd").write_text("placeholder")
        session = root / f"session-{count}"
        session.mkdir(parents=True)
        (session / "manifest.json").write_text("{}")
        window.show_sensor_monitor()
        window.sensor_monitor_tabs.setCurrentWidget(page)
        deadline = time.monotonic() + 4.
        while page.worker.busy and time.monotonic() < deadline:
            application.processEvents()
            time.sleep(.01)
        assert not page.worker.busy
        assert page.maps.count() == count
        assert page.sessions.count() == count
        assert page.sessions.currentData() == str(root / "session-1")
        window.show_system_home()
    window.show_sensor_monitor()
    session = root / "session-3"
    session.mkdir()
    (session / "manifest.json").write_text("{}")
    page.tabs.setCurrentIndex(1)
    deadline = time.monotonic() + 4.
    while page.worker.busy and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(.01)
    assert not page.worker.busy
    assert page.sessions.count() == 3
    assert page.sessions.currentData() == str(root / "session-1")
    assert not window.mapping_controller.process.active


def test_rtk_entry_discovers_hotplugged_ports_and_preserves_manual_path(window, application, monkeypatch):
    from nav_app.ui import operations

    page = window.operations_page.rtk
    ports = ["/dev/ttyUSB0"]
    monkeypatch.setattr(operations, "list_rtk_serial_ports", lambda: list(ports))
    window.show()
    window.show_sensor_tools()
    window.sensor_pages.setCurrentWidget(page)
    assert page.port.findText("/dev/ttyUSB0") >= 0
    page.manual_port.setChecked(True)
    page.port.setEditText("/dev/receiver-custom")
    window.show_system_home()
    ports.append("/dev/ttyUSB1")
    window.show_sensor_tools()
    assert page.port.findText("/dev/ttyUSB1") >= 0
    assert page.port.currentText() == "/dev/receiver-custom"
    assert not page.process.active


def test_diagnostics_and_phone_keep_refreshing_after_split(window, application, monkeypatch):
    from nav_app.features import device_diagnostics

    monkeypatch.setattr(device_diagnostics, "collect", lambda _: {})
    title = ["诊断一"]
    window.node.performance = SimpleNamespace(snapshot=lambda: dict(decision=dict(title=title[0])))
    window.operations_page.timer.start(20)
    phone = window.operations_page.phone
    phone.timer.setInterval(20)
    window.show()
    for count in (1, 2):
        title[0] = f"诊断{count}"
        window.show_diagnostics()
        assert window.operations_page.diagnostics_page.summary.text() == title[0]
        title[0] += "已更新"
        deadline = time.monotonic() + 4.
        while window.operations_page.diagnostics_page.summary.text() != title[0] and time.monotonic() < deadline:
            application.processEvents()
            time.sleep(.01)
        assert window.operations_page.diagnostics_page.summary.text() == title[0]
        window.show_phone_remote()
        phone.remote.error = f"测试状态{count}"
        deadline = time.monotonic() + 4.
        while phone.status.text() != phone.remote.error and time.monotonic() < deadline:
            application.processEvents()
            time.sleep(.01)
        assert phone.status.text() == phone.remote.error
        window.show_system_home()
    deadline = time.monotonic() + 4.
    while window.operations_page.diagnostics_page.worker.busy and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(.01)
    assert not window.operations_page.diagnostics_page.worker.busy


def test_map_progress_receives_task_updates_after_split(window, application, monkeypatch):
    operations = window.operations_page
    job = dict(id="ui-test-job", kind="navigation", state="queued", attempt=1, created=0.,
               payload=dict(title="测试地图"), progress={})
    monkeypatch.setattr(operations.task_coordinator, "snapshot", lambda: dict(jobs=[job]))
    operations.timer.start(20)
    window.show()
    window.show_map_progress()
    deadline = time.monotonic() + 4.
    while operations.map_tasks.rows.count() != 1 and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(.01)
    assert operations.map_tasks.rows.count() == 1
    window.show_system_home()
    manifest = Path(operations.library.root) / "test.scene.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text('{"version":2,"title":"测试地图"}')
    job["result"] = dict(scene=str(manifest))
    job["state"] = "succeeded"
    window.show_map_progress()
    deadline = time.monotonic() + 4.
    while "已完成" not in operations.map_tasks.rows.item(0).text() and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(.01)
    assert "已完成" in operations.map_tasks.rows.item(0).text()
    assert operations.map_tasks.selected()["id"] == job["id"]
    assert not window.mapping_controller.process.active


def test_sensor_topics_update_on_entry_without_starting_preview(window, application, monkeypatch):
    graph = {"lidar": ["/sensor/lidar1"], "camera": ["/sensor/camera1"], "imu": ["/sensor/imu1"]}
    monkeypatch.setattr(window.sensor_preview_adapter, "available_topics", lambda: graph)
    window.show()
    for count in (1, 2):
        window.show_sensor_monitor()
        for kind, combo in window.sensor_topic_combos.items():
            assert combo.findText(f"/sensor/{kind}{count}") >= 0
        assert not any(window.sensor_preview_adapter.topics.values())
        window.sensor_topic_combos["imu"].setCurrentText("/custom/imu")
        window.leave_sensor_monitor()
        for kind in graph:
            graph[kind].append(f"/sensor/{kind}2")
    assert window.sensor_topic_combos["imu"].currentText() == "/custom/imu"


def test_navigation_page_restores_history_without_starting_navigation(window, application):
    controller = window.navigation_stack_controller
    window.show()
    for count in (1, 2):
        controller.log_events.append(("system", f"navigation event {count}\n"))
        window.show_navigation_stack()
        assert f"navigation event {count}" in window.navigation_stack_page.log_views["all"].toPlainText()
        assert controller.overall_state == "stopped"
        window.show_system_home()
    assert "navigation event 1" in window.navigation_stack_page.log_views["all"].toPlainText()


@pytest.fixture
def navigation_grid(tmp_path):
    grid = GridMap(np.zeros((8, 8), dtype=np.int8), .05, np.zeros(3))
    path, _ = save_grid_map(tmp_path / "room.yaml", grid)
    return path


def test_enter_navigation_starts_stack_once_and_return_does_not_restart(window, monkeypatch, navigation_grid):
    starts = []
    window.navigation_stack_page.navigation_map.setText(str(navigation_grid))
    window.navigation_stack_page.localization_map.setText("room.pcd")
    def start():
        starts.append(window.navigation_stack_page.configuration())
        window.navigation_stack_controller.overall_state = "starting"
        return True, "导航系统开始启动"
    monkeypatch.setattr(window.navigation_stack_page, "start_stack", start)
    window.system_home_page.navigation_button.click()
    assert len(starts) == 1
    assert window.pages.currentWidget() is window.setup_page
    assert window.nav_status.text() == "导航系统开始启动"
    window.show_navigation_setup()
    window.system_home_page.navigation_button.click()
    assert len(starts) == 1
    window.navigation_stack_controller.overall_state = "running"
    window.system_home_page.navigation_button.click()
    assert len(starts) == 1
    assert not window.mapping_controller.process.active
    window.navigation_stack_controller.overall_state = "stopped"


def test_enter_navigation_preserves_admission_guard_and_reports_failure(window, navigation_grid):
    window.navigation_stack_page.navigation_map.setText(str(navigation_grid))
    window.navigation_stack_page.localization_map.setText("room.pcd")
    window.navigation_stack_controller.start_guard = lambda: (False, "请先停止建图")
    window.system_home_page.navigation_button.click()
    assert window.pages.currentWidget() is window.setup_page
    assert window.nav_status.text() == "请先停止建图"
    assert window.navigation_stack_controller.overall_state == "stopped"
    assert not any(process.active for process in window.navigation_stack_controller.processes.values())


def test_enter_navigation_with_missing_map_reports_reason_without_launch(window):
    window.system_home_page.navigation_button.click()
    assert "地图" in window.nav_status.text()
    assert window.navigation_stack_controller.overall_state == "stopped"
    assert not any(process.active for process in window.navigation_stack_controller.processes.values())


def test_navigation_and_editor_preserve_original_coordinates_without_leveling(window):
    points = np.array([[x, y, .25 * x] for x in range(5) for y in range(5)], dtype=np.float32)
    original = points.copy()
    window._set_local_cloud(PointCloudMap(points), reset_original=True)
    window.update_cloud_display_options()
    assert window.editor_map_panel._cloud_alignment == "original"
    np.testing.assert_array_equal(window.editor_map_panel._cloud_points, original)
    for panel in (window.map_panel, window.active_page.map_panel):
        assert panel._cloud_alignment == "original"
        assert "自动找平" not in panel.cloud_display_description()
    np.testing.assert_array_equal(window.local_cloud.points, original)

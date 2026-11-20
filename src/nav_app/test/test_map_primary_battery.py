"""Map/camera switching and read-only chassis battery updates."""
from concurrent.futures import Future
from types import SimpleNamespace
import time

import numpy as np
import pytest
from PyQt5.QtCore import QPoint

from nav_app.qt_nav_node import ActiveNavigationPage, NavigationWindow, QtNavRosNode
from woosh_robot_msgs.srv import Battery


@pytest.mark.parametrize("size", [(800, 480), (480, 800), (1600, 900)])
def test_primary_view_switch_preserves_map_camera_and_controls(application, size):
    exits, starts, modes = [], [], []
    page = ActiveNavigationPage(lambda: exits.append(True), modes.append,
                                lambda: starts.append(True))
    page.resize(*size)
    page.show()
    page.manual_button.show()
    page.set_preview_available(True)
    page.map_panel.set_navigation_state(np.array([[0., 0.], [2., 1.]]), np.array([1., .5]), .2)
    page.map_panel.set_motion_trace(np.array([[0., 0.], [1., .5]]))
    page.camera_panel.set_frame(np.zeros((120, 160, 3), dtype=np.uint8), None, 0., 1)
    application.processEvents()
    route = page.map_panel._path.copy()
    image = page.camera_panel._pixmap.cacheKey()
    assert page.camera_panel.geometry() == page.rect()
    page.view_button.click()
    assert page.map_panel.geometry() == page.rect()
    assert page.camera_panel.width() < page.width()
    assert page.camera_panel.height() < page.height()
    assert page.view_button.text() == "相机全屏"
    assert page.navigation_summary.isVisible()
    assert not page.camera_panel._show_status
    for widget in (page.camera_panel, page.grid_mode_button, page.cloud_mode_button,
                   page.follow_button, page.exit_button, page.begin_button,
                   page.manual_button, page.view_button):
        assert page.rect().contains(widget.geometry())
        assert page.childAt(widget.mapTo(page, widget.rect().center())) is widget
    page.cloud_mode_button.click()
    assert modes == ["cloud"]
    page.show_navigation_result(4, "done")
    assert page.childAt(page.result_overlay.geometry().center()) is page.result_overlay
    page.resize(size[0] + 20, size[1] + 20)
    application.processEvents()
    assert page.map_panel.geometry() == page.rect()
    page.clear_navigation_result()
    page.view_button.click()
    assert page.camera_panel.geometry() == page.rect()
    assert not page.navigation_summary.isVisible()
    assert page.camera_panel._show_status
    assert page.view_button.text() == "地图全屏"
    assert np.array_equal(page.map_panel._path, route)
    assert page.camera_panel._pixmap.cacheKey() == image
    assert not exits and not starts
    page.close()


@pytest.fixture
def battery_node():
    requests, removed, errors = [], [], []
    def call(request):
        requests.append(request)
        return Future()
    node = SimpleNamespace(_battery_request=None, _battery_type=Battery,
        _battery_client=SimpleNamespace(service_is_ready=lambda: True, call_async=call,
                                       remove_pending_request=removed.append),
        battery_percentage=None, battery_received_at=None, battery_charge_state=0,
        battery_error="等待底盘电量", get_logger=lambda: SimpleNamespace(error=errors.append))
    return node, requests, removed, errors


def test_battery_reads_percentage_and_charge_without_movement_clients(battery_node):
    node, requests, removed, errors = battery_node
    QtNavRosNode._poll_battery(node)
    assert len(requests) == 1 and isinstance(requests[0], Battery.Request)
    QtNavRosNode._poll_battery(node)
    assert len(requests) == 1
    reply = Battery.Response()
    reply.info.power = 36
    reply.info.charge_state.value = 2
    node._battery_request.set_result(reply)
    QtNavRosNode._poll_battery(node)
    assert node.battery_percentage == 36 and node.battery_charge_state == 2
    assert time.monotonic() - node.battery_received_at < 1.
    assert node.battery_error == "" and not errors


def test_battery_timeout_and_disconnection_do_not_fabricate_values(battery_node):
    node, requests, removed, errors = battery_node
    QtNavRosNode._poll_battery(node)
    future = node._battery_request
    node._battery_request_at -= 3.
    QtNavRosNode._poll_battery(node)
    assert removed == [future] and "超时" in node.battery_error
    assert node.battery_percentage is None
    node._battery_request = None
    node._battery_client.service_is_ready = lambda: False
    QtNavRosNode._poll_battery(node)
    assert "未连接" in node.battery_error and node.battery_received_at is None


@pytest.mark.parametrize("power,charge", [(101, 1), (36, 9)])
def test_invalid_external_battery_data_is_reported(battery_node, power, charge):
    node, requests, removed, errors = battery_node
    QtNavRosNode._poll_battery(node)
    reply = Battery.Response()
    reply.info.power = power
    reply.info.charge_state.value = charge
    node._battery_request.set_result(reply)
    QtNavRosNode._poll_battery(node)
    assert node.battery_percentage is None and "无效" in node.battery_error
    assert errors == [node.battery_error]


@pytest.mark.parametrize("size", [(800, 480), (480, 800), (1600, 900)])
def test_home_battery_is_live_charging_or_expired(tmp_path, monkeypatch, application, size):
    for name in ("NAV_ACCESS_SETTINGS", "NAV_TASK_STATE_DIR", "NAV_RUNTIME_DIR", "NAV_LOG_DIR"):
        monkeypatch.setenv(name, str(tmp_path / name))
    node = SimpleNamespace(robot_profile="woosh", mapping_backend="woosh_native5",
        navigation_status="等待准备", map_storage_dir=str(tmp_path / "maps"),
        battery_percentage=36, battery_charge_state=1,
        battery_received_at=time.monotonic(), battery_error="")
    view = NavigationWindow(node)
    view.refresh_timer.stop(); view.operations_page.timer.stop()
    view.resize(*size); view.show(); application.processEvents()
    label = view.system_home_page.battery
    assert label.isVisible() and label.text() == "底盘电量 36% · 未充电"
    home = view.system_home_page
    for widget in (label, home.mapping_button, home.navigation_button, home.maps_button,
                   home.settings_button, home.phone_button, home.device_button):
        assert home.rect().contains(widget.mapTo(home, QPoint(0, 0)))
        assert home.rect().contains(widget.mapTo(home, widget.rect().bottomRight()))
    node.battery_percentage = 18; node.battery_charge_state = 3
    view._refresh_system_home()
    assert label.text() == "底盘电量 18% · 自动充电中"
    assert "#ff737b" in label.styleSheet()
    node.battery_received_at -= 6.
    view._refresh_system_home()
    assert "数据已过期" in label.text() and "上次 18%" in label.text()
    assert "底盘电量 --" in label.text()
    node.battery_received_at = None; node.battery_percentage = None
    node.battery_error = "底盘电量服务未连接"
    view._refresh_system_home()
    assert label.text() == "底盘电量 -- · 底盘电量服务未连接"
    view.close(); application.processEvents()
    view.operations_page.task_coordinator.thread.join(timeout=5)

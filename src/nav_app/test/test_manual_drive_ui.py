"""All page motion tests inject an in-memory transport, never a robot."""
import time
from types import SimpleNamespace
import pytest
from PyQt5.QtCore import QEvent, QPoint, QPointF, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtGui import QTouchDevice, QTouchEvent
from test_height_preview import window
from test_manual_drive import Transport
from nav_app.features.manual_drive import ZERO
from nav_app.ui import manual_drive


@pytest.fixture(autouse=True)
def isolated_profiles(tmp_path, monkeypatch):
    monkeypatch.setenv("NAV_GAMEPAD_PROFILES", str(tmp_path/"gamepads.json"))
    monkeypatch.setattr(manual_drive.GamepadPanel, "scan", lambda self: None)


@pytest.fixture
def page(window, monkeypatch, application):
    transport = Transport()
    monkeypatch.setattr(manual_drive, "WooshTeleopRos", lambda node: transport)
    window.show()
    window.show_manual_drive()
    application.processEvents()
    page = window.manual_drive_page
    page.timer.stop()
    page.chassis.setCurrentIndex(page.chassis.findData("ackermann"))
    assert page.connect_agent()
    page.tabs.setCurrentIndex(0)
    page.tick()
    return page, transport


def test_entry_routes_and_connection_are_read_only(window, monkeypatch, application):
    transport = Transport()
    monkeypatch.setattr(manual_drive, "WooshTeleopRos", lambda node: transport)
    window.system_home_page.drive_button.click()
    page = window.manual_drive_page
    assert window.pages.currentWidget() is page
    assert page.tabs.currentIndex() == 2
    page.connect_button.click()
    page.tick()
    assert not hasattr(page, "manual_button")
    assert "自动" in page.link_status.text()
    assert not page.policy.armed and transport.commands == []
    page.header.back_button.click()
    window.system_settings_page.buttons["manual_drive"].click()
    assert window.pages.currentWidget() is page
    assert transport.commands == []


def test_virtual_stick_motion_release_and_page_exit(page, window, application):
    view, transport = page
    view.allow.setChecked(True)
    assert view.policy.armed
    stick = view.stick
    point = QPoint(stick.width()//2, int(stick.height()*.15))
    QTest.mousePress(stick, Qt.LeftButton, pos=point)
    view.policy.last_send = -1  # Advance past the 20 Hz command interval.
    view.tick()
    assert transport.commands[-1][0] > 0
    assert transport.commands[-1][1] == 0.
    assert abs(transport.commands[-1][2]) < .003  # Half-pixel rounding at the stick center.
    QTest.mouseRelease(stick, Qt.LeftButton, pos=point)
    assert transport.commands[-1] == ZERO
    view.tick()
    QTest.mousePress(stick, Qt.LeftButton, pos=point)
    view.policy.last_send = -1
    view.tick()
    assert transport.commands[-1][0] > 0
    window.show_system_home()
    assert not view.policy.armed and transport.commands[-1] == ZERO
    QTest.mouseRelease(stick, Qt.LeftButton, pos=point)


def test_focus_loss_and_navigation_block_manual_drive(page, window, application):
    view, transport = page
    view.allow.setChecked(True)
    view.policy.screen_input(0, 1, True)
    view.tick()
    application.sendEvent(window, QEvent(QEvent.WindowDeactivate))
    assert not view.policy.armed and transport.commands[-1] == ZERO
    window.navigation_stack_controller.overall_state = "running"
    view.tick()
    assert not view.allow.isEnabled()
    view.allow.setChecked(True)
    assert not view.policy.armed
    window.navigation_stack_controller.overall_state = "stopped"


def attach_gamepad(view, window, monkeypatch):
    pad = view.gamepad
    monkeypatch.setattr(pad, "scan", lambda: None)
    monkeypatch.setattr(window.node, "create_subscription", lambda *args: object(), raising=False)
    monkeypatch.setattr(window.node, "destroy_subscription", lambda sub: None, raising=False)
    calls = []
    monkeypatch.setattr(pad.driver, "start_launch", lambda args: calls.append(args))
    view.tabs.setCurrentIndex(1)
    pad.scan_timer.stop()
    pad.set_devices([dict(id=3, guid="a"*32, name="Mock Xbox")])
    return pad, calls


def test_gamepad_discovery_requires_calibration_and_uses_private_input(page, window, monkeypatch):
    view, transport = page
    pad, calls = attach_gamepad(view, window, monkeypatch)
    assert "device_id:=3" in calls[0]
    assert "joy:=/nav/manual_gamepad/joy" in calls[0]
    assert "autorepeat_rate:=20.0" in calls[0]
    view.tick()
    assert not view.allow.isEnabled() and transport.commands == []
    assert "新手柄" in pad.status.text()


def test_calibration_saves_reloads_new_device_number_and_never_moves(page, window, monkeypatch):
    view, transport = page
    pad, calls = attach_gamepad(view, window, monkeypatch)
    def frame(axes=(0., 0., 0., 0.), buttons=(0, 0, 0)):
        pad.receive(SimpleNamespace(axes=axes, buttons=buttons))
        view.tick()
    frame()
    pad.calibrate_button.click()
    pad.center_button.click()
    frame(buttons=(0, 0, 1))
    frame()
    for axes in [(0., -1., 0., 0.), (1., 0., 0., 0.)]:
        frame(axes)
        pad.calibration.candidate_at -= .25
        frame(axes)
        frame()
    assert pad.calibration.stage == "done"
    assert not view.allow.isEnabled() and transport.commands == []
    pad.save_button.click()
    assert pad.path.exists() and pad.ready
    assert view.policy.axes == (1, -1, 0)
    assert view.policy.signs == (-1, 1, 1) and view.policy.deadman == 2
    pad.set_devices([dict(id=8, guid="a"*32, name="Mock Xbox")])
    assert "device_id:=8" in calls[-1]
    assert pad.ready and not view.policy.armed and not view.allow.isChecked()
    frame()
    view.allow.setChecked(True)
    assert view.policy.armed
    frame((.5, -1., 0., 0.), (0, 0, 1))
    view.policy.last_send = -1
    view.tick()
    assert transport.commands[-1] == pytest.approx((.15, 0., .15))
    pad.set_devices([])
    assert not view.policy.armed and transport.commands[-1] == ZERO


@pytest.mark.parametrize("size", [(800, 480), (480, 800), (1280, 900)])
def test_controls_fit_target_viewports(window, application, size):
    window.resize(*size)
    window.show()
    application.processEvents()
    home = window.system_home_page
    assert home.rect().contains(home.drive_button.mapTo(home, home.drive_button.rect().bottomRight()))
    window.show_manual_drive()
    page = window.manual_drive_page
    for index, controls in [(0, [page.stick, page.horizontal]), (1, [page.gamepad.devices, page.gamepad.refresh_button]),
                            (2, [page.ip, page.start_agent_button, page.connect_button])]:
        page.tabs.setCurrentIndex(index)
        application.processEvents()
        for control in [page.chassis, page.linear, page.angular, page.allow, page.stop_button, *controls]:
            assert control.isVisible()
            assert page.rect().contains(control.mapTo(page, QPoint(0, 0)))
            assert page.rect().contains(control.mapTo(page, control.rect().bottomRight()))
    assert (window.width(), window.height()) == size


def test_single_touch_and_touch_cancel_request_zero(page, application):
    view, transport = page
    view.allow.setChecked(True)
    device = QTouchDevice()
    device.setType(QTouchDevice.TouchScreen)
    device.setCapabilities(QTouchDevice.Position)
    stick = view.stick
    point = QPoint(stick.width()//2, int(stick.height()*.15))
    # PyQt5 cannot construct TouchPoint or register a QTest touch device.
    begin = QEvent(QEvent.TouchBegin)
    begin.touchPoints = lambda: [SimpleNamespace(pos=lambda: QPointF(point))]
    assert stick.event(begin)
    assert stick.held
    view.policy.last_send = -1
    view.tick()
    assert transport.commands[-1][0] > 0
    application.sendEvent(stick, QTouchEvent(QEvent.TouchCancel, device))
    assert not stick.held and transport.commands[-1] == ZERO


def test_calibration_keeps_button_press_and_release_between_ui_refreshes(page, window, monkeypatch):
    view, transport = page
    pad, _calls = attach_gamepad(view, window, monkeypatch)
    pad.receive(SimpleNamespace(axes=[0., 0., 0., 0.], buttons=[0, 0, 0]))
    pad.begin()
    pad.record_center()
    pad.receive(SimpleNamespace(axes=[0., 0., 0., 0.], buttons=[0, 1, 0]))
    pad.receive(SimpleNamespace(axes=[0., 0., 0., 0.], buttons=[0, 0, 0]))
    assert pad.calibration.deadman == 1
    assert pad.calibration.stage == "axis"
    view.tick()
    assert "最近按下 [1]" in pad.live.text()
    assert transport.commands == [] and not view.policy.armed

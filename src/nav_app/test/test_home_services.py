"""Home shows resource owners and never silently stops a service to start mapping."""
import pytest
from PyQt5.QtCore import QPoint
from nav_app.ros_launch_process import RosLaunchProcess
from test_height_preview import window


def test_navigation_blocks_mapping_on_home_and_error_remains_visible(window, monkeypatch):
    window.navigation_stack_controller.overall_state = "running"
    starts = []
    monkeypatch.setattr(window.mapping_page, "start_mapping", lambda: starts.append(True))
    home = window.system_home_page
    home.mapping_button.click()
    assert not starts
    assert window.pages.currentWidget() is home
    assert "停止导航" in home.status.text()
    assert "运行中" in home.navigation_state.text()
    assert home.navigation_stop_button.isEnabled()
    window._refresh_system_home()
    assert "停止导航" in home.status.text()
    window.navigation_stack_controller.overall_state = "stopped"


def test_background_job_is_visible_and_failed_mapping_has_pause_entry(window, monkeypatch):
    coordinator = window.operations_page.task_coordinator
    job = dict(id="busy-job", kind="navigation", payload=dict(title="后台地图"), state="running",
               created=1., updated=1., attempt=1, progress={})
    monkeypatch.setattr(coordinator, "snapshot", lambda: dict(jobs=[job], loaded=True, foreground=None))
    monkeypatch.setattr(coordinator, "reserve_idle", lambda owner: False)
    window.start_mapping_from_home()
    home = window.system_home_page
    assert window.pages.currentWidget() is home
    assert "后台地图任务" in home.status.text()
    assert "查看/暂停" in home.status.text()
    assert "正在处理" in home.task_state.text()
    commands = []
    monkeypatch.setattr(coordinator, "_put", lambda *command: commands.append(command))
    home.task_button.click()
    assert window.operations_page.map_tasks.selected()["id"] == "busy-job"
    window.operations_page.map_tasks.pause.click()
    assert commands == [("cancel", "busy-job")]
    assert window.pages.currentWidget() is window.map_workspace_page
    assert window.map_pages.currentWidget() is window.operations_page.map_tasks


def test_residual_navigation_process_is_visible_even_when_controller_reports_error(window, monkeypatch):
    process = next(iter(window.navigation_stack_controller.processes.values()))
    monkeypatch.setattr(RosLaunchProcess, "active", property(lambda value: value is process))
    window.navigation_stack_controller.overall_state = "error"
    window._refresh_system_home()
    assert "残留组件" in window.system_home_page.navigation_state.text()
    assert window.system_home_page.navigation_stop_button.isEnabled()
    window.start_mapping_from_home()
    assert "停止导航" in window.system_home_page.status.text()


@pytest.mark.parametrize("service", ["navigation", "mapping"])
def test_stop_entry_requires_confirmation_and_only_stops_selected_service(window, monkeypatch, service):
    calls, confirmations = [], []
    monkeypatch.setattr(window.navigation_stack_controller, "stop", lambda: calls.append("navigation"))
    monkeypatch.setattr(window.mapping_controller, "stop", lambda: calls.append("mapping"))
    monkeypatch.setattr(window.operations_page, "confirm", lambda text, callback, title: confirmations.append((text,callback,title)))
    getattr(window, "stop_home_"+service)()
    assert calls == []
    assert len(confirmations) == 1
    confirmations[0][1]()
    assert calls == [service]


def test_saving_map_disables_direct_stop(window, monkeypatch):
    window.mapping_page.process_state = "running"
    monkeypatch.setattr(window.mapping_page, "_finishing_attempt", object())
    window._refresh_system_home()
    assert not window.system_home_page.mapping_stop_button.isEnabled()
    assert "正在保存" in window.system_home_page.mapping_state.text()


@pytest.mark.parametrize("size", [(800,480),(480,800),(1280,900)])
def test_service_status_and_actions_fit_on_home(window, application, size):
    window.resize(*size)
    window.show_system_home()
    window.show()
    application.processEvents()
    home = window.system_home_page
    for widget in (home.mapping_state,home.navigation_state,home.task_state,
                   home.mapping_stop_button,home.navigation_stop_button,home.task_button,
                   home.mapping_settings_button,home.mapping_button,home.navigation_button,home.maps_button,home.settings_button):
        assert widget.isVisible()
        assert home.rect().contains(widget.mapTo(home,QPoint(0,0)))
        assert home.rect().contains(widget.mapTo(home,widget.rect().bottomRight()))
    assert window.size().width() == size[0] and window.size().height() == size[1]


def test_failed_input_start_keeps_error_visible_and_configuration_reachable(window, monkeypatch):
    monkeypatch.setattr(window.mapping_page, "start_mapping", lambda: (False,"雷达输入不在线"))
    window.start_mapping_from_home()
    assert window.pages.currentWidget() is window.system_home_page
    assert "雷达输入不在线" in window.system_home_page.status.text()
    window.system_home_page.mapping_settings_button.click()
    assert window.pages.currentWidget() is window.mapping_page


def test_map_parameters_scroll_independently_from_cloud_gestures(window, application):
    from PyQt5.QtWidgets import QScrollArea
    window.resize(800,480)
    window.show_map_tools()
    window.show()
    application.processEvents()
    scroll = window.map_tools_page.findChild(QScrollArea, "transparentScroll")
    assert scroll.property("gsTouchScroll")
    assert scroll.viewport().property("gsTouchScroll")
    assert scroll.verticalScrollBar().maximum() > 0
    azimuth = window.editor_map_panel._cloud_azimuth
    scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
    application.processEvents()
    assert window.editor_map_panel._cloud_azimuth == azimuth

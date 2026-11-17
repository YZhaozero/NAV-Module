"""Visible camera orientation, bounded log work and asynchronous planning status."""
import numpy as np
import pytest
from types import SimpleNamespace as NS
from nav_app.qt_nav_node import CameraPanel, NavigationWindow, QtNavRosNode


@pytest.fixture
def view(tmp_path, monkeypatch, application):
    monkeypatch.setenv("NAV_ACCESS_SETTINGS", str(tmp_path/"access.ini"))
    monkeypatch.setenv("NAV_TASK_STATE_DIR", str(tmp_path/"tasks"))
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path/"runtime"))
    monkeypatch.setenv("NAV_LOG_DIR", str(tmp_path/"logs"))
    window = NavigationWindow(NS(robot_profile="woosh", navigation_status="ready",
        map_storage_dir=str(tmp_path/"maps"), mapping_backend="woosh_native5"))
    window.refresh_timer.stop();window.operations_page.timer.stop()
    window.resize(1600,900);window.show();application.processEvents()
    yield window
    window.close();application.processEvents()
    window.operations_page.task_coordinator.thread.join(timeout=5)


def test_camera_turns_display_only_without_changing_source_pixels(application, view):
    frame = np.zeros((40,60,3),dtype=np.uint8)
    frame[:20,:30] = [255,0,0]
    frame[20:,30:] = [0,0,255]
    original = frame.copy()
    panel = CameraPanel(show_status=False,compact=True,rotate_180=True)
    panel.resize(300,200);panel.set_frame(frame,None,0,1);panel.show();application.processEvents()
    image = panel.grab().toImage()
    assert image.pixelColor(75,50).red() == 255
    assert image.pixelColor(225,150).blue() == 255
    assert panel._pixmap.toImage().pixelColor(5,5).blue() == 255
    np.testing.assert_array_equal(frame,original)
    assert view.camera_panel._rotate_180
    assert view.active_page.camera_panel._rotate_180
    assert view.start_button.text() == "规划路线"
    panel.close()


def test_log_burst_does_no_hidden_text_work_and_entry_does_not_replay(view, application):
    page=view.navigation_stack_page
    changed=[]
    page.log_views["all"].textChanged.connect(lambda:changed.append(True))
    for i in range(2000):
        view.navigation_stack_controller.record_external_log("nav2",f"sample {i}\nready")
    application.processEvents()
    assert not changed
    view.show_navigation_stack()
    assert len(changed) == 1
    assert page.log_views["all"].document().blockCount() <= 3000
    assert "sample 1999" in page.log_views["all"].toPlainText()
    assert "sample 0\n" not in page.log_views["all"].toPlainText()
    page.activate()
    assert len(changed) == 1
    page.log_tabs.setCurrentWidget(page.log_views["nav2"])
    assert "sample 1999" in page.log_views["nav2"].toPlainText()
    page._clear_logs();page.activate()
    assert page.log_views["all"].toPlainText() == ""
    assert page.log_views["nav2"].toPlainText() == ""


def test_pending_status_recovers_without_sending_another_goal():
    ready=[False]
    node=NS(_route_wait_status="old wait",navigation_status="old wait",
        poll_navigation=lambda:None,navigation_route_ready=lambda:ready[0],
        navigation_readiness_reason="等待 Nav2 状态回执：路线规划",navigation_active=False)
    QtNavRosNode._monitor_localization(node)
    assert node.navigation_status == node.navigation_readiness_reason
    ready[0]=True
    QtNavRosNode._monitor_localization(node)
    assert "已就绪" in node.navigation_status and node._route_wait_status is None
    node._route_wait_status="old wait";node.navigation_status="地图定位尚未有效"
    QtNavRosNode._monitor_localization(node)
    assert node.navigation_status == "地图定位尚未有效"



def test_late_plan_topic_cannot_erase_the_planning_preview():
    from nav_msgs.msg import Path
    points=np.array([[0.,0.,0.],[1.,0.,0.]],dtype=np.float32)
    node=NS(navigation_active=False,walking=NS(phase="preview"),path=points,
        clear_navigation_path=lambda:pytest.fail("planning preview was cleared"))
    QtNavRosNode._on_path(node,Path(),"/plan")
    np.testing.assert_array_equal(node.path,points)

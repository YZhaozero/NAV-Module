"""Route preview, explicit manual navigation and a persistent arrival notice."""
from types import SimpleNamespace
import numpy as np
import pytest
from PyQt5.QtTest import QTest
from nav_app.qt_nav_node import NavigationWindow, QtNavRosNode
from nav_app.map_processing import GridMap
from nav_app.features.walking_guidance import WalkingGuidance
from test_walking_guidance import Client, Goal, path_reply


@pytest.fixture
def walking_ui(tmp_path, monkeypatch, application):
    for key in ("NAV_ACCESS_SETTINGS","NAV_TASK_STATE_DIR","NAV_RUNTIME_DIR","NAV_LOG_DIR"):
        monkeypatch.setenv(key,str(tmp_path/key))
    clock = [0.]
    node = SimpleNamespace(robot_profile="woosh",operating_mode="walking",
        navigation_status="ready",map_storage_dir=str(tmp_path/"maps"),
        mapping_backend="woosh_native5",map_frame="map",path_frame="map",
        latest_image=np.zeros((16,24,3),dtype=np.uint8),image_revision=1,
        latest_pointcloud=None,navigation_active=False,navigation_result_revision=0,
        navigation_result_status=None,pose=np.array([0.,0.]))
    motion_calls = []
    node.robot = SimpleNamespace(busy=False,start=lambda goal: motion_calls.append(goal))
    client = Client()
    session = WalkingGuidance(client,lambda session: QtNavRosNode._walking_changed(node,session),clock=lambda: clock[0])
    node.walking = session
    node.path = session.path
    node.send_navigation_waypoints = lambda points: session.plan(object())
    node.navigation_route_ready = lambda: True
    node.walking_pose = lambda: (node.pose,0.)
    node.begin_guidance = lambda: QtNavRosNode.begin_guidance(node)
    node.clear_navigation_path = lambda: QtNavRosNode.clear_navigation_path(node)
    node.cancel_navigation = lambda: QtNavRosNode.cancel_navigation(node)
    node.map_navigation_state = lambda: (node.path[:,:2],node.pose,0.)
    node.projected_route = lambda shape: None
    node.remaining_distance = lambda: session.remaining
    view = NavigationWindow(node)
    access = view.navigation_access_page
    backend = access.fields["navigation_engine"]
    backend.setCurrentIndex(backend.findData("nav2"))
    access.save()
    view.refresh_timer.stop();view.operations_page.timer.stop()
    view._set_local_grid(GridMap(np.zeros((40,40),dtype=np.int8),.2,np.zeros(3)),reset_original=True)
    yield view,client,clock,motion_calls
    view.close();application.processEvents()
    view.operations_page.task_coordinator.thread.join(timeout=5)


def test_woosh_manual_route_opens_navigation_page_and_only_starts_after_click(walking_ui):
    view,client,clock,motion_calls = walking_ui
    view.on_map_point(5.,0.)
    view.start_navigation()
    assert view.pages.currentWidget() is view.active_page
    assert view.node.walking.phase == "planning"
    assert motion_calls == []
    goal = Goal();client.requests[0].complete(goal)
    goal.result.complete(path_reply([(0.,0.),(5.,0.)]))
    view.refresh()
    assert view.node.walking.phase == "preview"
    assert not view.node.navigation_active
    assert not view.active_page.begin_button.isHidden()
    assert view.active_page.begin_button.text() == "开始导航"
    assert view.active_page.result_overlay.isHidden()
    assert "开始导航" in view.active_page.camera_panel.projection_status
    view.active_page.begin_button.click()
    assert view.node.walking.phase == "guiding"
    assert view.node.navigation_active
    assert view.pages.currentWidget() is view.active_page
    assert view.active_page.begin_button.isHidden()
    assert len(client.requests) == 1 and motion_calls == []


def test_manual_arrival_is_visible_until_return_and_cleans_up_routes(walking_ui):
    view,client,clock,motion_calls = walking_ui
    view.on_map_point(5.,0.);view.start_navigation()
    goal=Goal();client.requests[0].complete(goal)
    goal.result.complete(path_reply([(0.,0.),(5.,0.)]))
    view.refresh();view.active_page.begin_button.click()
    for x in (1.,2.,3.,4.,4.5):
        view.node.pose=np.array([x,0.])
        view.node.walking.update_pose(view.node.pose,0.)
        view._last_map_navigation_refresh=0.;view.refresh()
    assert view.node.walking.phase=="guiding"
    clock[0]=1.2
    view.node.walking.update_pose(view.node.pose,0.)
    view.refresh()
    assert view.node.walking.phase=="arrived"
    assert view.pending_navigation_result_status==4
    assert "已到达目的地" in view.active_page.result_overlay.text()
    assert not view.active_page.result_overlay.isHidden()
    assert view.map_panel._path is None and view.map_panel._motion_trace is None
    QTest.qWait(2400)
    assert view.pages.currentWidget() is view.active_page
    assert not view.active_page.result_overlay.isHidden()
    view.active_page.exit_button.click()
    assert view.pages.currentWidget() is view.setup_page
    assert view.waypoints==[] and view._motion_trace==[]
    assert motion_calls==[]


def test_begin_failure_is_visible_on_navigation_page(walking_ui):
    view,client,clock,motion_calls=walking_ui
    view.on_map_point(5.,0.);view.start_navigation()
    goal=Goal();client.requests[0].complete(goal)
    goal.result.complete(path_reply([(0.,0.),(5.,0.)]))
    view.node.pose=None
    view.refresh();view.active_page.begin_button.click();view.refresh()
    assert view.node.walking.phase=="preview"
    assert "当前位置未确认" in view.active_page.camera_panel.projection_status
    assert not view.active_page.begin_button.isHidden()
    assert motion_calls==[]


def test_hand_guidance_keeps_plan_while_localization_recovers(walking_ui):
    from nav_msgs.msg import Path
    view, client, clock, motion_calls = walking_ui
    node = view.node
    view.on_map_point(5., 0.)
    view.start_navigation()
    goal = Goal()
    client.requests[0].complete(goal)
    goal.result.complete(path_reply([(0., 0.), (5., 0.)]))
    view.active_page.begin_button.click()
    node.require_localizer_quality = True
    node._route_wait_status = None
    node.poll_navigation = lambda: node.walking.update_pose(*node.walking_pose())
    node.pose = None
    QtNavRosNode._monitor_localization(node)
    QtNavRosNode._on_path(node, Path(), "/plan")
    view.refresh()
    assert node.walking.phase == "lost" and view.active_page.map_panel._path is not None
    clock[0] += 3.
    node.pose = np.array([1., 0.])
    QtNavRosNode._monitor_localization(node)
    assert node.walking.phase == "guiding" and node.walking.remaining == 4.
    assert motion_calls == []

"""Ending a mission removes transient overlays without erasing the map or live pose."""
from types import SimpleNamespace
import numpy as np
import pytest
from nav_app.map_processing import GridMap
from test_height_preview import window


@pytest.fixture
def navigation_scene(window):
    node = window.node
    node.latest_image = np.zeros((8,12,3), dtype=np.uint8)
    node.image_revision = 1
    node.navigation_active = True
    node.navigation_result_revision = 0
    node.navigation_result_status = None
    node.walking = SimpleNamespace(phase="guiding")
    node.pose = np.array([1.,2.])
    node.route = np.array([[0.,0.],[1.,2.],[2.,3.]])
    node.map_navigation_state = lambda: (node.route,node.pose,0.)
    node.projected_route = lambda shape: None
    node.remaining_distance = lambda: 0.
    node.cancel_navigation = lambda: setattr(node,"navigation_active",False)
    window._set_local_grid(GridMap(np.zeros((12,16),dtype=np.int8),.1,np.zeros(3)), reset_original=True)
    window.on_map_point(2.,3.)
    window.start_navigation()
    window._motion_trace = [np.array([0.,0.]),np.array([1.,2.])]
    window._cached_route = np.array([[1.,1.],[2.,2.]])
    for panel in (window.map_panel,window.active_page.map_panel):
        panel.set_navigation_state(node.route,node.pose,0.)
        panel.set_motion_trace(np.array(window._motion_trace))
    window.active_page.camera_panel.set_frame(node.latest_image,window._cached_route,3.)
    return window


@pytest.mark.parametrize("status",[4,5,6])
@pytest.mark.parametrize("page",["setup","active"])
def test_terminal_result_clears_both_map_views_and_refresh_cannot_restore_old_lines(navigation_scene,status,page):
    view = navigation_scene
    before = view.local_grid.occupancy.copy()
    if page == "setup":view.show_navigation_setup()
    view.node.navigation_active = False
    view.node.walking.phase = "arrived" if status==4 else "idle"
    view.node.navigation_result_status = status
    view.node.navigation_result_revision += 1
    view._handle_navigation_terminal_state()
    if page == "active":
        assert not view.active_page.result_overlay.isHidden()
        view._finish_navigation_return()
    for index in range(3):
        view.node.pose = np.array([5.+index,4.])
        view._last_map_navigation_refresh = 0.
        view.refresh()
        for panel in (view.map_panel,view.active_page.map_panel):
            assert panel._path is None
            assert panel._motion_trace is None
            assert np.array_equal(panel._robot_xy,view.node.pose)
        assert view._motion_trace == []
    assert view._cached_route is None
    assert view.active_page.camera_panel._route is None
    assert np.array_equal(view.local_grid.occupancy,before)


def test_manual_exit_clears_route_trace_and_ar_while_cancellation_is_pending(navigation_scene):
    view = navigation_scene
    # Keep the stale source route, as a delayed publisher/callback can do.
    view.exit_navigation()
    view.node.walking.phase = "preview"
    view._last_map_navigation_refresh = 0.
    view.refresh()
    assert view.pages.currentWidget() is view.setup_page
    assert view.waypoints == []
    assert view._motion_trace == []
    assert view.map_panel._path is None
    assert view.active_page.map_panel._motion_trace is None
    assert view.active_page.camera_panel._route is None


@pytest.mark.parametrize("state",["stopped","error"])
def test_stopping_or_failing_stack_removes_overlays_even_with_stale_active_flag(navigation_scene,state):
    view = navigation_scene
    view._navigation_stack_status_changed(state)
    view._last_map_navigation_refresh = 0.
    view.refresh()
    assert view.node.navigation_active  # Display cleanup does not issue a new robot command.
    assert view._motion_trace == []
    assert view.map_panel._path is None
    assert view.active_page.map_panel._motion_trace is None


def test_new_session_preview_and_tracking_start_without_previous_trace(navigation_scene):
    view = navigation_scene
    view.exit_navigation()
    view.node.navigation_active = False
    view.node.walking.phase = "preview"
    view.node.route = np.array([[20.,10.],[21.,10.]])
    view.on_map_point(21.,10.)
    view.start_navigation()
    for position in ([20.,10.],[21.,10.]):
        view.node.pose = np.array(position)
        view._last_map_navigation_refresh = 0.
        view.refresh()
    assert np.array_equal(view.map_panel._path,view.node.route)
    assert np.array_equal(np.array(view._motion_trace),view.node.route)
    assert np.array_equal(view.active_page.map_panel._motion_trace,view.node.route)


def test_rejected_second_start_keeps_the_existing_mission_display(navigation_scene):
    view = navigation_scene
    trace = np.array(view._motion_trace)
    view.node.send_navigation_waypoints = lambda waypoints: False
    view.start_navigation()
    assert view._navigation_overlay_enabled
    assert np.array_equal(view.map_panel._path,view.node.route)
    assert np.array_equal(view._motion_trace,trace)

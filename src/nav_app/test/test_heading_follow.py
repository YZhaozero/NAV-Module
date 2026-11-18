"""Screen-space navigation checks; all poses/maps here are synthetic."""
import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication
from nav_app.qt_nav_node import MapPanel


@pytest.fixture
def panel():
    app = QApplication.instance() or QApplication([])
    view = MapPanel()
    view.resize(800, 480)
    view.set_map(np.zeros((100, 160), dtype=np.int8), np.array([-4., -3.]), .1, .37)
    yield view
    view.close()
    app.processEvents()


@pytest.mark.parametrize("yaw", [0., math.pi/2, math.pi, -math.pi/2, 3.13, -3.13])
def test_follow_places_forward_above_device_and_picks_original_coordinates(panel, yaw):
    pose = np.array([1., 2.])
    panel.set_navigation_follow(True)
    panel.set_navigation_state(None, pose, yaw)
    center = panel.world_to_widget(pose)
    ahead = panel.world_to_widget(pose + np.array([math.cos(yaw), math.sin(yaw)]))
    assert center.x() == pytest.approx(400.)
    assert center.y() == pytest.approx(288.)
    assert ahead.x() == pytest.approx(center.x(), abs=1e-5)
    assert ahead.y() < center.y()
    from types import SimpleNamespace
    arrows = []
    panel._draw_robot(SimpleNamespace(setPen=lambda _: None, setBrush=lambda _: None,
                                      drawPolygon=arrows.append))
    tip = arrows[0][0]
    assert tip.x() == pytest.approx(center.x())
    assert tip.y() < center.y()
    np.testing.assert_allclose(panel.widget_to_world(ahead),
                               pose + [math.cos(yaw), math.sin(yaw)], atol=1e-6)


def test_lost_pose_cannot_retain_a_trusted_arrow(panel):
    panel.set_navigation_follow(True)
    panel.set_navigation_state(None, np.array([1., 2.]), 1.)
    viewport = panel._map_follow_transform()
    assert not viewport.isIdentity()
    panel.set_navigation_state(None, None, 0.)
    assert panel._robot_xy is None
    assert panel._map_follow_transform() == viewport
    panel.set_navigation_state(None, np.array([1., np.nan]), 0.)
    assert panel._robot_xy is None
    panel.set_navigation_state(None, np.array([1., 2.]), float('inf'))
    assert panel._robot_xy is None


def test_manual_pan_and_edit_release_follow_and_button_can_restore_it(panel):
    panel.set_navigation_state(None, np.array([1., 2.]), .8)
    panel.set_navigation_follow(True)
    panel._pan_map_view(12., 20.)
    assert not panel.navigation_follow_enabled
    panel.set_navigation_follow(True)
    panel.set_edit_tool('erase', .2)
    assert not panel.navigation_follow_enabled
    panel.set_navigation_follow(True)
    assert not panel.navigation_follow_enabled
    panel.set_edit_tool('navigate', .2)
    panel.set_navigation_follow(True)
    assert panel.navigation_follow_enabled
    panel.fit_view()
    assert not panel.navigation_follow_enabled


def test_cloud_turn_and_translation_follow_the_same_world_heading():
    app = QApplication.instance() or QApplication([])
    view = MapPanel(cloud_3d=True)
    view.resize(800, 480)
    view.set_cloud_display_options('original', 'XYZ', (False, False, False),
                                   'perspective', 'height')
    view.set_pointcloud(np.array([[-5.,-5.,0.], [5.,5.,0.], [5.,-5.,1.]]))
    view.set_display_mode('cloud')
    view.set_navigation_follow(True)
    try:
        for pose, yaw in [(np.array([1.,2.]),0.), (np.array([2.,3.]),math.pi/2)]:
            view.set_navigation_state(None, pose, yaw)
            direction = np.array([math.cos(yaw), math.sin(yaw)])
            ground = view._cloud_ground_points_from_xy(np.array([pose, pose+direction]))
            x,y,_ = view._project_cloud_points(ground)
            assert x[0] == pytest.approx(400., abs=1e-4)
            assert x[1] == pytest.approx(x[0], abs=1e-4)
            assert y[1] < y[0]
        view._orbit_cloud_view(30., 0.)
        assert not view.navigation_follow_enabled
    finally:
        view.close()
        app.processEvents()


def test_cloud_display_reconfiguration_preserves_follow_direction():
    app = QApplication.instance() or QApplication([])
    view = MapPanel(cloud_3d=True); view.resize(800, 480)
    x,y = np.meshgrid(np.linspace(-5,5,20),np.linspace(-5,5,20))
    view.set_pointcloud(np.c_[x.ravel(),y.ravel(),(.2*x+.15*y).ravel()])
    view.set_display_mode('cloud')
    view.set_navigation_state(None,np.array([1.,2.]),.4)
    view.set_navigation_follow(True)
    try:
        for projection in ('perspective', 'orthographic'):
            view.set_cloud_display_options('original','XYZ',(False,False,False),projection,'height')
            xy=np.array([[1.,2.],np.array([1.,2.])+[math.cos(.4),math.sin(.4)]])
            sx,sy,_=view._project_cloud_points(view._cloud_ground_points_from_xy(xy))
            assert sx[0]==pytest.approx(400.,abs=1e-4)
            assert sx[1]==pytest.approx(sx[0],abs=1e-4) and sy[1]<sy[0]
    finally:
        view.close(); app.processEvents()

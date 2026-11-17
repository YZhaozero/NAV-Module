"""Cloud gesture events preserve map data and recover after interrupted input."""
from types import SimpleNamespace
import numpy as np
import pytest
from PyQt5.QtCore import QEvent, QPointF, Qt
from PyQt5.QtGui import QMouseEvent, QTouchEvent
from nav_app.qt_nav_node import MapPanel
from test_height_preview import window


class TouchInput(QTouchEvent):
    """PyQt does not expose construction of QTouchEvent.TouchPoint."""
    def __init__(self, kind, points):
        super().__init__(kind)
        self.points = [SimpleNamespace(
            id=lambda identity=identity: identity,
            pos=lambda position=QPointF(x, y): position,
            state=lambda state=state: state)
            for identity, x, y, state in points]

    def touchPoints(self):
        return self.points


@pytest.fixture
def cloud(application):
    panel = MapPanel(cloud_3d=True)
    panel.resize(800, 480)
    panel.set_pointcloud(np.array([[0, 0, 0], [1, 1, 1], [2, -1, 2]], dtype=np.float32))
    panel.set_display_mode("cloud")
    panel.show()
    application.processEvents()
    yield panel
    panel.close()


def test_single_touch_rotates_and_releases_without_changing_points(cloud):
    points = cloud._cloud_source_points.copy()
    before = cloud._cloud_azimuth
    assert cloud.testAttribute(Qt.WA_AcceptTouchEvents)
    cloud.event(TouchInput(QEvent.TouchBegin, [(0, 200, 200, Qt.TouchPointPressed)]))
    cloud.event(TouchInput(QEvent.TouchUpdate, [(0, 260, 220, Qt.TouchPointMoved)]))
    assert cloud._cloud_azimuth != before
    cloud.event(TouchInput(QEvent.TouchEnd, [(0, 260, 220, Qt.TouchPointReleased)]))
    assert cloud._touch_positions == {}
    assert cloud._view_drag_button == Qt.NoButton
    assert np.array_equal(points, cloud._cloud_source_points)


def test_two_fingers_pan_and_pinch_without_spinning_or_jumping(cloud):
    azimuth = cloud._cloud_azimuth
    target = cloud._cloud_view_target.copy()
    cloud.event(TouchInput(QEvent.TouchBegin, [(0, 200, 200, Qt.TouchPointPressed), (1, 300, 200, Qt.TouchPointPressed)]))
    cloud.event(TouchInput(QEvent.TouchUpdate, [(0, 210, 220, Qt.TouchPointMoved), (1, 330, 220, Qt.TouchPointMoved)]))
    assert cloud._cloud_azimuth == azimuth
    assert not np.array_equal(target, cloud._cloud_view_target)
    assert cloud._cloud_zoom > 1.
    cloud.event(TouchInput(QEvent.TouchUpdate, [(0, 210, 220, Qt.TouchPointStationary), (1, 330, 220, Qt.TouchPointReleased)]))
    assert cloud._cloud_azimuth == azimuth
    cloud.event(TouchInput(QEvent.TouchUpdate, [(0, 230, 220, Qt.TouchPointMoved)]))
    assert cloud._cloud_azimuth != azimuth
    cloud.event(TouchInput(QEvent.TouchEnd, [(0, 230, 220, Qt.TouchPointReleased)]))
    assert cloud._view_drag_button == Qt.NoButton


def test_touch_cancel_clears_drag_without_selecting_a_point(cloud):
    selected = []
    cloud.point_selected.connect(lambda *position: selected.append(position))
    cloud.event(TouchInput(QEvent.TouchBegin, [(0, 200, 200, Qt.TouchPointPressed)]))
    cloud.event(TouchInput(QEvent.TouchCancel, []))
    assert cloud._touch_positions == {}
    assert cloud._view_drag_button == Qt.NoButton
    assert selected == []


def test_quick_second_mouse_drag_does_not_reset_zoom(cloud):
    cloud._cloud_zoom = 2.
    cloud.mouseDoubleClickEvent(QMouseEvent(QEvent.MouseButtonDblClick, QPointF(200,200),
                                          Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    cloud.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, QPointF(250,220),
                                    Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
    cloud.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, QPointF(250,220),
                                       Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
    assert cloud._cloud_zoom == 2.
    assert cloud._view_drag_button == Qt.NoButton


def test_cloud_double_click_still_resets_after_release(cloud):
    cloud._cloud_zoom = 2.
    cloud.mouseDoubleClickEvent(QMouseEvent(QEvent.MouseButtonDblClick, QPointF(200,200),
                                          Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    cloud.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, QPointF(200,200),
                                       Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
    assert cloud._cloud_zoom == 1.


def test_mouse_can_tilt_below_horizontal_and_pan_without_spinning(cloud):
    cloud._orbit_cloud_view(0,100)
    assert cloud._cloud_elevation < 0
    azimuth = cloud._cloud_azimuth
    target = cloud._cloud_view_target.copy()
    cloud.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, QPointF(200,200),
                                     Qt.RightButton, Qt.RightButton, Qt.NoModifier))
    cloud.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, QPointF(230,220),
                                    Qt.NoButton, Qt.RightButton, Qt.NoModifier))
    cloud.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, QPointF(230,220),
                                       Qt.RightButton, Qt.NoButton, Qt.NoModifier))
    assert cloud._cloud_azimuth == azimuth
    assert not np.array_equal(target, cloud._cloud_view_target)


def test_single_point_screen_can_pan_zoom_and_rotate_with_toolbar(window, application):
    from nav_app.map_processing import PointCloudMap
    points = np.array([[0,0,0],[1,1,1],[2,-1,2]], dtype=np.float32)
    window._set_local_cloud(PointCloudMap(points.copy()), reset_original=True)
    window.set_editor_map_mode("cloud")
    window.resize(800,480)
    window.show_map_tools();window.show();application.processEvents()
    panel = window.editor_map_panel
    window.editor_pan_button.click()
    assert panel._edit_tool == "pan"
    assert "平移" in window.editor_gesture_mode.text()
    azimuth = panel._cloud_azimuth
    target = panel._cloud_view_target.copy()
    panel.event(TouchInput(QEvent.TouchBegin, [(0,100,100,Qt.TouchPointPressed)]))
    panel.event(TouchInput(QEvent.TouchUpdate, [(0,130,120,Qt.TouchPointMoved)]))
    panel.event(TouchInput(QEvent.TouchEnd, [(0,130,120,Qt.TouchPointReleased)]))
    assert not np.array_equal(target,panel._cloud_view_target)
    assert panel._cloud_azimuth == azimuth
    zoom = panel._cloud_zoom
    window.editor_zoom_in_button.click()
    assert panel._cloud_zoom > zoom
    window.editor_zoom_out_button.click()
    assert panel._cloud_zoom == pytest.approx(zoom)
    window.editor_rotate_button.click()
    assert panel._edit_tool == "navigate"
    assert "旋转" in window.editor_gesture_mode.text()
    panel.event(TouchInput(QEvent.TouchBegin, [(0,100,100,Qt.TouchPointPressed)]))
    panel.event(TouchInput(QEvent.TouchUpdate, [(0,140,120,Qt.TouchPointMoved)]))
    panel.event(TouchInput(QEvent.TouchEnd, [(0,140,120,Qt.TouchPointReleased)]))
    assert panel._cloud_azimuth != azimuth
    window.editor_fit_button.click()
    assert panel._cloud_zoom == 1.
    assert np.array_equal(window.local_cloud.points, points)
    assert window.width() == 800 and window.height() == 480

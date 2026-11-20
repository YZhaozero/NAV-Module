import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from nav_app.navigation_display import route_guidance, route_chevrons, guidance_state
from nav_app.ar_projection import grounded_route
from nav_app.nav_math import RenderStyle


@pytest.mark.parametrize('yaw,instruction', [(0., '沿路线前行'), (-math.pi/2, '向左转'),
                                             (math.pi/2, '向右转'), (math.pi, '请转身')])
def test_direction_uses_robot_heading(yaw, instruction):
    cue = route_guidance([[0, 0], [10, 0]], [0, 0], yaw)
    assert cue['live'] and cue['instruction'] == instruction
    assert cue['remaining_m'] == 10.


@pytest.mark.parametrize('end,maneuver', [([5, 5], '左转'), ([5, -5], '右转'), ([0, 0], '掉头')])
def test_upcoming_maneuver_uses_route_order(end, maneuver):
    cue = route_guidance([[0, 0], [5, 0], end], [0, 0], 0.)
    assert cue['instruction'] == '前方 5 米' + maneuver
    assert cue['angle'] == pytest.approx(0.)  # The current segment still leads straight.


def test_near_endpoint_is_not_nav2_success():
    cue = route_guidance([[0, 0], [2, 0]], [1.6, 0], 0.)
    assert cue['instruction'] == '接近目的地'
    assert cue['remaining_m'] == pytest.approx(.4)


def test_crossing_route_does_not_jump_to_later_segment():
    cue = route_guidance([[0, 0], [5, 0], [5, 5], [0, 0], [-5, 0]], [.1, 0], 0.)
    assert cue['progress'] == pytest.approx(.1)
    lost = route_guidance([[0, 0], [10, 0]], [2, 2], 0.)
    assert not lost['live'] and lost['remaining_m'] is None


@pytest.mark.parametrize('reverse', [False, True])
def test_chevrons_follow_order_in_both_directions(reverse):
    points = np.array([[200., 450.], [200., 50.]])
    if reverse:
        points = points[::-1]
    arrows = route_chevrons(points, [True, True], (0, 0, 500, 500))
    assert len(arrows) >= 3
    for arrow in arrows:
        tip_direction = arrow[1] - (arrow[0] + arrow[2])/2
        assert np.dot(tip_direction, points[1]-points[0]) > 0
    assert not route_chevrons(points, [False, True], (0, 0, 500, 500))


def test_offscreen_chevrons_have_bounded_work_and_stay_visible():
    arrows = route_chevrons([[-1e12, 200], [1e12, 200]], [True, True], (0, 0, 640, 480))
    assert 1 <= len(arrows) <= 12
    assert all(np.all((arrow >= [0, 0]) & (arrow <= [640, 480])) for arrow in arrows)


def test_ground_frame_clipping_uses_body_yaw_and_ignores_path_height():
    body = np.eye(4)
    body[:2, :2] = [[0, -1], [1, 0]]
    body[:3, 3] = [10, 20, .72]
    stamp = object()
    calls = []
    def lookup(target, source, image_stamp):
        calls.append((target, source, image_stamp))
        return body if source == 'body' else np.eye(4)
    node = NS(ground_frame='floor', base_frame='body', path_frame='map',
              path=np.array([[10, 21, 99], [10, 24, -99], [10, 40, 0]]),
              lookup_matrix=lookup, ground_render_offset_m=.015, style=RenderStyle())
    points, frame, visible = grounded_route(node, stamp)
    np.testing.assert_allclose(points[:, 2], .015)
    assert frame == 'floor' and visible.tolist() == [True, True, False]
    assert all(call[2] is stamp for call in calls)


def test_direction_pauses_on_stale_image_and_missing_image_time_pose():
    node = NS(navigation_active=True, path_error='', require_localizer_quality=False,
              path=np.array([[0., 0., 0.], [10., 0., 0.]]), path_frame='map', base_frame='body',
              image_stamp=NS(sec=100, nanosec=0), _image_received_at=time.monotonic(),
              camera_error='', _guidance_progress=0.)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_100_000_000))
    calls = []
    def lookup(target, source, stamp):
        calls.append(stamp)
        return np.eye(4)
    node.lookup_matrix = lookup
    assert guidance_state(node)['live'] and calls == [node.image_stamp]
    node.image_stamp.sec -= 2
    assert not guidance_state(node)['live'] and len(calls) == 1
    node.image_stamp.sec += 2
    node.lookup_matrix = lambda *_: None
    assert not guidance_state(node)['live']
    node.camera_error = 'unsupported encoding'
    assert not guidance_state(node)['live']


def test_camera_panel_arrows_clear_on_stale_frame():
    from PyQt5.QtWidgets import QApplication
    from nav_app.qt_nav_node import CameraPanel
    app = QApplication.instance() or QApplication([])
    panel = CameraPanel(compact=True, fill=True)
    panel.resize(480, 800)
    image = np.zeros((720, 1280, 3), np.uint8)
    center = np.array([[640., 700.], [640., 200.]])
    route = (center, center-[50, 0], center+[50, 0], np.ones(2, bool))
    cue = route_guidance([[0, 0], [10, 0]], [0, 0], 0.)
    panel.set_frame(image, route, 10., guidance=cue)
    panel.set_frame_age(.01, .02)
    panel.show()
    app.processEvents()
    panel.grab()
    assert not panel._route_arrow_path.isEmpty()
    panel.set_frame_age(.01, 2.)
    panel.grab()
    assert not panel._camera_live and panel._route is None
    panel.clear_frame()
    assert panel._route_arrow_path.isEmpty()
    panel.close()



def test_camera_redraws_on_changed_content_or_freshness_only(application):
    from nav_app.qt_nav_node import CameraPanel

    class CountingPanel(CameraPanel):
        def paintEvent(self, event):
            self.paints += 1
            super().paintEvent(event)

    panel = CountingPanel(fill=True)
    panel.paints = 0
    panel.resize(1600, 900)
    panel.show()
    image = np.zeros((360, 640, 3), np.uint8)
    panel.set_frame(image, None, 0., frame_revision=1)
    panel.set_frame_age(.1, .1)
    application.processEvents()
    before = panel.paints
    for _ in range(5):
        panel.set_frame(image, None, 0., frame_revision=1)
        panel.set_frame_age(.15, .15)
        application.processEvents()
    assert panel.paints == before
    panel.set_frame_age(.7, .7)
    application.processEvents()
    assert panel.paints == before + 1 and not panel._camera_live
    panel.set_frame(image, None, 0., frame_revision=2)
    panel.set_frame_age(.1, .1)
    application.processEvents()
    assert panel.paints == before + 2 and panel._camera_live
    panel.close()


def test_turn_distance_does_not_depend_on_planner_sampling():
    sparse = np.array([[0., 0.], [5., 0.], [5., 5.]])
    dense = np.vstack([np.linspace(a, b, 40, endpoint=False) for a, b in zip(sparse[:-1], sparse[1:])] + [sparse[-1:]])
    assert route_guidance(dense, [0, 0], 0.)['instruction'] == route_guidance(sparse, [0, 0], 0.)['instruction']

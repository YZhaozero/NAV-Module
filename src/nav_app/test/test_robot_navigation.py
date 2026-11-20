from types import SimpleNamespace as NS
from test_walking_guidance import Client, Goal, Future
from nav_app.features.robot_navigation import RobotNavigation


class RobotClient(Client):
    def __init__(self):
        super().__init__()
        self.feedback = []
    def send_goal_async(self, goal, feedback_callback):
        self.feedback.append(feedback_callback)
        return super().send_goal_async(goal)


def test_cancel_before_acceptance_blocks_new_motion_and_ignores_old_result():
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None)
    assert session.start(object())
    session.cancel()
    assert not session.start(object())
    old = Goal()
    client.requests[0].complete(old)
    assert old.cancel_count == 1
    old.result.complete(NS(status=5))
    assert not session.busy
    assert session.start(object())
    new = Goal()
    client.requests[1].complete(new)
    client.feedback[0](NS(feedback=NS(number_of_poses_remaining=99)))
    old.result.complete(NS(status=6))
    assert session.handle is new and session.phase == "running" and session.result is None


def test_cancel_timeout_keeps_handle_and_allows_retry_without_new_motion():
    now = [0.]
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None, clock=lambda: now[0])
    session.start(object())
    goal = Goal()
    client.requests[0].complete(goal)
    session.cancel()
    now[0] = 9.
    session.poll()
    assert session.phase == "cancel_failed" and session.handle is goal
    assert not session.start(object())
    session.cancel()
    assert goal.cancel_count == 2


def test_delayed_acceptance_after_submit_timeout_is_cancelled():
    now = [0.]
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None, clock=lambda: now[0])
    session.start(object())
    now[0] = 9.
    session.poll()
    goal = Goal()
    client.requests[0].complete(goal)
    assert goal.cancel_count == 1 and session.phase == "cancelling"


def test_cancel_ack_is_not_confused_with_robot_stopped():
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None)
    session.start(object())
    goal = Goal()
    response = Future()
    goal.cancel_goal_async = lambda: response
    client.requests[0].complete(goal)
    session.cancel()
    response.complete(NS(return_code=0, goals_canceling=[1]))
    assert session.busy and session.phase == 'cancelling'
    assert not session.start(object())
    goal.result.complete(NS(status=5))
    assert not session.busy and session.result == 5


def test_lost_goal_response_does_not_allow_a_second_motion_task():
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None)
    session.start(object())
    client.requests[0].complete(RuntimeError('response transport lost'))
    assert session.busy and session.phase == 'cancel_failed'
    assert session.result is None and not session.start(object())


def test_nonterminal_result_does_not_claim_the_robot_has_stopped():
    client = RobotClient()
    session = RobotNavigation(client, lambda _: None)
    session.start(object())
    goal = Goal()
    client.requests[0].complete(goal)
    goal.result.complete(NS(status=0))
    assert session.busy and session.phase == 'cancel_failed'
    assert session.result is None


def test_robot_goal_is_only_sent_after_explicit_preview_confirmation():
    import numpy as np
    from builtin_interfaces.msg import Time
    from nav_app.qt_nav_node import QtNavRosNode
    from nav_app.features.walking_guidance import WalkingGuidance
    from test_walking_guidance import path_reply
    class Device:
        send_navigation_waypoints = QtNavRosNode.send_navigation_waypoints
        begin_guidance = QtNavRosNode.begin_guidance
        grid_frame='map'
        _meshnav_active=False
        robot_profile=''
        operating_mode='walking'
        def navigation_route_ready(self):return True
        def walking_pose(self): return np.array([0., 0.]), 0.
        def get_clock(self): return NS(now=lambda:NS(to_msg=lambda:Time(sec=10), nanoseconds=10_000_000_000))
    device=Device()
    plan_client, robot_client = Client(), RobotClient()
    device.walking = WalkingGuidance(plan_client, lambda _:None)
    device.robot = RobotNavigation(robot_client, lambda _:None)
    for mode in ('walking','robot'):
        device.operating_mode=mode
        assert device.send_navigation_waypoints([np.array([5.,0.])])
        goal=Goal()
        plan_client.requests[-1].complete(goal)
        goal.result.complete(path_reply([(0,0),(5,0)]))
        assert not robot_client.requests
        if mode=='robot':
            assert not device.begin_guidance() and not robot_client.requests
            device.motion_session=NS(available=lambda:True)
        assert device.begin_guidance()
        assert len(robot_client.requests) == (1 if mode=='robot' else 0)

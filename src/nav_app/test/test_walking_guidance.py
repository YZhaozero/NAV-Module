from types import SimpleNamespace as NS
import numpy as np
from nav_app.features.walking_guidance import WalkingGuidance


class Future:
    def __init__(self):
        self.callbacks, self.value = [], None
    def add_done_callback(self, fn):
        self.callbacks.append(fn)
    def result(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value
    def complete(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


class Goal:
    accepted = True
    def __init__(self):
        self.result = Future()
        self.cancel_count = 0
    def cancel_goal_async(self):
        self.cancel_count += 1
        return Future()
    def get_result_async(self):
        return self.result


class Client:
    def __init__(self):
        self.requests = []
    def server_is_ready(self):
        return True
    def send_goal_async(self, goal):
        future = Future()
        self.requests.append(future)
        return future


def path_reply(points, frame="map"):
    poses = [NS(pose=NS(position=NS(x=x, y=y, z=0.))) for x, y in points]
    return NS(status=4, result=NS(path=NS(header=NS(frame_id=frame), poses=poses)))


def ready(points, clock=None):
    client = Client()
    session = WalkingGuidance(client, lambda _: None, **({"clock": clock} if clock else {}))
    session.plan(object())
    goal = Goal()
    client.requests[0].complete(goal)
    goal.result.complete(path_reply(points))
    return session, client, goal


def test_ending_then_replanning_ignores_late_old_acceptance_and_result():
    client = Client()
    session = WalkingGuidance(client, lambda _: None)
    session.plan(object())
    session.cancel()
    session.plan(object())
    old, current = Goal(), Goal()
    client.requests[0].complete(old)
    assert old.cancel_count == 1
    client.requests[1].complete(current)
    current.result.complete(path_reply([(0, 0), (5, 0)]))
    old.result.complete(path_reply([(0, 0), (999, 999)]))
    assert session.phase == "preview" and session.remaining == 5.
    assert session.begin()


def test_timeout_and_frame_mismatch_preserve_retryability():
    now = [0.]
    client = Client()
    session = WalkingGuidance(client, lambda _: None, clock=lambda: now[0])
    session.plan(object())
    now[0] = 13.
    session.poll()
    assert session.phase == "error"
    late = Goal()
    client.requests[0].complete(late)
    assert late.cancel_count == 1
    session.plan(object())
    next_goal = Goal()
    client.requests[1].complete(next_goal)
    next_goal.result.complete(path_reply([(0, 0), (5, 0)], "other_map"))
    assert session.phase == "error" and len(session.path) == 0


def test_pose_loss_hides_instruction_and_recovery_does_not_skip_route():
    session, _, _ = ready([(0, 0), (4, 0), (4, 4)])
    session.begin()
    session.update_pose([1., 0.], 0.)
    assert session.direction == "沿路线前行"
    progress = session.progress
    session.update_pose(None, 0.)
    assert session.phase == "lost" and "停下" in session.direction
    session.update_pose([4., 4.], 0.)
    assert session.phase == "deviated" and session.progress == progress
    session.update_pose([1.5, 0.], 0.)
    assert session.phase == "guiding" and session.remaining > 5.


def test_arrival_requires_route_progress_and_stable_position():
    now = [0.]
    session, _, _ = ready([(0, 0), (5, 0)], lambda: now[0])
    session.begin()
    for x in (1., 2., 3., 4., 4.5):
        session.update_pose([x, 0.], 0.)
    assert session.phase == "guiding"
    now[0] = 1.2
    session.update_pose([4.5, 0.], 0.)
    assert session.phase == "arrived" and session.result == 4


def test_turn_directions_are_relative_to_device_heading():
    session, _, _ = ready([(0, 0), (0, 5)])
    session.begin()
    session.update_pose([0., 0.], 0.)
    assert session.direction == "向左转"
    session.update_pose([0., 0.], np.pi)
    assert session.direction == "向右转"

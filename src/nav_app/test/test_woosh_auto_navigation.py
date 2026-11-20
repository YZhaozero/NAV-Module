"""Exercise the automatic WOOSH path without sending any physical command."""
import time
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace
from builtin_interfaces.msg import Time as RosTime

import pytest

from nav_app import woosh_base_adapter
from nav_app.features.navigation_stack import (
    NavigationStackConfig, build_navigation_launches, validate_navigation_stack,
)
from nav_app.qt_nav_node import QtNavRosNode


def test_only_live_pointcloud_nav2_can_enable_woosh_base(tmp_path):
    grid = tmp_path / "site.yaml"
    grid.touch()
    config = NavigationStackConfig(str(grid), "", robot_profile="woosh", woosh_control_mode=1)
    args = build_navigation_launches(config)["nav2"]
    assert args[:3] == ["launch", "nav_app", "woosh_nav2.launch.py"]
    assert {"enable_base_control:=true", "required_control_mode:=1"} <= set(args)
    without_base = build_navigation_launches(replace(config, woosh_control_mode=0))["nav2"]
    assert "enable_base_control:=true" not in without_base
    for overrides in (dict(sensor_source="bag", use_sim_time=True),
                      dict(use_sim_time=True), dict(localization_backend="amcl"),
                      dict(navigation_backend="walking"), dict(woosh_control_mode=3)):
        ok, reason = validate_navigation_stack(replace(config, **overrides))
        assert not ok and "WOOSH 底盘执行" in reason


def test_woosh_action_requires_preview_and_stationary_base():
    submitted = []
    mode = SimpleNamespace(available=lambda: True, state={"ready_to_start": False},
                           base_reason=lambda: "等待底盘实测速度归零")
    node = SimpleNamespace(robot_profile="woosh", operating_mode="robot", _woosh_motion_enabled=True,
        navigation_route_ready=lambda: True, walking_pose=lambda: ((0., 0.), 0.),
        motion_session=mode, walking=SimpleNamespace(phase="preview"), grid_frame="map",
        _planned_waypoints=[(1., 0.)], robot=SimpleNamespace(start=lambda goal: submitted.append(goal) or True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=RosTime)))
    assert not QtNavRosNode.begin_guidance(node) and not submitted
    assert node.navigation_status == "等待底盘实测速度归零"
    mode.state["ready_to_start"] = True
    assert QtNavRosNode.begin_guidance(node)
    assert submitted[0].poses[0].pose.position.x == 1.


@pytest.mark.parametrize("fault", ["permit", "timeout"])
def test_adapter_sends_no_idle_requests_and_prioritizes_zero(fault, monkeypatch):
    clock = [10.]
    sent = []
    pending = []
    cancelled = []
    status = []
    sample = {"PoseSpeed": (clock[0], None)}
    speed = [.0, .0]

    def send(values):
        future = Future()
        pending.append(future)
        sent.append(values)
        return future

    transport = SimpleNamespace(
        poll=lambda now: None,
        health=lambda now, **kwargs: dict(ready=True, mode=1, estop_released=True, measured=speed),
        clients={"Twist": SimpleNamespace(service_is_ready=lambda: True)},
        send=send, cancel=cancelled.append, samples=sample,
    )
    logger = SimpleNamespace(error=lambda msg: status.append(msg))
    node = SimpleNamespace(transport=transport, required_mode=1, command_at=clock[0],
        sequence=1, ack=0, velocity=(.1, 0., 0.), pending=None, permit_at=None,
        needs_stop=False, zero_ack_at=None, fault="", get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))),
        feedback_pub=SimpleNamespace(publish=lambda msg: None))
    node.fail = lambda reason: woosh_base_adapter.WooshBaseAdapter.fail(node, reason)
    monkeypatch.setattr(woosh_base_adapter.time, "monotonic", lambda: clock[0])
    tick = lambda: woosh_base_adapter.WooshBaseAdapter.tick(node)
    tick()
    assert not sent
    node.permit_at = clock[0]
    tick()
    assert sent == [(.1, 0., 0.)]
    if fault == "permit":
        node.permit_at = None
        clock[0] += .05
    else:
        clock[0] += .31
        node.permit_at = node.command_at = clock[0]
    tick()
    assert cancelled == [pending[0]]
    assert sent[-1] == (0., 0., 0.) and node.needs_stop
    assert bool(status) == (fault == "timeout")
    node.permit_at = None
    pending[-1].set_result(SimpleNamespace(ok=True))
    clock[0] += .05
    tick()
    assert node.needs_stop  # Old measured speed and a zero reply alone are insufficient.
    sample["PoseSpeed"] = (clock[0]+.01, None)
    pending[-1].set_result(SimpleNamespace(ok=True))
    clock[0] += .05
    tick()
    assert not node.needs_stop and node.pending is None


def test_sdk_adapter_uses_status_services_and_stops_with_real_wire_types(monkeypatch):
    vendor = pytest.importorskip("woosh_robot_msgs.srv", reason="Requires the installed WOOSH SDK")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from nav_app.motion_guard_node import MotionGuardNode
    from rclpy.signals import SignalHandlerOptions

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=["--ros-args", "-p", "required_control_mode:=1"],
               domain_id=202, signal_handler_options=SignalHandlerOptions.NO)
    adapter = woosh_base_adapter.WooshBaseAdapter()
    base = rclpy.create_node("nav_woosh_mock_base")
    guard = MotionGuardNode()
    monkeypatch.setattr(guard.health, "valid", lambda stamp: True)
    permit = base.create_publisher(Bool, "/nav/motion_permit", 1)
    planner = base.create_publisher(Twist, "/nav/planner_cmd_vel", 1)
    arm = base.create_client(SetBool, "/nav/arm_motion")
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(base)
    executor.add_node(guard)
    commands = []

    def mode(_request, response):
        response.info.ctrl.value = 1
        return response

    def device(_request, response):
        response.info.hardware = 0
        return response

    def speed(_request, response):
        response.info.has_field = response.info.TWIST_FIELD_SET
        response.info.twist.linear, _, response.info.twist.angular = commands[-1] if commands else (0., 0., 0.)
        return response

    def twist(request, response):
        commands.append((request.arg.linear, request.arg.linear_y, request.arg.angular))
        response.ok = True
        return response

    def until(predicate, refresh=lambda: None):
        deadline = time.monotonic()+5
        while not predicate() and time.monotonic() < deadline:
            refresh()
            executor.spin_once(timeout_sec=.02)
        assert predicate()

    def refresh_inputs():
        permit.publish(Bool(data=True))
        cmd = Twist()
        cmd.linear.x = .1
        planner.publish(cmd)

    try:
        until(lambda: "Twist" in guard.base_reason)
        assert not guard.guard.connected and not guard.guard.ready() and commands == []
        for name, callback in (("Mode", mode), ("DeviceState", device), ("PoseSpeed", speed), ("Twist", twist)):
            base.create_service(getattr(vendor, name), "/woosh_robot/robot/"+name, callback)
        until(lambda: guard.guard.connected and arm.service_is_ready())
        assert commands == []
        until(guard.guard.ready, refresh_inputs)
        arm_reply = arm.call_async(SetBool.Request(data=True))
        until(arm_reply.done, refresh_inputs)
        assert arm_reply.result().success
        until(lambda: any(command[0] > 0. for command in commands), refresh_inputs)
        until(lambda: commands[-1] == (0., 0., 0.) and not adapter.needs_stop and not guard.guard.armed,
              lambda: permit.publish(Bool(data=False)))
        assert not adapter.fault
    finally:
        executor.shutdown()
        adapter.destroy_node()
        base.destroy_node()
        guard.destroy_node()
        rclpy.try_shutdown()

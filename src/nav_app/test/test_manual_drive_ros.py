"""Real SDK wire types against mock services in domain 201; never the robot domain."""
import time
import pytest

vendor = pytest.importorskip("woosh_robot_msgs.srv", reason="ARM64 vendor SDK required")
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from nav_app.features.manual_drive import ManualDrive, WooshTeleopRos, ZERO


def test_sdk_contract_in_isolated_domain(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("NAV_GAMEPAD_PROFILES", str(tmp_path/"gamepads.json"))
    context = Context()
    rclpy.init(context=context, domain_id=201)
    node = rclpy.create_node("nav_manual_mock", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    commands, modes = [], []
    state = dict(mode=1, hardware=0)
    def mode(request, response):
        response.info.ctrl.value = state["mode"]
        return response
    def device(request, response):
        response.info.hardware = state["hardware"]
        return response
    def speed(request, response):
        response.info.has_field = response.info.TWIST_FIELD_SET
        response.info.twist.linear = .12
        response.info.twist.angular = -.2
        return response
    def switch(request, response):
        modes.append(request.arg.mode.value)
        response.ok = False
        response.msg = "Software switching is not enabled"
        return response
    def twist(request, response):
        commands.append((request.arg.linear, request.arg.linear_y, request.arg.angular))
        response.ok = True
        return response
    for name, callback in [("Mode", mode), ("DeviceState", device), ("PoseSpeed", speed),
                           ("SwitchControlMode", switch), ("Twist", twist)]:
        node.create_service(getattr(vendor, name), "/nav_manual_test/robot/"+name, callback)
    adapter = WooshTeleopRos(node, namespace="/nav_manual_test")
    controller = ManualDrive(lambda: "")
    controller.transport = adapter
    controller.chassis = "omni"
    def until(predicate):
        deadline = time.monotonic()+5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            controller.tick()
        assert predicate()
    try:
        until(lambda: controller.health()["ready"])
        assert controller.health()["measured"] == pytest.approx((.12, -.2))
        assert controller.health()["mode"] == 1
        assert "SwitchControlMode" not in adapter.clients
        assert commands == [] and modes == []
        assert controller.arm()
        controller.screen_input(-.5, 1., True)
        until(lambda: any(command[0] > 0 for command in commands))
        assert commands[-1] == pytest.approx((.15, 0., .15))
        assert state["mode"] == 1 and modes == []
        controller.screen_input(0., 0., False)
        until(lambda: commands[-1] == ZERO)
        controller.stop()
        until(lambda: controller.pending is None and controller.stop_pending is None)
        controller.strafe = True
        assert controller.arm()
        controller.screen_input(-1., 0., True)
        until(lambda: any(command[1] > 0 for command in commands))
        assert commands[-1] == pytest.approx((0., .15, 0.))
        state["hardware"] = adapter.emergency_mask
        until(lambda: not controller.armed)
        until(lambda: commands[-1] == ZERO)
        assert not controller.arm()
        from nav_app.ui.gamepad import GamepadPanel
        panel = GamepadPanel(node, lambda: dict(chassis="omni", linear=.15, angular=.3))
        monkeypatch.setattr(panel.driver, "start_launch", lambda args: True)
        panel.set_devices([dict(id=0, guid="a"*32, name="mock controller")])
        publisher = node.create_publisher(Joy, "/nav/manual_gamepad/joy", 1)
        message = Joy(axes=[.5, 1., 0., 0.], buttons=[0, 0, 0, 0, 1])
        deadline = time.monotonic()+3
        while panel.sample is None and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
        assert panel.sample[0] == pytest.approx((.5, 1., 0., 0.))
        assert panel.sample[1] == (0, 0, 0, 0, 1)
        panel.shutdown()
        assert all(client.srv_name.startswith("/nav_manual_test/") for client in adapter.clients.values())
    finally:
        adapter.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()

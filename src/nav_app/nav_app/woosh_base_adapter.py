"""Forward authorized Nav2 velocities through the WOOSH Agent SDK."""
import json
import signal
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String

from .features.manual_drive import WooshTeleopRos, ZERO
from .features.woosh_control import decode_command


class WooshBaseAdapter(Node):
    def __init__(self):
        super().__init__("nav_woosh_base_adapter")
        self.required_mode = self.declare_parameter("required_control_mode", 0).value
        if self.required_mode not in (1, 2):
            raise ValueError("Set required_control_mode to the mode confirmed by WOOSH for external velocity control")
        self.transport = WooshTeleopRos(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.request_pub = self.create_publisher(TwistStamped, "/nav/velocity_request", qos)
        self.feedback_pub = self.create_publisher(String, "/nav/base_feedback", qos)
        self.create_subscription(Twist, "/nav/planner_cmd_vel", self.planner, qos)
        self.create_subscription(String, "/nav/base_command", self.command, qos)
        self.create_subscription(Bool, "/nav/motion_permit", self.permit, qos)
        self.sequence = 0
        self.ack = 0
        self.velocity = ZERO
        self.command_at = None
        self.permit_at = None
        self.pending = None
        self.needs_stop = False
        self.zero_ack_at = None
        self.fault = ""
        self.timer = self.create_timer(.05, self.tick)

    def planner(self, message):
        request = TwistStamped()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "base_link"
        request.twist = message
        self.request_pub.publish(request)

    def permit(self, message):
        self.permit_at = time.monotonic() if message.data else None

    def command(self, message):
        try:
            sequence, velocity = decode_command(message.data, self.get_clock().now().nanoseconds, self.sequence)
        except (ValueError, KeyError, TypeError) as error:
            self.fail(str(error))
            return
        self.sequence, self.velocity = sequence, velocity
        self.command_at = time.monotonic()

    def fail(self, reason):
        if not self.fault:
            self.fault = reason
            self.get_logger().error(reason)

    def tick(self):
        now = time.monotonic()
        try:
            self.transport.poll(now)
        except RuntimeError as error:
            self.fail(f"WOOSH state request failed: {error}")
        health = self.transport.health(now, max_age=.45)
        released = health["estop_released"] and health["mode"] == self.required_mode
        permitted = self.permit_at is not None and 0 <= now-self.permit_at <= .3
        can_send = health["ready"] and released and permitted and not self.fault
        current = self.command_at is not None and 0 <= now-self.command_at <= .3
        values = self.velocity if can_send and current else ZERO
        if self.pending is not None:
            future, sent_at, sequence, sent = self.pending
            if future.done():
                self.pending = None
                try:
                    response = future.result()
                    if not response.ok:
                        self.fail("WOOSH velocity request rejected: " + response.msg)
                    else:
                        self.ack = sequence
                        if sent == ZERO and self.needs_stop and self.zero_ack_at is None:
                            self.zero_ack_at = now
                except RuntimeError as error:
                    self.fail(f"WOOSH velocity request failed: {error}")
            elif now-sent_at > .3 or (sent != ZERO and values == ZERO):
                self.transport.cancel(future)
                self.pending = None
                if now-sent_at > .3:
                    self.fail("WOOSH velocity service response timed out")

        measured = health["measured"]
        if measured is not None:
            vx, wz = measured
            if (self.zero_ack_at is not None and self.transport.samples["PoseSpeed"][0] > self.zero_ack_at
                    and abs(vx) < .02 and abs(wz) < .03):
                self.needs_stop = False
        if self.fault:
            reason = f"底盘执行故障：{self.fault}"
        elif not health["ready"]:
            reason = health["reason"]
        elif not health["estop_released"]:
            reason = "底盘急停已触发"
        elif health["mode"] != self.required_mode:
            reason = f"底盘当前模式 {health['mode']}，导航配置为 {self.required_mode}"
        elif self.needs_stop or abs(vx) >= .02 or abs(wz) >= .03:
            reason = "等待底盘实测速度归零"
        else:
            reason = ""
        feedback = dict(schema=1, stamp_ns=self.get_clock().now().nanoseconds,
            connected=health["ready"] and not self.fault, estop_released=released,
            ack_sequence=self.ack, measured_velocity=(vx, 0., wz) if measured is not None else None,
            stop_pending=self.needs_stop, base_reason=reason[:240],
            acknowledgement_source="woosh_service_response", velocity_source="woosh_pose_speed")
        self.feedback_pub.publish(String(data=json.dumps(feedback, allow_nan=False)))

        if self.pending is not None or not self.transport.clients["Twist"].service_is_ready():
            return
        if not self.needs_stop and (not can_send or self.fault):
            return
        values = ZERO if self.fault else values
        if values != ZERO:
            self.needs_stop = True
            self.zero_ack_at = None
        try:
            self.pending = (self.transport.send(values), now, self.sequence, values)
        except RuntimeError as error:
            self.fail(f"WOOSH velocity send failed: {error}")

    def stop_on_shutdown(self, executor):
        self.timer.cancel()
        if not self.needs_stop or not self.transport.clients["Twist"].service_is_ready():
            return
        try:
            future = self.transport.send(ZERO)
            executor.spin_until_future_complete(future, timeout_sec=.4)
            if not future.done() or not future.result().ok:
                self.get_logger().error("WOOSH shutdown zero speed was not acknowledged")
        except RuntimeError as error:
            self.get_logger().error(f"WOOSH shutdown zero speed failed: {error}")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WooshBaseAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while not stopping:
            executor.spin_once(timeout_sec=.05)
    finally:
        node.stop_on_shutdown(executor)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

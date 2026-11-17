"""Optional SDK adapter. Service acknowledgements are not motor acknowledgements."""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

from .features.woosh_control import decode_command


class WooshBaseAdapter(Node):
    def __init__(self):
        super().__init__("nav_woosh_base_adapter")
        self.declare_parameter("required_control_mode", 0)
        self.required_mode = self.get_parameter("required_control_mode").value
        if self.required_mode not in (1, 2, 3):
            raise ValueError("Set required_control_mode to the mode confirmed by WOOSH for external velocity control")
        from woosh_robot_msgs.srv import Twist as WooshTwist
        from woosh_robot_msgs.msg import DeviceState, DeviceStateHardwareBit, Mode, PoseSpeed
        self.request_type = WooshTwist.Request
        self.emergency_mask = DeviceStateHardwareBit.K_EMG_BTN
        self.client = self.create_client(WooshTwist, "/woosh_robot/robot/Twist")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.request_pub = self.create_publisher(TwistStamped, "/nav/velocity_request", qos)
        self.feedback_pub = self.create_publisher(String, "/nav/base_feedback", qos)
        self.create_subscription(Twist, "/nav/planner_cmd_vel", self.planner, qos)
        self.create_subscription(String, "/nav/base_command", self.command, qos)
        self.samples = {}
        for cls, topic, key in ((DeviceState, "DeviceState", "device"),
                                (Mode, "Mode", "mode"), (PoseSpeed, "PoseSpeed", "speed")):
            self.create_subscription(cls, "/woosh_robot/robot/" + topic,
                lambda msg, key=key: self.samples.update({key: (time.monotonic(), msg)}), qos)
        self.sequence = 0
        self.ack = 0
        self.velocity = (0., 0., 0.)
        self.command_at = None
        self.pending = None
        self.pending_at = None
        self.fault = ""
        self.create_timer(.05, self.tick)

    def planner(self, message):
        request = TwistStamped()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "base_link"
        request.twist = message
        self.request_pub.publish(request)

    def command(self, message):
        try:
            sequence, velocity = decode_command(message.data, self.get_clock().now().nanoseconds, self.sequence)
        except (ValueError, KeyError, TypeError) as exc:
            self.fault = str(exc)
            self.get_logger().error(self.fault)
            self.velocity = (0., 0., 0.)
            return
        self.sequence, self.velocity = sequence, velocity
        self.command_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = all(key in self.samples and 0 <= now-self.samples[key][0] <= .3
                    for key in ("device", "mode", "speed"))
        connected = fresh and self.client.service_is_ready() and not self.fault
        released = bool(fresh and not self.samples["device"][1].hardware & self.emergency_mask
                        and self.samples["mode"][1].ctrl.value == self.required_mode)
        measured = None
        if fresh:
            speed = self.samples["speed"][1].twist
            measured = (speed.linear, 0., speed.angular)
            if not all(math.isfinite(v) for v in measured):
                self.fault = "Non-finite WOOSH speed feedback"
                self.get_logger().error(self.fault)
                connected = False
            else:
                feedback = dict(schema=1, stamp_ns=self.get_clock().now().nanoseconds,
                    connected=bool(connected), estop_released=released, ack_sequence=self.ack,
                    measured_velocity=measured, acknowledgement_source="woosh_service_response",
                    velocity_source="woosh_pose_speed")
                self.feedback_pub.publish(String(data=json.dumps(feedback, allow_nan=False)))
        if self.pending is not None:
            if self.pending.done():
                response = self.pending.result()
                if response.ok:
                    self.ack = self.pending_sequence
                else:
                    self.fault = "WOOSH velocity request rejected: " + response.msg
                    self.get_logger().error(self.fault)
                self.pending = None
            elif now-self.pending_at > .3 and not self.fault:
                self.fault = "WOOSH velocity service response timed out"
                self.get_logger().error(self.fault)
            if self.pending is not None:
                return
        if not self.client.service_is_ready():
            return
        current = (connected and released and self.command_at is not None
                   and 0 <= now-self.command_at <= .3 and not self.fault)
        values = self.velocity if current else (0., 0., 0.)
        request = self.request_type()
        request.arg.linear, request.arg.linear_y, request.arg.angular = values
        self.pending_sequence = self.sequence
        self.pending = self.client.call_async(request)
        self.pending_at = now


def main(args=None):
    rclpy.init(args=args)
    node = WooshBaseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""Convert WOOSH MID-360 .3 IMU from g to SI; DLIO owns its extrinsics."""
from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


GRAVITY = 9.80665


def normalize_imu(message):
    if message.header.frame_id != "livox_frame":
        raise ValueError(f"Unexpected WOOSH IMU frame: {message.header.frame_id}")
    result = deepcopy(message)
    result.header.frame_id = "woosh_livox3_imu"
    for axis in ("x", "y", "z"):
        setattr(result.linear_acceleration, axis,
                getattr(message.linear_acceleration, axis) * GRAVITY)
    if message.linear_acceleration_covariance[0] >= 0:
        result.linear_acceleration_covariance = [
            value * GRAVITY ** 2 for value in message.linear_acceleration_covariance]
    # This driver publishes gyro/acceleration only, without an orientation estimate.
    result.orientation_covariance[0] = -1.0
    return result


class WooshImu(Node):
    def __init__(self):
        super().__init__("woosh_livox3_imu")
        self.declare_parameter("input_topic", "/foo/livox/imu_192_168_128_3")
        # Match the FAST-LIVO IMU reader: retain 10 seconds at 200 Hz during bursts.
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        self.output = self.create_publisher(Imu, "/nav/woosh/imu3_si", qos)
        self.subscription = self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self.receive, qos)

    def receive(self, message):
        self.output.publish(normalize_imu(message))


def main(args=None):
    rclpy.init(args=args)
    node = WooshImu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

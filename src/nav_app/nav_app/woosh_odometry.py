"""Express measured FAST-LIVO IMU propagation at the calibrated WOOSH base origin."""
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException
from tf2_msgs.msg import TFMessage
from .nav_math import transform_matrix


def base_odometry(message, imu_from_base, imu_frame="woosh_livox3_imu"):
    if message.header.frame_id != "odom" or message.child_frame_id != imu_frame:
        raise ValueError(f"WOOSH propagation requires odom -> {imu_frame}")
    pose = message.pose.pose
    q = pose.orientation; mount = imu_from_base.rotation
    for quaternion in (q, mount):
        norm = sum(getattr(quaternion, key)**2 for key in ("x", "y", "z", "w"))
        if not np.isfinite(norm) or not .99 < norm < 1.01:
            raise ValueError("Invalid odometry or calibration quaternion")
    world_from_imu = transform_matrix(Transform(rotation=q))[:3, :3]
    imu_from_base_matrix = transform_matrix(imu_from_base)
    offset = imu_from_base_matrix[:3, 3]
    base_from_imu = imu_from_base_matrix[:3, :3].T
    result = deepcopy(message)
    result.child_frame_id = "base_link"
    position = np.array([pose.position.x, pose.position.y, pose.position.z]) + world_from_imu @ offset
    result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = map(float, position)
    a,b,c,d = q.x,q.y,q.z,q.w; x,y,z,w = mount.x,mount.y,mount.z,mount.w
    rotation = result.pose.pose.orientation
    rotation.x, rotation.y, rotation.z, rotation.w = (d*x+a*w+b*z-c*y, d*y-a*z+b*w+c*x,
                                                    d*z+a*y-b*x+c*w, d*w-a*x-b*y-c*z)
    twist = message.twist.twist
    velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
    linear_base = base_from_imu @ (velocity + np.cross(angular, offset))
    angular_base = base_from_imu @ angular
    result.twist.twist.linear.x, result.twist.twist.linear.y, result.twist.twist.linear.z = map(float, linear_base)
    result.twist.twist.angular.x, result.twist.twist.angular.y, result.twist.twist.angular.z = map(float, angular_base)
    # ROS pose covariance uses fixed world axes; twist covariance uses child axes.
    pose_j = np.eye(6); x,y,z = world_from_imu @ offset
    pose_j[:3,3:] = [[0,z,-y],[-z,0,x],[y,-x,0]]
    twist_j = np.zeros((6,6)); twist_j[:3,:3] = twist_j[3:,3:] = base_from_imu
    x,y,z = offset
    twist_j[:3,3:] = base_from_imu @ np.array([[0,z,-y],[-z,0,x],[y,-x,0]])
    result.pose.covariance = (pose_j @ np.array(message.pose.covariance).reshape(6,6) @ pose_j.T).ravel().tolist()
    result.twist.covariance = (twist_j @ np.array(message.twist.covariance).reshape(6,6) @ twist_j.T).ravel().tolist()
    return result


class WooshOdometry(Node):
    def __init__(self):
        super().__init__("woosh_navigation_odometry")
        self.imu_frame = self.declare_parameter("imu_frame", "woosh_livox3_imu").value
        self.buffer = Buffer()
        self.static_subscription = self.create_subscription(TFMessage, "/tf_static", self.static_transforms,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        self.output = self.create_publisher(Odometry, "/nav/woosh/odom", qos)
        self.subscription = self.create_subscription(Odometry, "/LIVO2/imu_propagate", self.receive, qos)
        self.last_stamp = 0

    def static_transforms(self, message):
        for transform in message.transforms:
            self.buffer.set_transform_static(transform, "woosh_calibration")

    def receive(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp - self.last_stamp < 20_000_000:
            return  # 50 Hz navigation samples retain their actual measurement stamps.
        try:
            mount = self.buffer.lookup_transform(self.imu_frame, "base_link", Time()).transform
        except TransformException as error:
            self.get_logger().warning(f"Waiting for calibrated IMU-to-base TF: {error}", throttle_duration_sec=2.)
            return
        self.output.publish(base_odometry(message, mount, self.imu_frame))
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args); node = WooshOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()

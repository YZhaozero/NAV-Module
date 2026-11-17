import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from .quality import Fix, EnuOrigin, validate_fix


class GnssMonitor(Node):
    def __init__(self):
        super().__init__("nav_gnss")
        topic = self.declare_parameter("fix_topic", "/rtk/fix").value
        self.max_age = self.declare_parameter("max_age_s", 1.0).value
        self.max_std = self.declare_parameter("max_horizontal_std_m", 0.5).value
        self.max_speed = self.declare_parameter("max_speed_mps", 15.0).value
        if not all(math.isfinite(v) and v > 0 for v in (self.max_age, self.max_std, self.max_speed)):
            raise ValueError("GNSS thresholds must be finite and positive")
        self.origin = None
        self.accepted = self.rejected = 0
        self.last_stamp = None
        self.last_seen = None
        self.last_xyz = None
        self.message = "等待 GNSS 数据"
        self.level = DiagnosticStatus.WARN
        self.point_pub = self.create_publisher(PointStamped, "/rtk/enu", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/rtk/diagnostics", 10)
        self.create_subscription(NavSatFix, topic, self.on_fix, qos_profile_sensor_data)
        self.create_timer(0.5, self.publish_status)

    def on_fix(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        fix = Fix(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                  msg.latitude, msg.longitude, msg.altitude, msg.status.status,
                  msg.position_covariance_type, tuple(msg.position_covariance))
        valid, reason = validate_fix(fix, now, self.max_age, self.max_std)
        if valid and self.last_stamp is not None and fix.stamp <= self.last_stamp:
            valid, reason = False, "重复或乱序的 GNSS 数据"
        self.last_seen = now
        self.message = reason
        self.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        if not valid:
            self.rejected += 1
            return
        if self.origin is None:
            self.origin = EnuOrigin(fix.latitude, fix.longitude, fix.altitude)
        xyz = self.origin.convert(fix.latitude, fix.longitude, fix.altitude)
        if self.last_xyz is not None:
            distance = math.hypot(*(xyz[:2] - self.last_xyz[:2]))
            if distance > self.max_speed * (fix.stamp - self.last_stamp) + 3 * self.max_std:
                self.rejected += 1
                self.message = "GNSS 位置跳变超过运动范围"
                self.level = DiagnosticStatus.WARN
                return
        self.last_stamp = fix.stamp
        self.last_xyz = xyz
        self.accepted += 1
        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = "gnss_enu"
        point.point.x, point.point.y, point.point.z = map(float, xyz)
        self.point_pub.publish(point)

    def publish_status(self):
        now = self.get_clock().now()
        age = None if self.last_seen is None else now.nanoseconds * 1e-9 - self.last_seen
        stale = age is None or age > self.max_age or age < 0
        status = DiagnosticStatus(name="nav/gnss", hardware_id="configured_gnss",
            level=DiagnosticStatus.STALE if stale else self.level,
            message="GNSS 数据已中断或尚未收到" if stale else self.message)
        status.values = [KeyValue(key="accepted", value=str(self.accepted)),
                         KeyValue(key="rejected", value=str(self.rejected)),
                         KeyValue(key="origin", value=str(self.origin.origin if self.origin else None)),
                         KeyValue(key="fusion", value="disabled")]
        self.diag_pub.publish(DiagnosticArray(header=Header(
            stamp=now.to_msg()), status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = GnssMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

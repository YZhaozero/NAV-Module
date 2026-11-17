import pytest
rclpy = pytest.importorskip("rclpy")
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_rtk.gnss.node import GnssMonitor
from types import SimpleNamespace
from diagnostic_msgs.msg import DiagnosticStatus


def test_real_ros_message_rejects_duplicate_and_missing_covariance():
    rclpy.init()
    node = GnssMonitor()
    try:
        msg = NavSatFix()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude, msg.longitude, msg.altitude = 39.9, 116.4, 50.
        node.on_fix(msg)
        assert node.accepted == 0 and node.rejected == 1
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.04]
        node.on_fix(msg)
        node.on_fix(msg)
        assert node.accepted == 1 and node.rejected == 2
        assert node.origin.origin == (39.9, 116.4, 50.)
        node.publish_status()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_absence_jump_and_recovery_keep_origin_without_navigation_tf():
    rclpy.init()
    node = GnssMonitor()
    messages = []
    node.diag_pub = SimpleNamespace(publish=messages.append)
    try:
        node.publish_status()
        assert messages[-1].status[0].level == DiagnosticStatus.STALE
        msg = NavSatFix()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.latitude, msg.longitude, msg.altitude = 39.9, 116.4, 50.
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.04]
        node.on_fix(msg)
        origin = node.origin
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.longitude += 1.
        node.on_fix(msg)
        assert node.rejected == 1 and "跳变" in node.message
        node.last_seen -= 5
        node.publish_status()
        assert messages[-1].status[0].level == DiagnosticStatus.STALE
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.longitude -= 1.
        node.on_fix(msg)
        assert node.accepted == 2 and node.origin is origin
        node.publish_status()
        assert dict((p.key, p.value) for p in messages[-1].status[0].values)["fusion"] == "disabled"
        assert not hasattr(node, "tf_broadcaster")
    finally:
        node.destroy_node()
        rclpy.shutdown()

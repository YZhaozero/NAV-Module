import time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped


class RtkMonitorAdapter:
    """Read the validated GNSS stream; it never publishes navigation TF."""
    def __init__(self, node):
        self._message = "等待 GNSS 数据"
        self._received = 0.
        self.position = None
        self.counts = ""
        self._subscriptions = [
            node.create_subscription(DiagnosticArray, "/rtk/diagnostics", self._status, 10),
            node.create_subscription(PointStamped, "/rtk/enu", self._position, 10)]

    @property
    def status(self):
        return self._message if time.monotonic() - self._received < 2.0 else "等待 GNSS 监测数据"

    def _status(self, msg):
        for status in msg.status:
            if status.name == "nav/gnss":
                self._message = status.message
                self._received = time.monotonic()
                values = {v.key: v.value for v in status.values}
                self.counts = f"接收有效 {values.get('accepted', '0')} · 拒绝 {values.get('rejected', '0')}"

    def _position(self, msg):
        self.position = (msg.point.x, msg.point.y, msg.point.z)

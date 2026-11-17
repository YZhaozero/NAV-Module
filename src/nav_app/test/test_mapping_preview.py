"""Preview must follow available geometry when RGB has no visible points."""
import time
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from nav_app.features.mapping import MappingRosAdapter


class Node:
    def __init__(self): self.subscriptions = {}
    def create_timer(self, *args): return object()
    def destroy_timer(self, timer): pass
    def create_client(self, *args): return None
    def create_subscription(self, cls, topic, callback, qos):
        self.subscriptions[topic] = callback
        return topic
    def destroy_subscription(self, topic): self.subscriptions.pop(topic)


def cloud(x):
    header = Header(); header.frame_id = 'odom'
    return point_cloud2.create_cloud_xyz32(header, [[x, 0., 1.]])


def settle(adapter):
    revision = adapter.cloud_revision
    deadline = time.monotonic()+2
    while adapter.cloud_revision == revision and time.monotonic() < deadline:
        adapter.poll_cloud(); time.sleep(.005)
    assert adapter.cloud_revision > revision


def test_complete_preview_legacy_selection_fallback_expiry_and_stop():
    node = Node(); adapter = MappingRosAdapter(node)
    try:
        assert adapter.start_preview('/fast_livo/map_rgb')[0]
        old_geometry = node.subscriptions['/fast_livo/map']
        old_geometry(cloud(1.)); settle(adapter)
        assert adapter.cloud[0, 0] == 1 and adapter.cloud_frame == 'odom'
        assert '/fast_livo/map_rgb' not in node.subscriptions
        node.subscriptions['/fast_livo/map_preview'](cloud(2.)); settle(adapter)
        assert adapter.cloud[0, 0] == 2
        revision = adapter.cloud_revision
        old_geometry(cloud(3.)); time.sleep(.02); adapter.poll_cloud()
        assert adapter.cloud_revision == revision
        adapter._rgb_received_at -= 4.
        old_geometry(cloud(3.)); settle(adapter)
        assert adapter.cloud[0, 0] == 3
        adapter.start_preview('/fast_livo/map_rgb')
        old_geometry(cloud(4.)); time.sleep(.02); adapter.poll_cloud()
        assert adapter.cloud is None, 'Old-session geometry must not unlock a new capture'
        adapter.stop_preview()
        assert not node.subscriptions
    finally: adapter.close()


def test_other_backend_does_not_subscribe_to_fastlivo_geometry():
    node = Node(); adapter = MappingRosAdapter(node)
    try:
        assert adapter.start_preview('/dlio/map_node/map')[0]
        assert list(node.subscriptions) == ['/dlio/map_node/map']
    finally: adapter.close()

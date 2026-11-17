import json
import threading
from types import SimpleNamespace as NS
import pytest


def test_720p_burst_preserves_full_subscription_window_without_unbounded_memory():
    """A delivered 16-frame burst must fit; the next frame remains bounded.

    Field replay delivered every input but the 32 MiB byte limit accepted only
    twelve 720p CDR frames, independently of the declared 16-frame window.
    """
    entered, release = threading.Event(), threading.Event()
    output = []
    payload_bytes = 1280 * 720 * 3 + 72
    def transform(message):
        entered.set()
        assert release.wait(5)
        return NS(data=b'jpeg', n=message.n)
    worker = BoundedImageWorker(transform, lambda message: output.append(message.n))
    try:
        worker.submit(NS(data=bytes(payload_bytes), n=0))
        assert entered.wait(3)
        for index in range(1, 17):
            worker.submit(NS(data=bytes(payload_bytes), n=index))
        snapshot = worker.snapshot()
        assert snapshot['received'] == 17
        assert snapshot['dropped'] == 1
        assert snapshot['pending_bytes'] == 16 * payload_bytes
        assert snapshot['pending_bytes'] <= worker.max_bytes <= 64 * 1024**2
    finally:
        release.set()
        result = worker.close()
    assert output == list(range(16))
    assert result['published'] == 16 and result['pending_bytes'] == 0
import yaml
from nav_app.features.image_archive import (
    BoundedImageWorker, FrameNumberWindow, ImageArchiveEncoder, archive_routes,
    encode_image, decode_image)


def test_worker_is_bounded_and_drains_in_order():
    ready, release = threading.Event(), threading.Event()
    output = []
    def encode(m):
        ready.set(); assert release.wait(3)
        return m
    worker = BoundedImageWorker(encode, lambda m: output.append(m.n), max_bytes=4, max_frames=2)
    worker.submit(NS(data=b'12', n=1)); assert ready.wait(3)
    worker.submit(NS(data=b'34', n=2))
    worker.submit(NS(data=b'56', n=3))
    assert worker.snapshot()['peak_bytes'] == 4
    release.set()
    result = worker.close()
    assert output == [1, 2]
    assert result['received'] == 3 and result['published'] == 2 and result['dropped'] == 1
    assert result['pending_bytes'] == 0 and not worker.thread.is_alive()


def test_worker_reports_failure_without_deadlocking_close():
    def fail(m): raise ValueError('bad input')
    worker = BoundedImageWorker(fail, lambda m: None)
    worker.submit(NS(data=b'1'))
    assert worker.close()['error'] == 'bad input'


def test_serialized_worker_budgets_active_cdr_bytes_and_drains():
    entered, release = threading.Event(), threading.Event()
    output = []
    def convert(data):
        entered.set(); assert release.wait(3)
        return NS(data=data)
    worker = BoundedImageWorker(convert, lambda m: output.append(m.data),
                                max_bytes=9, max_frames=3, size_of=len)
    worker.submit(b'first'); assert entered.wait(3)
    worker.submit(b'next')
    worker.submit(b'over')
    assert worker.snapshot()['pending_bytes'] == 9
    release.set()
    result = worker.close()
    assert output == [b'first', b'next']
    assert (result['received'], result['published'], result['dropped']) == (3, 2, 1)
    assert result['pending_bytes'] == 0 and not worker.thread.is_alive()


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
@pytest.mark.parametrize('bigendian', [False, True])
@pytest.mark.parametrize('publisher_kind', ['reliable', 'best_effort', 'undiscovered'])
def test_raw_subscription_defers_real_ros_conversion_and_preserves_archive(
        monkeypatch, encoding, bigendian, publisher_kind):
    """Real CDR bytes reach the callback; legacy and new JPEG bytes must agree."""
    Image = pytest.importorskip('sensor_msgs.msg').Image
    serialization = pytest.importorskip('rclpy.serialization')
    from rclpy.qos import ReliabilityPolicy
    from array import array
    entered, release = threading.Event(), threading.Event()
    input_messages = []
    for index in range(2):
        message = Image()
        message.header.stamp.sec = 123 + index
        message.header.stamp.nanosec = 456789
        message.header.frame_id = 'camera_color_optical_frame'
        message.width, message.height, message.step = 32, 24, 100
        message.encoding, message.is_bigendian = encoding, bigendian
        message.data = array('B', ((i + index * 51) % 256 for i in range(2400)))
        input_messages.append(message)
    serialized = [serialization.serialize_message(m) for m in input_messages]
    expected = [encode_image(m) for m in input_messages]
    deserialize = serialization.deserialize_message
    decoded = []
    def delayed_deserialize(data, cls):
        assert threading.current_thread() is not threading.main_thread()
        entered.set(); assert release.wait(3)
        message = deserialize(data, cls)
        decoded.append(message)
        return message
    monkeypatch.setattr(serialization, 'deserialize_message', delayed_deserialize)

    class Node:
        def __init__(self): self.destroyed = False; self.output = []
        def get_publishers_info_by_topic(self, name):
            if publisher_kind == 'undiscovered':
                return []
            reliability = (ReliabilityPolicy.RELIABLE if publisher_kind == 'reliable'
                           else ReliabilityPolicy.BEST_EFFORT)
            return [NS(qos_profile=NS(reliability=reliability))]
        def create_publisher(self, *args):
            return NS(publish=self.output.append, wait_for_all_acked=self.acked)
        def create_subscription(self, cls, topic, callback, qos, *, raw=False):
            assert cls is Image and raw and qos.depth == 16
            assert qos.reliability == (ReliabilityPolicy.RELIABLE if publisher_kind == 'reliable'
                                       else ReliabilityPolicy.BEST_EFFORT)
            self.callback = callback
            return object()
        def destroy_subscription(self, subscription):
            self.destroyed = True
            release.set()  # closing the input happens before draining the worker
            return True
        def acked(self, duration):
            assert self.destroyed and len(self.output) == 2
            return True

    node = Node()
    encoder = ImageArchiveEncoder(node, '/camera/raw', '/test/archive')
    try:
        assert encoder.worker.max_bytes == 48 * 1024**2
        assert encoder.worker.max_frames == 16
        node.callback(serialized[0]); assert entered.wait(3)
        node.callback(serialized[1])
        assert not node.output  # the ROS callback returns while conversion waits
        assert encoder.worker.snapshot()['pending_bytes'] == sum(map(len, serialized))
        stats = encoder.close()
    finally:
        release.set()
        encoder.close()
    assert decoded == input_messages  # includes stamp, padding, encoding and endian
    # CDR padding is unspecified; compare every ROS field and the exact JPEG
    # payload rather than unrelated allocator contents in alignment padding.
    assert node.output == expected
    assert [bytes(m.data) for m in node.output] == [bytes(m.data) for m in expected]
    assert (stats['received'], stats['published'], stats['dropped']) == (2, 2, 0)
    assert stats['pending_bytes'] == 0 and not stats['error']


def test_raw_archive_reports_invalid_cdr_in_worker():
    pytest.importorskip('sensor_msgs.msg')
    from nav_app.features.image_archive import encode_serialized_image
    worker = BoundedImageWorker(encode_serialized_image, lambda m: None, size_of=len)
    worker.submit(b'not valid CDR')
    result = worker.close()
    assert result['error'] and result['published'] == 0
    assert result['pending_bytes'] == 0 and not worker.thread.is_alive()


def test_real_ros_image_subscription_reencodes_same_jpeg(monkeypatch):
    """Exercise actual DDS Image -> raw callback -> worker -> CompressedImage."""
    msgs = pytest.importorskip('sensor_msgs.msg')
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from array import array
    import time
    import uuid
    from nav_app.features import image_archive
    original = image_archive.encode_serialized_image
    converted = []
    def transform(data):
        assert isinstance(data, bytes)
        converted.append(threading.current_thread().name)
        return original(data)
    monkeypatch.setattr(image_archive, 'encode_serialized_image', transform)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('archive_raw_test_' + uuid.uuid4().hex[:12], context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    base = '/archive_raw_test/run_' + uuid.uuid4().hex
    source = node.create_publisher(msgs.Image, base + '/raw', 8)
    received = []
    node.create_subscription(msgs.CompressedImage, base + '/jpeg', received.append, 16)
    encoder = ImageArchiveEncoder(node, base + '/raw', base + '/jpeg')
    def spin_until(predicate):
        deadline = time.monotonic() + 5.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate()
    try:
        assert encoder.subscription.qos_profile.depth == 16
        assert encoder.worker.max_bytes == 48 * 1024**2
        assert encoder.worker.max_frames == 16
        spin_until(lambda: source.get_subscription_count() == 1
                   and encoder.publisher.get_subscription_count() == 1)
        expected = []
        for index in range(3):
            message = msgs.Image()
            message.header.stamp.sec = 456 + index
            message.header.stamp.nanosec = 654321
            message.header.frame_id = 'optical'
            message.width, message.height, message.step = 96, 64, 292
            message.encoding = 'rgb8'
            message.data = array('B', ((i + index * 7) % 256 for i in range(64 * 292)))
            expected.append(encode_image(message))
            source.publish(message)
            spin_until(lambda: len(received) == index + 1)
        stats = encoder.close()
        assert received == expected
        assert converted == ['rgb-archive'] * 3
        assert stats['received'] == stats['published'] == 3
        assert stats['dropped'] == 0 and not stats['error']
    finally:
        encoder.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_counter_window_excludes_start_stop_edges_but_detects_internal_gap():
    w = FrameNumberWindow()
    for t, n in [(10, 1), (20, 2), (30, 3), (40, 5), (50, 6)]: w.metadata(t, n)
    for t in [20, 30, 40]: w.image(t)
    assert w.counter == dict(first=2, last=5, count=3, non_increasing=0)
    assert w.counter['last'] - w.counter['first'] + 1 == 4


def test_replay_restores_sensor_contract_and_remaps(tmp_path):
    from nav_app.features.rosbag import read_bag_metadata, build_play_arguments, PlaybackRequest
    route = dict(source_topic='/rgb/image_raw', stored_topic='/rgb/image_raw/image_archive/jpeg', codec='jpeg', quality=95)
    (tmp_path/'nav_manifest.json').write_text(json.dumps(dict(image_archive=dict(schema=1, profile='jpeg95', images=[route]))))
    info = dict(topics_with_message_count=[dict(topic_metadata=dict(name=route['stored_topic'],
        type='sensor_msgs/msg/CompressedImage'), message_count=10)])
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))
    topic = read_bag_metadata(tmp_path).topics[0]
    assert (topic.name, topic.message_type, topic.message_count) == ('/rgb/image_raw', 'sensor_msgs/msg/Image', 10)
    args = build_play_arguments(PlaybackRequest(tmp_path, ('/rgb/image_raw', '/imu'), (('/rgb/image_raw', '/camera/rgb'), ('/imu', '/livox/imu'))))
    assert args[:4] == ['run', 'nav_app', 'image_archive_play', '--routes']
    assert json.loads(args[4])[0]['output_topic'] == '/camera/rgb'
    assert args[args.index('--topics')+1:args.index('--remap')] == [route['stored_topic'], '/imu']
    assert args[-1] == '/imu:=/livox/imu'
    imu_only = build_play_arguments(PlaybackRequest(tmp_path, ('/imu',)))
    assert imu_only[:2] == ['bag', 'play']
    assert archive_routes(tmp_path)[0] == route




@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_jpeg_keeps_header_dimensions_colors_and_handles_row_padding(encoding):
    Image = pytest.importorskip('sensor_msgs.msg').Image
    np = pytest.importorskip('numpy')
    m = Image(); m.width = 32; m.height = 24; m.step = 100; m.encoding = encoding
    m.header.stamp.sec = 123; m.header.stamp.nanosec = 456; m.header.frame_id = 'optical'
    pixels = np.zeros((24, 100), np.uint8)
    pixels[:, :96].reshape(24,32,3)[:] = [220, 50, 20]
    m.data = pixels.tobytes()
    archived = encode_image(m); restored = decode_image(archived)
    assert restored.header == m.header and restored.encoding == m.encoding
    assert (restored.width, restored.height, restored.step) == (32,24,96)
    error = np.abs(np.frombuffer(restored.data, np.uint8).reshape(24,32,3).astype(float)-[220,50,20])
    assert error.max() < 4
    m.encoding = '16UC1'
    with pytest.raises(ValueError, match='深度必须无损'): encode_image(m)
    archived.data = b'not jpeg'
    with pytest.raises(ValueError, match='损坏'): decode_image(archived)


def test_archive_audit_decodes_every_frame_and_checks_camera_dimensions(tmp_path):
    msgs = pytest.importorskip('sensor_msgs.msg')
    from rclpy.serialization import serialize_message
    from nav_app.features.bag_audit import audit_bag
    rgb, info_topic = '/rgb/image_raw/image_archive/jpeg', '/rgb/camera_info'
    route = dict(source_topic='/rgb/image_raw', stored_topic=rgb, codec='jpeg', quality=95)
    manifest = dict(image_archive=dict(schema=1, profile='jpeg95', images=[route]),
        capture_protocol=dict(roles=dict(rgb=rgb, camera_info=info_topic)),
        calibration_start={info_topic: dict(width=32, height=24)})
    (tmp_path/'nav_manifest.json').write_text(json.dumps(manifest))
    rows = []
    for sec in range(1,4):
        m=msgs.Image(); m.width=32; m.height=24; m.step=96; m.encoding='rgb8'; m.data=bytes(32*24*3)
        m.header.stamp.sec=sec
        c=msgs.CameraInfo(); c.header=m.header
        rows += [(info_topic, serialize_message(c), sec*10**9), (rgb, serialize_message(encode_image(m)), sec*10**9+100)]
    (tmp_path/'part.mcap').write_bytes(b'fixture')
    topics=[dict(topic_metadata=dict(name=n,type=t),message_count=3) for n,t in
            [(rgb,'sensor_msgs/msg/CompressedImage'),(info_topic,'sensor_msgs/msg/CameraInfo')]]
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
        relative_file_paths=['part.mcap'], message_count=6, topics_with_message_count=topics))))
    class Reader:
        def __init__(self): self.rows=iter(rows); self.count=0
        def has_next(self): return self.count<len(rows)
        def read_next(self): self.count+=1; return next(self.rows)
    result=audit_bag(tmp_path,[rgb,info_topic],reader_factory=lambda *_:Reader())
    assert result['complete'], result
    assert result['image_archive'][rgb]['decoded']==3
    manifest['calibration_start'][info_topic]['width']=64
    (tmp_path/'nav_manifest.json').write_text(json.dumps(manifest))
    result=audit_bag(tmp_path,reader_factory=lambda *_:Reader())
    assert not result['complete'] and any('标定不一致' in e for e in result['errors'])

"""Versioned RGB archive codec; live estimator topics are never changed."""
from collections import deque
from cv2 import error as ImageCodecError
from array import array
import json
from pathlib import Path
import threading
import time

# Sixteen 1280x720 RGB CDR frames require about 42.2 MiB. The byte and frame
# bounds must both accommodate the subscriber's burst window; allocation is
# demand-driven, and oversized frames still hit the fixed byte limit.
ARCHIVE_MAX_BYTES = 48 * 1024**2
ARCHIVE_MAX_FRAMES = 16


def archive_routes(bag):
    path = Path(bag) / 'nav_manifest.json'
    if not path.is_file():
        return []
    document = json.loads(path.read_text(encoding='utf-8'))
    archive = document.get('image_archive', {})
    if not archive:
        return []
    if archive.get('schema') != 1 or archive.get('profile') != 'jpeg95':
        raise ValueError('不支持的图像录制格式')
    routes = archive.get('images', [])
    sources, stored = set(), set()
    for route in routes:
        a, b = route['source_topic'], route['stored_topic']
        if (not a.startswith('/') or not b.startswith('/') or a == b
                or a in sources or b in stored or route.get('codec') != 'jpeg'
                or route.get('quality') != 95):
            raise ValueError('图像录制话题配置无效')
        sources.add(a); stored.add(b)
    if sources & stored:
        raise ValueError('图像录制与回放话题冲突')
    return routes


def rgb_array(message):
    import cv2
    import numpy as np
    if message.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('彩色压缩仅支持rgb8/bgr8，深度必须无损保存')
    if (message.width <= 0 or message.height <= 0 or message.step < message.width * 3
            or len(message.data) != message.step * message.height):
        raise ValueError('彩色图像尺寸与数据长度不一致')
    a = np.ndarray((message.height, message.width, 3), np.uint8,
                   buffer=message.data, strides=(message.step, 3, 1))
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if message.encoding == 'rgb8' else a


def encode_image(message):
    import cv2
    from sensor_msgs.msg import CompressedImage
    ok, payload = cv2.imencode('.jpg', rgb_array(message), [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError('彩色图像压缩失败')
    result = CompressedImage()
    result.header = message.header
    result.format = message.encoding + '; jpeg compressed bgr8'
    # ROS generated setters validate plain bytes element by element in Python.
    # A typed byte array uses their C-backed fast path (important at 720p/15Hz).
    result.data = array('B', payload.tobytes())
    return result


def encode_serialized_image(data):
    # Keep large Image conversion off the recorder's shared ROS executor. Use
    # ROS' own decoder so row padding, byte order and all header fields retain
    # exactly the same contract as the former typed subscription.
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    return encode_image(deserialize_message(data, Image))


def decode_image(message):
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    encoding = message.format.split(';', 1)[0].strip()
    if encoding not in ('rgb8', 'bgr8') or not any(codec in message.format for codec in ('jpeg', 'png')):
        raise ValueError('不支持的彩色图像编码')
    data = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if data is None:
        raise ValueError('录包中的彩色图像已损坏')
    if encoding == 'rgb8':
        data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
    result = Image()
    result.header = message.header
    result.height, result.width = data.shape[:2]
    result.encoding = encoding
    result.is_bigendian = False
    result.step = result.width * 3
    result.data = array('B', data.tobytes())
    return result


class BoundedImageWorker:
    """Bounded input FIFO including active bytes; conversion runs on one worker.

    Codec workspace (one decoded image and color conversion) is separate from
    the queued input budget. Serialized input includes its CDR header in that
    budget, and never creates a second unbounded queue of decoded images.
    """
    def __init__(self, transform, publish, max_bytes=ARCHIVE_MAX_BYTES, max_frames=ARCHIVE_MAX_FRAMES,
                 *, size_of=None, stamp_of=None):
        self.transform, self.publish = transform, publish
        self.size_of = size_of or (lambda message: len(message.data))
        self.stamp_of = stamp_of
        self.accepted = 0
        self.first_accepted_stamp = self.last_accepted_stamp = None
        self.first_published_stamp = self.last_published_stamp = None
        self.max_bytes, self.max_frames = max_bytes, max_frames
        self.condition = threading.Condition()
        self.queue = deque()
        self.pending_bytes = self.pending_frames = 0
        self.received = self.published = self.dropped = self.output_bytes = 0
        self.peak_bytes = 0
        self.transform_seconds = self.publish_seconds = 0.
        self.max_transform_s = self.max_publish_s = 0.
        self.error = ''
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='rgb-archive', daemon=True)
        self.thread.start()

    def submit(self, message):
        size = self.size_of(message)
        with self.condition:
            if self.closed or self.error:
                return
            self.received += 1
            if self.pending_bytes + size > self.max_bytes or self.pending_frames >= self.max_frames:
                self.dropped += 1
                return
            stamp = self.stamp_of(message) if self.stamp_of is not None else None
            if self.stamp_of is not None and (not isinstance(stamp,int) or stamp <= 0):
                self.error = '彩色归档输入的原始时间戳无效'
                return
            self.accepted += 1
            if self.first_accepted_stamp is None:self.first_accepted_stamp=stamp
            self.last_accepted_stamp=stamp
            self.pending_bytes += size; self.pending_frames += 1
            self.peak_bytes = max(self.peak_bytes, self.pending_bytes)
            self.queue.append((message, size, stamp)); self.condition.notify()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.queue or self.closed)
                if not self.queue:
                    return
                message, size, stamp = self.queue.popleft()
            try:
                started = time.monotonic()
                result = self.transform(message)
                encoded = time.monotonic()
                self.publish(result)
                delivered = time.monotonic()
                with self.condition:
                    self.published += 1; self.output_bytes += len(result.data)
                    if self.first_published_stamp is None:self.first_published_stamp=stamp
                    self.last_published_stamp=stamp
                    self.transform_seconds += encoded-started
                    self.publish_seconds += delivered-encoded
                    self.max_transform_s = max(self.max_transform_s, encoded-started)
                    self.max_publish_s = max(self.max_publish_s, delivered-encoded)
            except (ImageCodecError, OSError, RuntimeError, ValueError, TypeError, MemoryError) as exc:
                with self.condition:
                    self.error = str(exc)
                    self.queue.clear()
                    self.pending_bytes = self.pending_frames = 0
                return
            with self.condition:
                self.pending_bytes -= size; self.pending_frames -= 1

    def snapshot(self):
        with self.condition:
            value = dict(received=self.received, published=self.published, dropped=self.dropped,
                        pending_bytes=self.pending_bytes, peak_bytes=self.peak_bytes,
                        output_bytes=self.output_bytes, error=self.error,
                        transform_seconds=self.transform_seconds, publish_seconds=self.publish_seconds,
                        max_transform_s=self.max_transform_s, max_publish_s=self.max_publish_s)
            if self.stamp_of is not None:
                value['recording_window']=dict(schema=1,first_stamp_ns=self.first_accepted_stamp,
                    last_stamp_ns=self.last_accepted_stamp,accepted=self.accepted,published=self.published,
                    first_published_stamp_ns=self.first_published_stamp,last_published_stamp_ns=self.last_published_stamp,
                    reference='Original Image headers accepted after recorder endpoint readiness')
            return value

    def close(self, timeout=15.):
        with self.condition:
            self.closed = True; self.condition.notify_all()
        self.thread.join(timeout)
        if self.thread.is_alive():
            raise TimeoutError('彩色图像归档收尾超时')
        return self.snapshot()


class ImageArchiveEncoder:
    def __init__(self, node, source, stored, publisher=None):
        import cv2
        from sensor_msgs.msg import Image, CompressedImage
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        cv2.setNumThreads(1)
        self.node = node
        self.publisher = publisher or node.create_publisher(CompressedImage, stored,
            QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE))
        from .capture_quality import header_stamp
        self.worker = BoundedImageWorker(encode_serialized_image,
            self.publisher.publish,size_of=len,stamp_of=lambda data:header_stamp(data,'sensor_msgs/msg/Image'))
        infos = node.get_publishers_info_by_topic(source)
        reliable = infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
        # Keep a bounded 16-sample DDS history for bursty delivery before the
        # callback can enqueue work. This is separate from the worker's
        # 16-frame / 48-MiB budget: a nine-frame replay burst can evict its oldest
        # sample with depth 8 before the worker receives it. DDS payload storage
        # is additional to the worker budget (about 42 MiB at 720p RGB).
        self.subscription = node.create_subscription(Image, source, self.worker.submit,
            QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
            raw=True)

    def close(self):
        from rclpy.duration import Duration
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription); self.subscription = None
        result = self.worker.close()
        if not self.publisher.wait_for_all_acked(Duration(seconds=5)):
            result['error'] = result['error'] or '彩色归档消息未完成传输'
        return result


def pending_recorder_endpoints(node, publisher, topics, recorder_name='rosbag2_recorder'):
    """Only the recorder's endpoints count, and JPEG must be DDS matched.

    The guard supplies offered, selected topics, excluding absent optional
    receivers. No samples are consumed to probe readiness.
    """
    missing=[]
    for topic in topics:
        endpoints=node.get_subscriptions_info_by_topic(topic)
        if not any(row.node_name==recorder_name for row in endpoints):missing.append(topic)
    if publisher.get_subscription_count()<1:missing.append('彩色归档DDS接收端')
    return missing


class FrameNumberWindow:
    """Bounded join: sensor counters inside the archived image header interval."""
    def __init__(self):
        self.pending = deque()
        self.first = self.last = None
        self.counter = None

    def image(self, stamp):
        if stamp is None:
            raise ValueError('图像时间戳无效')
        if self.first is None:
            self.first = stamp
        self.last = stamp
        self._flush()

    def metadata(self, stamp, number):
        if stamp is None:
            raise ValueError('相机元数据时间戳无效')
        self.pending.append((stamp, number))
        self._flush()
        # Metadata can precede the first stored image. Keep only a bounded
        # startup tail; once images exist an overflow means missing evidence.
        if len(self.pending) > 512:
            if self.first is not None:
                raise ValueError('图像与相机帧计数长时间未同步')
            self.pending.popleft()

    def _flush(self):
        while self.last is not None and self.pending and self.pending[0][0] <= self.last:
            stamp, number = self.pending.popleft()
            if stamp < self.first:
                continue
            if self.counter is None:
                self.counter = dict(first=number, last=number-1, count=0, non_increasing=0)
            self.counter['non_increasing'] += int(number <= self.counter['last'])
            self.counter['last'] = number
            self.counter['count'] += 1

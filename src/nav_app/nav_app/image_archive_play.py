"""Play a JPEG95 capture through its original sensor Image topics."""
import json
import signal
import subprocess
import sys
import time
from .features.image_archive import BoundedImageWorker, decode_image


def main(argv=None):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.duration import Duration
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image, CompressedImage
    from rosbag2_interfaces.srv import Resume
    args = list(sys.argv[1:] if argv is None else argv)
    split = args.index('--')
    if args[:1] != ['--routes'] or split != 2:
        raise ValueError('Expected --routes JSON -- bag play ...')
    routes, play = json.loads(args[1]), args[split+1:]
    auto_resume = '--start-paused' not in play
    if auto_resume:
        play.insert(3, '--start-paused')
    cv2.setNumThreads(1)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('nav_image_archive_player')
    workers, publishers, subscriptions = [], [], []
    child = None
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        for route in routes:
            publisher = node.create_publisher(Image, route['output_topic'],
                QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE))
            worker = BoundedImageWorker(decode_image, publisher.publish)
            subscriptions.append(node.create_subscription(CompressedImage, route['stored_topic'], worker.submit,
                QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)))
            publishers.append(publisher); workers.append(worker)
        # Isolate signals so a launcher stop is forwarded exactly once. The
        # child wrapper also stops playback if this decoder unexpectedly dies.
        child = subprocess.Popen([sys.executable, '-m', 'nav_app.record_child', *play], start_new_session=True)
        if auto_resume:
            client = node.create_client(Resume, '/rosbag2_player/resume')
            deadline = time.monotonic() + 30
            while not stopping and child.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                if client.service_is_ready() and all(node.count_publishers(r['stored_topic']) for r in routes):
                    future = client.call_async(Resume.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
                    if not future.done() or future.exception():
                        raise RuntimeError('回放启动失败')
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError('回放图像连接超时')
        while child.poll() is None and not stopping:
            rclpy.spin_once(node, timeout_sec=.02)
            for worker in workers:
                status = worker.snapshot()
                if status['error'] or status['dropped']:
                    raise RuntimeError('彩色回放未能完整解码，请降低回放倍率：' + json.dumps(status))
        if stopping and child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=10)
        # Deliver queued DDS samples before closing the FIFO at natural EOF.
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        subscriptions.clear()
        for route, worker, publisher in zip(routes, workers, publishers):
            status = worker.close()
            if status['error'] or status['dropped']:
                raise RuntimeError('彩色回放不完整：' + json.dumps(status))
            publisher.wait_for_all_acked(Duration(seconds=5))
            print('NAV_IMAGE_REPLAY_STATISTICS ' + json.dumps(dict(route=route, **status)), flush=True)
        return 0 if child.returncode in (0, -signal.SIGINT) else child.returncode
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=10)
            except subprocess.TimeoutExpired: child.kill(); child.wait()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        for worker in workers:
            worker.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

"""Synthetic record/flush/audit/play smoke check; never opens a physical device."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, CameraInfo, PointCloud2, PointField
import yaml
from nav_app.features.bag_audit import finalize_recording
from nav_app.features.rosbag import read_bag_metadata, build_play_arguments, PlaybackRequest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lidar-only', action='store_true')
    parser.add_argument('--image-storage', choices=['raw', 'jpeg95'], default='raw')
    options = parser.parse_args()
    root = Path(tempfile.mkdtemp(prefix='nav-recording-smoke-'))
    roles = {name: '/migration_smoke/' + name for name in ('rgb', 'camera_info', 'lidar', 'imu')}
    protocol = root / 'capture.yaml'
    protocol.write_text(yaml.safe_dump(dict(schema_version=1, roles=roles,
        required_roles=['lidar', 'imu'], parameter_nodes=[], max_silence_s=2.)))
    selected = [roles['lidar'], roles['imu']] if options.lidar_only else list(roles.values())
    rclpy.init()
    node = Node('migration_recording_source')
    image_pub = node.create_publisher(Image, roles['rgb'], 10)
    info_pub = node.create_publisher(CameraInfo, roles['camera_info'], 10)
    lidar_pub = node.create_publisher(PointCloud2, roles['lidar'], 10)
    imu_pub = node.create_publisher(Imu, roles['imu'], 100)
    image = Image(height=16, width=16, encoding='rgb8', step=48, data=[80]*768)
    info = CameraInfo(height=16, width=16, k=[10.,0.,8.,0.,10.,8.,0.,0.,1.])
    cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12, data=[0]*12,
        fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(('x','y','z'))])
    imu = Imu()
    imu.linear_acceleration.z = 9.81
    sequence = 0
    def publish():
        nonlocal sequence
        stamp = node.get_clock().now().to_msg()
        imu.header.stamp = stamp
        imu_pub.publish(imu)
        if sequence % 5 == 0:
            for message, publisher in ((image,image_pub),(info,info_pub),(cloud,lidar_pub)):
                message.header.stamp = stamp
                message.header.frame_id = 'smoke_sensor'
                publisher.publish(message)
        sequence += 1
    node.create_timer(.01, publish)
    bag = root / 'bag'
    environment = {**os.environ, 'NAV_CAPTURE_CONFIG': str(protocol), 'PYTHONNOUSERSITE': '1',
                   'NAV_RUNTIME_DIR': str(root / 'runtime')}
    stream = (root / 'recorder.log').open('w')
    child = subprocess.Popen(['ros2','run','nav_app','reliable_record','--image-storage',options.image_storage,'--',
        'bag','record','--output',str(bag),'--storage','sqlite3','--max-cache-size','16777216',
        *selected], env=environment, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        started = None
        deadline = time.monotonic() + 35.
        while time.monotonic() < deadline and child.poll() is None:
            rclpy.spin_once(node, timeout_sec=.01)
            receipt = bag / 'nav_manifest.json'
            if receipt.is_file() and json.loads(receipt.read_text()).get('state') == 'recording':
                started = started or time.monotonic()
                if time.monotonic() - started > 4.:
                    break
        assert started is not None, (root/'recorder.log').read_text()
        os.killpg(child.pid, signal.SIGINT)
        deadline = time.monotonic() + 30.
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert child.poll() == 0, (root/'recorder.log').read_text()
        closed = json.loads((bag/'nav_manifest.json').read_text())
        assert closed['state'] == 'stored' and closed['storage_flush_confirmed']
        result = finalize_recording(bag)
        assert result['state'] == 'complete', result['errors']
        metadata = read_bag_metadata(bag)
        assert metadata.message_count > 100
        # Destroy publishers before replay so only the bag can produce evidence.
        node.destroy_node()
        node = Node('migration_playback_observer')
        received, images = [], []
        node.create_subscription(Imu, roles['imu'], received.append, 100)
        node.create_subscription(Image, roles['rgb'], images.append, 100)
        playback_log = (root/'playback.log').open('w')
        player = subprocess.Popen(['ros2', *build_play_arguments(PlaybackRequest(bag, tuple(selected)))],
            stdout=playback_log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15.
            while time.monotonic() < deadline and player.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
            assert player.poll() == 0 and len(received) > 50, (root/'playback.log').read_text()
        finally:
            if player.poll() is None:
                os.killpg(player.pid, signal.SIGINT)
                player.wait(timeout=10)
            playback_log.close()
        if not options.lidar_only:
            assert len(images) > 10, (root/'playback.log').read_text()
            assert all((image.width, image.height, image.encoding) == (16, 16, 'rgb8') for image in images)
        report = dict(recording=str(bag), messages=metadata.message_count,
            received_images=len(images), image_storage=options.image_storage,
            received_imu=len(received), state=result['state'], storage_flush_confirmed=True,
            lidar_only=options.lidar_only)
        (root/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=30)
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Exercise real ROS/Qt AR entry points with synthetic images and timestamped TF."""
import json
import math
from pathlib import Path as FilePath
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo, Image
from PyQt5.QtWidgets import QApplication

from nav_app.qt_nav_node import QtNavRosNode, NavigationWindow
from nav_app.ar_nav_node import ArNavNode, bgr_to_image
from nav_app.renderer import synthetic_camera_frame


def main():
    root = FilePath(tempfile.mkdtemp(prefix='nav-ar-smoke-'))
    rclpy.init(args=['--ros-args', '-p', 'show_window:=false'])
    app = QApplication.instance() or QApplication([])
    node = QtNavRosNode()
    viewer = ArNavNode()
    observer = Node('nav_ar_smoke_observer')
    received = []
    observer.create_subscription(Image, '/nav/ar_image', received.append, qos_profile_sensor_data)
    window = NavigationWindow(node)
    window.refresh_timer.stop()
    window.show()
    window.pages.setCurrentWidget(window.active_page)
    info = CameraInfo(width=1280, height=720, distortion_model='plumb_bob',
                      k=[500., 0., 639.5, 0., 500., 359.5, 0., 0., 1.])
    frame = synthetic_camera_frame(1280, 720)
    cv2.putText(frame, 'SYNTHETIC TEST / NO HARDWARE', (780, 680),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (240, 240, 240), 2)
    image = bgr_to_image(frame)
    image.header.frame_id = 'camera_optical'
    image.header.stamp = Time(seconds=100.).to_msg()
    for target in (node, viewer):
        clock = target.get_clock()
        clock.set_ros_time_override(Time(seconds=100.1))
        clock._set_ros_time_is_active(True)
        target.projection_mode = 'calibrated'
        if target is node:
            target._on_camera_info(info)
        else:
            target.on_camera_info(info)
        # Camera optical +Z points along body +X; both origins at 0.72 m.
        for parent, child, xyz, q in (
                ('base_footprint', 'base_link', (0., 0., .72), (0., 0., 0., 1.)),
                ('base_link', 'camera_optical', (0., 0., 0.), (-.5, .5, -.5, .5))):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
            rotation = transform.transform.rotation
            rotation.x, rotation.y, rotation.z, rotation.w = q
            target.tf_buffer.set_transform_static(transform, 'synthetic_test')
    results = []
    image_time = 100.
    try:
        for size in ((800, 480), (480, 800), (1280, 900)):
            window.resize(*size)
            app.processEvents()
            for name, yaw, vertices, instruction in (
                    ('forward', 0., [(0., 0.), (14., 0.)], '沿路线前行'),
                    ('left', 0., [(0., 0.), (5., 0.), (5., 5.)], '前方 5 米左转'),
                    ('right', 0., [(0., 0.), (5., 0.), (5., -5.)], '前方 5 米右转'),
                    ('turnaround', math.pi, [(0., 0.), (14., 0.)], '请转身')):
                image_time += 1.
                image.header.stamp = Time(seconds=image_time).to_msg()
                for target in (node, viewer):
                    target.get_clock().set_ros_time_override(Time(seconds=image_time+.1))
                    for stamp, heading in ((image_time, yaw), (image_time+.1, yaw+.7)):
                        transform = TransformStamped()
                        transform.header.stamp = Time(seconds=stamp).to_msg()
                        transform.header.frame_id = 'map'
                        transform.child_frame_id = 'base_footprint'
                        transform.transform.rotation.z = math.sin(heading/2)
                        transform.transform.rotation.w = math.cos(heading/2)
                        target.tf_buffer.set_transform(transform, 'synthetic_test')
                route = Path()
                route.header.frame_id = 'map'
                # Dense planner samples; their arbitrary Z must be ignored for the floor.
                for first, last in zip(vertices[:-1], vertices[1:]):
                    for xy in np.linspace(first, last, 40, endpoint=False):
                        pose = PoseStamped()
                        pose.pose.position.x, pose.pose.position.y = map(float, xy)
                        pose.pose.position.z = 9.
                        route.poses.append(pose)
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = vertices[-1]
                route.poses.append(pose)
                node.navigation_active = True
                node._on_path(route)
                node._on_image(image)
                window.refresh()
                app.processEvents()
                cue = node.guidance_state()
                assert cue['instruction'] == instruction, cue
                assert node.ar_status['code'] == ('outside_view' if name == 'turnaround' else 'ready'), node.ar_status
                if name != 'turnaround':
                    assert not window.active_page.camera_panel._route_arrow_path.isEmpty()
                # The more recent transform must not replace the image-time heading.
                actual = node.lookup_matrix('map', 'base_link', image.header.stamp)
                assert abs(math.atan2(actual[1, 0], actual[0, 0])-yaw) < 1e-6
                shot = root / f'{name}-{size[0]}x{size[1]}.png'
                assert window.grab().save(str(shot))
                results.append(dict(size=size, scenario=name, cue=cue, projection=dict(node.ar_status), screenshot=str(shot)))
                viewer.on_path(route)
                viewer.on_image(image)
                assert viewer.ar_status['code'] == node.ar_status['code']
                assert np.count_nonzero(viewer.compose(frame, 'camera_optical') != frame) > 1000
            image.header.stamp = Time(seconds=image_time-2.).to_msg()
            node._on_image(image)
            window.refresh()
            app.processEvents()
            assert node.ar_status['code'] == 'camera_stale'
            assert not window.active_page.camera_panel._camera_live
            assert window.active_page.camera_panel._route is None
            shot = root / f'stale-{size[0]}x{size[1]}.png'
            assert window.grab().save(str(shot))
            results.append(dict(size=size, scenario='stale', projection=dict(node.ar_status), screenshot=str(shot)))
            image.header.stamp = Time(seconds=image_time).to_msg()
        # Invalid external CameraInfo must not cause the auto mode to draw a virtual route.
        node.projection_mode = 'auto'
        node._on_camera_info(CameraInfo())
        node._on_image(image)
        assert node.projected_route(frame.shape) is None and node.ar_status['code'] == 'calibration'
        node._on_path(Path())
        assert len(node.path) == 0
        deadline = time.monotonic()+3.
        while not any(msg.header.stamp == image.header.stamp for msg in received) and time.monotonic() < deadline:
            viewer.on_image(image)
            rclpy.spin_once(observer, timeout_sec=.05)
        assert received and received[-1].encoding == 'bgr8'
        assert any(msg.header.stamp == image.header.stamp for msg in received)
        (root/'result.json').write_text(json.dumps(dict(cases=results, raw_output_received=len(received),
            source_commit='aafb462ea8f7875205da888946d05c3d31bc8477'), ensure_ascii=False, indent=2))
        print(root/'result.json')
    finally:
        window.close()
        observer.destroy_node()
        viewer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

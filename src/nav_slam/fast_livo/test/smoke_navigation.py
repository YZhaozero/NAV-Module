"""Synthetic static-room integration check. Run in a built ROS 2 workspace.

No physical sensor is started. This validates interfaces, not pose accuracy.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from fast_livo.srv import SaveMap


def main():
    output = Path(tempfile.mkdtemp(prefix="nav-smoke-"))
    log = (output / "mapper.log").open("w")
    config = yaml.safe_load((Path(get_package_share_directory("fast_livo")) /
                             "config/mid360_live.yaml").read_text())
    config["/**"]["ros__parameters"]["navigation.map_spool_directory"] = str(output / "map-spool")
    params = output / "smoke.yaml"
    params.write_text(yaml.safe_dump(config))
    prefix = (["--launch-prefix", "gdb --batch -ex run -ex 'thread apply all bt' --args",
               "--launch-prefix-filter", "fastlivo_mapping"] if os.environ.get("NAV_SMOKE_GDB") else [])
    process = subprocess.Popen(["ros2", "launch", *prefix, "fast_livo", "navigation.launch.py", "img_en:=0", f"params_file:={params}"],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node("navigation_smoke_source")
    lidar_pub = node.create_publisher(CustomMsg, "/livox/lidar", 10)
    imu_pub = node.create_publisher(Imu, "/livox/imu", 100)
    bodies, odoms, maps = [], [], []
    subscriptions = [
        node.create_subscription(PointCloud2, "/fast_livo/cloud_body", bodies.append, 10),
        node.create_subscription(PointCloud2, "/fast_livo/map", maps.append, 10),
        node.create_subscription(Odometry, "/aft_mapped_to_init", odoms.append, 10)]
    client = node.create_client(SaveMap, "/fast_livo/save_map")
    rng = np.random.default_rng(36)
    xyz = rng.uniform([-5, -3, -1], [5, 3, 2], (1800, 3))
    for i, row in enumerate(xyz):
        axis = i % 3
        row[axis] = ([-5, -3, -1] if i % 2 else [5, 3, 2])[axis]
    template = []
    for i, p in enumerate(xyz):
        template.append(CustomPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]),
            reflectivity=100, tag=16, line=i % 6, offset_time=int(i * 99000000 / len(xyz))))
    try:
        assert client.wait_for_service(timeout_sec=15), "FAST-LIVO2 save service did not start"
        start, last_scan = time.monotonic(), -1
        while time.monotonic() - start < 12:
            assert process.poll() is None, "Mapper exited; inspect " + str(output / "mapper.log")
            now = node.get_clock().now()
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = "livox_frame"
            imu.linear_acceleration.z = 9.81
            imu_pub.publish(imu)
            tick = int((time.monotonic() - start) * 10)
            if tick != last_scan:
                cloud = CustomMsg()
                cloud.header.stamp = now.to_msg()
                cloud.header.frame_id = "livox_frame"
                cloud.timebase = now.nanoseconds
                cloud.points = template
                cloud.point_num = len(template)
                lidar_pub.publish(cloud)
                last_scan = tick
            rclpy.spin_once(node, timeout_sec=0.001)
            time.sleep(0.004)
        assert len(bodies) >= 10 and len(odoms) >= 10 and maps, "Missing navigation outputs"
        assert all(p.header.frame_id == "base_link" for p in bodies)
        assert all(p.header.frame_id == "odom" and p.child_frame_id == "base_link" for p in odoms)
        assert all(p.header.frame_id == "odom" for p in maps)
        stamp = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)
        paired = set(map(stamp, bodies)) & set(map(stamp, odoms))
        assert len(paired) >= 10, "Cloud and odometry timestamps do not agree"
        path = output / "room.pcd"
        request = SaveMap.Request(save_path=str(path), leaf_size=0.2)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result().success, (
            "Map save failed: " + (future.result().message if future.done() else "no response")
            + "; logs: " + str(output / "mapper.log"))
        assert path.is_file() and path.stat().st_size > 100
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and not future.result().success, "Existing file was overwritten"
        result = {"synthetic": True, "body_messages": len(bodies), "odom_messages": len(odoms),
                  "map_messages": len(maps), "exact_timestamp_pairs": len(paired),
                  "saved_bytes": path.stat().st_size, "overwrite_rejected": True,
                  "output_directory": str(output)}
        (output / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()


if __name__ == "__main__":
    main()

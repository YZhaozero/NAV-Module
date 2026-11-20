"""Execute the confirmed MeshNav preview using the existing WOOSH motion gate."""
from collections import deque
import math
import threading
import time

import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .mesh_follow import FootprintError, MeshFollower
from ..map_processing import pointcloud2_to_xyz


class MeshDrive:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, "/nav/planner_cmd_vel", 1)
        self._lock = threading.RLock()
        self.phase = "idle"
        self.follower = None
        self.generation = 0
        # ROS receives on the TF executor; tracking reads a snapshot of the newest frames.
        self.clouds = deque(maxlen=3)
        self.surface = None
        self.after_stop = "pause"
        self.reason = ""
        self.timer = node.motion_node.create_timer(.10, self.tick, clock=node.get_clock())

    def destroy(self):
        with self._lock:
            self.phase = "closed"
            self.zero()
            self.node.motion_node.destroy_timer(self.timer)
            self.node.destroy_publisher(self.publisher)

    def zero(self):
        self.node.motion_permit_pub.publish(Bool(data=False))
        self.publisher.publish(Twist())
        self.node.motion_session.poll(("mesh", self.generation), False)

    def stop(self, reason, after="pause"):
        with self._lock:
            if self.phase == "running":
                self.node.get_logger().info(f"MeshNav 停车：{reason}")
            if self.phase != "stopping":
                self.stop_stamp = self.node.get_clock().now().nanoseconds
            self.phase, self.reason, self.after_stop = "stopping", reason, after
            self.zero()
            self.node.navigation_active = False
            self.node.navigation_status = reason + "，等待底盘静止反馈"

    def command(self):
        n = self.node
        if not n.navigation_route_ready():
            raise ValueError(n.navigation_readiness_reason)
        reason = n.motion_session.unavailable_reason()
        if reason:
            raise ValueError(reason)
        state = n.motion_session.state
        measured = state.get("measured_velocity")
        if measured is None:
            raise ValueError("等待底盘实测速度")
        frames = tuple(self.clouds)
        if not frames:
            raise ValueError("等待实时三维避障点云")
        pose = n.lookup_matrix(n.map_frame, n.base_frame)
        map_from_odom = n.lookup_matrix(n.map_frame, "odom")
        if pose is None or map_from_odom is None:
            raise ValueError("等待地图定位坐标变换")

        # Use the acquisition-time odometry for the cloud and the same map correction
        # as the vehicle. A slightly newer cloud can precede its odometry transform.
        now = n.get_clock().now().nanoseconds
        for cloud in reversed(frames):
            stamp = cloud.header.stamp
            stamp_ns = stamp.sec * 10**9 + stamp.nanosec
            if not -.05 <= (now - stamp_ns) / 1e9 <= .30:
                raise ValueError("三维避障点云已过期或时钟不一致")
            if not cloud.header.frame_id or not cloud.width or not cloud.height:
                raise ValueError("三维避障点云为空或缺少坐标系")
            odom_from_cloud = n.lookup_matrix("odom", cloud.header.frame_id, stamp)
            if odom_from_cloud is not None:
                break
        else:
            raise ValueError("等待点云采集时刻的里程计坐标变换")
        xyz = pointcloud2_to_xyz(cloud, max_points=cloud.width * cloud.height)
        if not len(xyz):
            raise ValueError("三维避障点云没有有效测距")
        transform = map_from_odom @ odom_from_cloud
        # Crop in the sensor frame first. The box encloses the full swept
        # volume, including overhead clearance and tilted sensors, without sampling.
        center_sensor = (pose[:3, 3] - transform[:3, 3]) @ transform[:3, :3]
        body_axes = pose[:3, :2].T @ transform[:3, :3]
        body_xy = np.einsum("ij,kj->ik", xyz - center_sensor, body_axes)
        # The self-filtered scan may have no returns in open directions.
        if not np.any(np.einsum("ij,ij->i", body_xy, body_xy) > .08**2):
            raise ValueError("三维避障点云没有车身中心 8 厘米以外的有效测距")
        near = np.all(np.abs(xyz - center_sensor) <= 2.5, axis=1)
        nearby = xyz[near] @ transform[:3, :3].T + transform[:3, 3]
        if n._meshnav_surface is not self.surface:
            raise ValueError("导航网格已变更，请重新规划路线")
        support = n.meshnav_start()
        if support is None:
            center = pose[:3, 3]
            raise FootprintError((np.empty((0, 3)), center[None, :]), "小车已离开可通行网格，请查看红色标记")
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        if abs(measured[0]) > .16 or abs(measured[2]) > .50 or measured[0] < -.03:
            raise ValueError("底盘实测速度超出低速跟踪范围")
        command = self.follower.command(support[0], yaw, self.surface, support[1], nearby, measured)
        if n.get_clock().now().nanoseconds - stamp_ns > 300_000_000:
            raise ValueError("三维避障点云已过期")
        return command

    def begin(self):
        with self._lock:
            n = self.node
            if self.phase not in ("idle", "paused") or n.walking.phase != "preview":
                return False
            reason = n.motion_session.unavailable_reason()
            if reason or not n.motion_session.state.get("ready_to_start"):
                n.navigation_status = reason or n.motion_session.base_reason() or "等待底盘模式、定位和静止反馈"
                return False
            try:
                if self.phase == "idle":
                    self.follower = MeshFollower(n.walking.path)
                self.command()
                n.walking.body_conflict = None
            except ValueError as error:
                if isinstance(error, FootprintError):
                    n.walking.body_conflict = error.conflict
                n.navigation_status = str(error)
                return False
            self.generation += 1
            self.phase = "running"
            self.last_motion = time.monotonic()
            n.navigation_active = True
            n.navigation_status = "自动行驶准备中"
            return True

    def tick(self):
        with self._lock:
            n = self.node
            if self.phase == "stopping":
                self.zero()
                state = n.motion_session.state
                if (n.motion_session.fresh() and state.get("stop_confirmed")
                        and state["stamp_ns"] > self.stop_stamp + 300_000_000):
                    self.phase = "paused" if self.after_stop == "pause" else "idle"
                    n.navigation_status = self.reason + "，底盘已静止"
                    if self.after_stop in ("exit", "arrived"):
                        n.walking.cancel(announce=False)
                        n._set_navigation_terminal(4 if self.after_stop == "arrived" else 5, n.navigation_status)
                        n.walking.result_revision = n.navigation_result_revision
                    elif self.after_stop == "manual":
                        n.set_woosh_control_mode(0)
                        n._woosh_motion_enabled = False
                        n.navigation_status = "底盘已静止，已切至手柄引导"
                return
            if self.phase != "running":
                return
            try:
                command = self.command()
            except ValueError as error:
                if isinstance(error, FootprintError):
                    n.walking.body_conflict = error.conflict
                self.stop(str(error))
                return
            if command is None:
                self.stop("已到达目的地", "arrived")
                return
            reason = n.motion_session.unavailable_reason()
            if reason:
                self.stop(reason)
                return
            n.motion_permit_pub.publish(Bool(data=True))
            error = n.motion_session.poll(("mesh", self.generation), True)
            if error:
                self.stop(error)
                return
            velocity = Twist()
            if n.motion_session.armed:
                measured = n.motion_session.state["measured_velocity"]
                if abs(measured[0]) >= .008 or abs(measured[2]) >= .025:
                    self.last_motion = time.monotonic()
                elif time.monotonic() - self.last_motion > 5.:
                    self.stop("底盘持续无运动反馈")
                    return
                velocity.linear.x, velocity.angular.z = map(float, command)
                remaining = self.follower.distance[-1] - self.follower.progress
                n.navigation_status = f"自动行驶 · 剩余 {remaining:.1f} m"
            self.publisher.publish(velocity)

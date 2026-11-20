"""Pedestrian wayfinding: planning and instructions, never motion commands."""
import math
import time
import numpy as np


class WalkingGuidance:
    def __init__(self, client, changed, clock=time.monotonic, ready=None, arrival_radius=.8):
        self.client, self.changed, self.clock = client, changed, clock
        self.ready = ready
        self.arrival_radius = arrival_radius
        self.generation = 0
        self.phase = "idle"
        self.status = "选择目的地，规划步行路线"
        self.path = np.empty((0, 3), dtype=np.float32)
        self.frame = "map"
        self._dimensions = 2
        self.handle = None
        self.deadline = None
        self.remaining = 0.
        self.progress = 0.
        self.direction = "等待路线"
        self.result = None
        self.result_revision = 0
        self._arrival_since = None
        self.body_conflict = None
        self._goals = []
        self._segments = []

    def notify(self, text):
        self.status = text
        self.changed(self)

    def plan(self, goal, *, remaining_goals=(), dimensions=2, surface=None, body_component=None, start_yaw=None):
        if not self.client.server_is_ready() or self.ready is not None and not self.ready():
            self.notify("路线服务未就绪，请先准备设备")
            return False
        self.cancel(announce=False)
        token = self.generation
        self.phase, self.result = "planning", None
        self._goals = list(remaining_goals)
        self._dimensions = dimensions
        self._surface = surface
        self._body_component = body_component
        self._body_yaw = start_yaw
        self.deadline = self.clock() + 12.
        self.notify("正在规划路线，请稍候")
        return self._request(token, goal)

    def _request(self, token, goal):
        try:
            if self._surface is not None:
                position = goal.start_pose.pose.position
                self._start = np.array([position.x, position.y, position.z], dtype=float)
                position = goal.target_pose.pose.position
                self._target = np.array([position.x, position.y, position.z], dtype=np.float32)
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda reply: self._accepted(token, reply))
        except Exception as exc:
            self._fail(token, f"路线请求失败：{exc}")
            return False
        return True

    def _accepted(self, token, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(token, f"路线请求失败：{exc}")
            return
        if token != self.generation or self.phase != "planning":
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass  # This is an obsolete read-only planning request.
            return
        if not handle.accepted:
            self._fail(token, "路线无法规划，请更换目的地或检查地图")
            return
        self.handle = handle
        try:
            handle.get_result_async().add_done_callback(lambda result: self._planned(token, result))
        except Exception as exc:
            self._fail(token, f"路线获取失败：{exc}")

    def _planned(self, token, future):
        if token != self.generation or self.phase != "planning":
            return
        try:
            reply = future.result()
            path = reply.result.path
            points = np.asarray([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                 for p in path.poses], dtype=np.float32)
            if (reply.status != 4 or getattr(reply.result, "outcome", 0) != 0
                    or len(points) < 2 or not np.isfinite(points).all()):
                raise ValueError("目的地不可达或路线为空")
            if path.header.frame_id != self.frame:
                raise ValueError("路线与地图坐标不一致")
            # The surface planner's last pose is a graph vertex. Include the selected
            # point only when the final segment lies on the same traversable surface.
            if self._surface is not None and not np.allclose(points[-1], self._target, atol=1.e-5, rtol=0):
                goal = self._surface.point(self._target, max_height_error=.04)
                length = np.linalg.norm(self._target - points[-1])
                connector = np.linspace(points[-1], self._target, int(np.ceil(length / .04)) + 1)
                if goal is None or not self._surface.covers(connector[:, :2], connector[:, 2], goal[1], .04):
                    raise ValueError("规划路线无法沿可通行网格连接到所选终点")
                points = np.vstack((points, self._target))
            if self._body_component is not None:
                if not np.allclose(points[0], self._start, atol=1.e-5, rtol=0):
                    points = np.vstack((self._start, points))
                conflict = self._surface.body_conflict(points, self._body_component, start_yaw=self._body_yaw)
                if conflict is not None:
                    self._fail(token, f"第 {len(self._segments)+1} 段路线车体越界，请查看地图红叉", conflict)
                    return
                directions = np.diff(points[:, :2], axis=0)
                moving = directions[np.any(directions != 0., axis=1)]
                if len(moving):
                    self._body_yaw = math.atan2(moving[-1, 1], moving[-1, 0])
        except Exception as exc:
            self._fail(token, f"未找到可用路线：{exc}")
            return
        if self._segments and np.allclose(self._segments[-1][-1], points[0], atol=1.e-4):
            points = points[1:]
        self._segments.append(points)
        if self._goals:
            self.handle = None
            self._request(token, self._goals.pop(0))
            return
        points = np.concatenate(self._segments)
        self._segments.clear()
        self.path = points
        self._lengths = np.linalg.norm(np.diff(points[:, :self._dimensions], axis=0), axis=1)
        self._distances = np.r_[0., np.cumsum(self._lengths)]
        self.remaining = float(self._distances[-1])
        self.progress = 0.
        self.phase, self.handle, self.deadline = "preview", None, None
        self.notify(f"路线约 {self.remaining:.0f} 米，请确认后开始引导")

    def begin(self):
        if self.phase != "preview":
            return False
        self.phase = "guiding"
        self._arrival_since = None
        self.notify("沿路线前行，请留意周围环境")
        return True

    def update_pose(self, xy, yaw):
        if self.phase not in ("guiding", "lost", "deviated"):
            return
        if xy is None:
            self.phase, self.direction = "lost", "请停下确认位置"
            self._arrival_since = None
            self.notify("定位暂时中断，请保持设备稳定")
            return
        position = np.asarray(xy)[:self._dimensions]
        # Restrict matching to local route progress; crossings must not skip ahead.
        route = self.path[:, :self._dimensions]
        starts, delta = route[:-1], np.diff(route, axis=0)
        fraction = np.clip(np.sum((position - starts) * delta, axis=1) / np.maximum(self._lengths ** 2, 1e-8), 0., 1.)
        projected = starts + fraction[:, None] * delta
        route_distance = self._distances[:-1] + fraction * self._lengths
        lateral = np.linalg.norm(projected - position, axis=1)
        eligible = (route_distance >= self.progress - .75) & (route_distance <= self.progress + 3.)
        lateral[~eligible] = np.inf
        index = int(np.argmin(lateral))
        if lateral[index] > 1.5:
            self.phase, self.direction = "deviated", "请停下确认路线"
            self._arrival_since = None
            self.notify("已偏离路线，可从当前位置重新规划")
            return
        self.phase = "guiding"
        self.progress = max(self.progress, float(route_distance[index]))
        self.remaining = max(0., float(self._distances[-1]) - self.progress)
        target_distance = min(float(self._distances[-1]), self.progress + 1.5)
        target = np.array([np.interp(target_distance, self._distances, self.path[:, axis]) for axis in (0, 1)])
        vector = target - position[:2]
        angle = math.atan2(math.sin(math.atan2(vector[1], vector[0]) - yaw),
                           math.cos(math.atan2(vector[1], vector[0]) - yaw))
        self.direction = ("请转身" if abs(angle) > 2.2 else
                          "向左转" if angle > .6 else "向右转" if angle < -.6 else "沿路线前行")
        near = (self.remaining < self.arrival_radius
                and np.linalg.norm(position - route[-1]) < self.arrival_radius)
        self._arrival_since = (self._arrival_since if self._arrival_since is not None else self.clock()) if near else None
        if self._arrival_since is not None and self.clock() - self._arrival_since >= 1.0:
            self.phase, self.direction, self.result = "arrived", "已到达目的地", 4
            self.result_revision += 1
            self.notify("已到达目的地")
        else:
            self.notify(f"{self.direction} · 剩余约 {self.remaining:.0f} 米")

    def cancel(self, announce=True):
        self.generation += 1
        self.body_conflict = None
        if self.handle is not None:
            try:
                self.handle.cancel_goal_async()
            except Exception:
                pass  # Cancel computation only; it cannot drive a robot.
        self.phase, self.handle, self.deadline = "idle", None, None
        self._goals.clear()
        self._segments.clear()
        self.path = np.empty((0, 3), dtype=np.float32)
        self.direction = "引导已结束"
        if announce:
            self.notify("引导已结束，目的地已保留")

    def _fail(self, token, text, conflict=None):
        if token != self.generation or self.phase != "planning":
            return
        self.cancel(announce=False)
        self.body_conflict = conflict
        self.phase, self.result = "error", -1
        self.result_revision += 1
        self.notify(text)

    def poll(self):
        if self.phase == "planning" and self.deadline is not None and self.clock() > self.deadline:
            self._fail(self.generation, "路线规划超时，可重试；目的地已保留")

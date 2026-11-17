"""ROS 2 node composing camera, global path, occupancy map and navigation HUD."""

from __future__ import annotations

import math
import os
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .renderer import (
    RenderStyle,
    cumulative_distance,
    draw_hud,
    draw_minimap,
    draw_route,
    make_ribbon,
    project_ground,
    project_optical,
    synthetic_camera_frame,
    waiting_camera_frame,
)


def transform_matrix(transform) -> np.ndarray:
    t = transform.translation
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        rotation = np.eye(3)
    else:
        s = 2.0 / n
        rotation = np.array([
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def apply_transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert common ROS image encodings without depending on cv_bridge."""
    enc = msg.encoding.lower()
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(enc)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    packed = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == "mono8":
        return cv2.cvtColor(packed[:, :, 0], cv2.COLOR_GRAY2BGR)
    if enc == "rgb8":
        return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(packed, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(packed, cv2.COLOR_BGRA2BGR)
    return packed.copy()


def bgr_to_image(image: np.ndarray, source: Optional[Image] = None) -> Image:
    msg = Image()
    if source is not None:
        msg.header = source.header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


class ArNavNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_ar_nav")
        self.declare_parameter("camera_topic", "/color/image_raw")
        self.declare_parameter("camera_info_topic", "/color/camera_info")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("output_topic", "/nav/ar_image")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ground_frame", "base_footprint")
        self.declare_parameter("ground_height_m", .72)
        self.declare_parameter("ground_render_offset_m", .015)
        self.declare_parameter("route_near_clip_m", .45)
        self.declare_parameter("route_far_clip_m", 15.)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("projection_mode", "auto")
        self.declare_parameter("route_width_m", 0.95)
        self.declare_parameter("camera_height_m", 0.72)
        self.declare_parameter("horizon_ratio", 0.43)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("demo_mode", False)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ground_frame = str(self.get_parameter("ground_frame").value)
        self.ground_height_m = float(self.get_parameter("ground_height_m").value)
        self.ground_render_offset_m = float(self.get_parameter("ground_render_offset_m").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.camera_frame_override = str(self.get_parameter("camera_frame").value)
        self.projection_mode = str(self.get_parameter("projection_mode").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.demo_mode = bool(self.get_parameter("demo_mode").value)
        self.style = RenderStyle(
            ribbon_width_m=float(self.get_parameter("route_width_m").value),
            route_near_clip_m=float(self.get_parameter("route_near_clip_m").value),
            route_far_clip_m=float(self.get_parameter("route_far_clip_m").value),
            camera_height_m=float(self.get_parameter("camera_height_m").value),
            horizon_ratio=float(self.get_parameter("horizon_ratio").value),
        )
        if not (0 <= self.style.route_near_clip_m < self.style.route_far_clip_m
                and math.isfinite(self.style.route_far_clip_m) and self.style.ribbon_width_m > 0
                and math.isfinite(self.style.ribbon_width_m) and self.ground_height_m >= 0
                and math.isfinite(self.ground_height_m) and math.isfinite(self.ground_render_offset_m)):
            raise ValueError("AR projection configuration is invalid")
        if self.show_window and not os.environ.get("DISPLAY"):
            self.get_logger().warning("DISPLAY is not set; disabling the OpenCV preview window")
            self.show_window = False
        if self.show_window:
            try:
                cv2.namedWindow("NAV 导航画面", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    "NAV 导航画面", self.window_width, self.window_height)
                if self.fullscreen:
                    cv2.setWindowProperty(
                        "NAV 导航画面", cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN)
            except cv2.error as exc:
                self.get_logger().warning(
                    f"Cannot create preview window; continuing topic output only: {exc}")
                self.show_window = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output_pub = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), sensor_qos)
        self.create_subscription(
            Image, str(self.get_parameter("camera_topic").value), self.on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info, sensor_qos)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value),
            self.on_map, map_qos)
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.on_path, reliable_qos)

        self.path = np.empty((0, 3), dtype=np.float32)
        self.path_frame = self.map_frame
        self.occupancy: Optional[np.ndarray] = None
        self.grid_frame = self.map_frame
        self.map_origin_xy = np.zeros(2, dtype=np.float32)
        self.map_origin_yaw = 0.0
        self.map_resolution = 0.0
        self.navigation_active = False
        self.require_localizer_quality = False  # This entry is a path/image viewer, not an action client.
        self.image_stamp = None
        self._image_received_at = 0.
        self._path_received_at = 0.
        self._guidance_progress = 0.
        self.camera_frame = ""
        self.camera_error = self.path_error = ""
        self.camera_calibration_error = ""
        self.camera_distortion = ()
        self.camera_distortion_model = "plumb_bob"
        self.ar_status = dict(code="camera_time", message="等待相机时间")
        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_info_size: Optional[Tuple[int, int]] = None
        self.last_real_image_ns = 0
        self.last_tf_warning_ns = 0
        self.camera_topic = str(self.get_parameter("camera_topic").value)

        if self.demo_mode:
            self.map_frame = self.base_frame
            self.ground_frame = ""
            self.navigation_active = True
            self.path = self._demo_path()
            self.path_frame = self.base_frame
            self.occupancy = self._demo_grid()
            self.grid_frame = self.base_frame
            self.map_origin_xy = np.array([-10.0, -12.0], dtype=np.float32)
            self.map_resolution = 0.10
        # Always render an idle frame. This makes ar_nav.launch.py open its UI
        # immediately, even when the camera driver is still starting.
        self.idle_timer = self.create_timer(0.1, self.render_idle)
        self.get_logger().info(
            f"NAV AR navigation ready: camera={self.get_parameter('camera_topic').value}, "
            f"map={self.get_parameter('map_topic').value}, "
            f"path={self.get_parameter('path_topic').value}, "
            f"output={self.get_parameter('output_topic').value}")

    @staticmethod
    def _demo_path() -> np.ndarray:
        x = np.linspace(0.7, 22.0, 70, dtype=np.float32)
        y = 0.8 * np.sin((x - 4.0) / 5.0)
        return np.column_stack((x, y, np.zeros_like(x)))

    @staticmethod
    def _demo_grid() -> np.ndarray:
        grid = np.zeros((240, 360), dtype=np.int8)
        grid[:8, :] = 100
        grid[-8:, :] = 100
        grid[:, :8] = 100
        grid[:, -8:] = 100
        grid[25:95, 85:105] = 100
        grid[145:220, 205:225] = 100
        grid[70:88, 255:335] = 100
        return grid

    def on_camera_info(self, msg: CameraInfo) -> None:
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] > 0 and matrix[1, 1] > 0 and np.isfinite(matrix).all() and msg.width > 0 and msg.height > 0:
            self.camera_matrix = matrix
            self.camera_info_size = (msg.width, msg.height)
            self.camera_calibration_error = ""
            self.camera_distortion = tuple(msg.d)
            self.camera_distortion_model = msg.distortion_model or "plumb_bob"
        else:
            self.camera_matrix = None
            self.camera_info_size = None
            self.camera_calibration_error = "相机内参或分辨率无效，请检查 CameraInfo"

    def on_path(self, msg: Path) -> None:
        self._guidance_progress = 0.
        self._path_received_at = time.monotonic()
        self.path_error = ""
        if not msg.poses:
            self.navigation_active = False
            self.path = np.empty((0, 3), dtype=np.float32)
            return
        self.path = np.array([
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ], dtype=np.float32)
        self.navigation_active = len(self.path) >= 2
        if not np.isfinite(self.path).all():
            self.path = np.empty((0, 3), dtype=np.float32)
            self.path_error = "规划路线包含无效坐标"
        self.path_frame = (
            msg.header.frame_id or msg.poses[0].header.frame_id or self.map_frame)

    def on_map(self, msg: OccupancyGrid) -> None:
        expected = int(msg.info.width * msg.info.height)
        data = np.asarray(msg.data, dtype=np.int8)
        if expected == 0 or len(data) != expected:
            self.get_logger().warning("Ignoring malformed OccupancyGrid message")
            return
        self.occupancy = data.reshape(msg.info.height, msg.info.width).copy()
        self.grid_frame = msg.header.frame_id or self.map_frame
        self.map_resolution = float(msg.info.resolution)
        origin = msg.info.origin
        self.map_origin_xy = np.array(
            [origin.position.x, origin.position.y], dtype=np.float32)
        q = origin.orientation
        self.map_origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def lookup_matrix(self, target: str, source: str, stamp=None) -> Optional[np.ndarray]:
        if target == source:
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp) if stamp is not None else Time())
            return transform_matrix(tf.transform)
        except TransformException as exc:
            now = self.get_clock().now().nanoseconds
            if now - self.last_tf_warning_ns > 5_000_000_000:
                self.get_logger().warning(f"Waiting for TF {target} <- {source}: {exc}")
                self.last_tf_warning_ns = now
            return None

    def _path_in_frame(self, target: str) -> Optional[np.ndarray]:
        if not len(self.path):
            return self.path.copy()
        matrix = self.lookup_matrix(target, self.path_frame)
        return None if matrix is None else apply_transform(self.path, matrix)

    def _grounded_route(self, stamp):
        from .ar_projection import grounded_route
        return grounded_route(self, stamp)

    def _project_route(self, image: np.ndarray, camera_frame: str):
        from .ar_projection import projected_route
        self.camera_frame = camera_frame
        return projected_route(self, image.shape)

    def _minimap_data(self):
        if self.occupancy is None:
            return None, None, 0.0
        robot_xy = None
        robot_yaw = 0.0
        path_xy = self.path[:, :2] if len(self.path) else None
        common_frame = self.grid_frame
        robot_tf = self.lookup_matrix(common_frame, self.base_frame)
        if robot_tf is not None:
            robot_xy = robot_tf[:2, 3]
            robot_yaw = math.atan2(robot_tf[1, 0], robot_tf[0, 0])
        elif self.path_frame == self.base_frame:
            robot_xy = np.zeros(2, dtype=np.float32)
            common_frame = self.base_frame
        if path_xy is not None and self.path_frame != common_frame:
            matrix = self.lookup_matrix(common_frame, self.path_frame)
            path_xy = None if matrix is None else apply_transform(self.path, matrix)[:, :2]
        return path_xy, robot_xy, robot_yaw

    def compose(self, image: np.ndarray, camera_frame: str) -> np.ndarray:
        output = image.copy()
        projected = self._project_route(output, camera_frame)
        if projected is not None:
            draw_route(output, *projected, self.style)
        path_xy, robot_xy, robot_yaw = self._minimap_data()
        draw_minimap(
            output, self.occupancy, self.map_origin_xy, self.map_resolution,
            self.map_origin_yaw, path_xy, robot_xy, robot_yaw)
        from .navigation_display import guidance_state
        cue = guidance_state(self)
        remaining = cue['remaining_m'] or 0.
        status = "DEMO" if self.demo_mode else "APPROX" if self.ar_status['code']=='approximate' else "LIVE" if projected is not None else "WAIT"
        draw_hud(output, remaining, status, cue)
        return output

    def publish_and_show(
        self, output: np.ndarray, source: Optional[Image] = None
    ) -> None:
        msg = bgr_to_image(output, source)
        if source is None:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame_override or self.base_frame
        self.output_pub.publish(msg)
        if self.show_window:
            cv2.imshow("NAV 导航画面", output)
            cv2.waitKey(1)

    def on_image(self, msg: Image) -> None:
        try:
            image = image_to_bgr(msg)
        except (ValueError, TypeError) as exc:
            self.camera_error = f"Cannot decode camera image: {exc}"
            self.get_logger().error(self.camera_error)
            return
        self.last_real_image_ns = self.get_clock().now().nanoseconds
        self.image_stamp = msg.header.stamp
        self._image_received_at = time.monotonic()
        self.camera_error = ""
        camera_frame = self.camera_frame_override or msg.header.frame_id
        self.publish_and_show(self.compose(image, camera_frame), msg)

    def render_idle(self) -> None:
        if self.get_clock().now().nanoseconds - self.last_real_image_ns < 1_000_000_000:
            return
        if self.demo_mode:
            self.image_stamp = self.get_clock().now().to_msg()
            self._image_received_at = self._path_received_at = time.monotonic()
            image = synthetic_camera_frame(self.window_width, self.window_height)
        else:
            image = waiting_camera_frame(
                self.camera_topic, self.window_width, self.window_height)
        self.publish_and_show(self.compose(image, ""))

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

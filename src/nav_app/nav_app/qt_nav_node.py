"""Interactive Qt dashboard for camera and Nav2 navigation control."""

from __future__ import annotations

import math
import re
import signal
import sys
import time
from pathlib import Path as FilePath
from typing import Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from PyQt5.QtCore import (
    QEvent,
    QPointF,
    QRectF,
    Qt,
    QTimer,
    pyqtSignal,
)
from PyQt5.QtGui import (
    QColor,
    QFont,
    QImage,
    QMouseEvent,
    QKeySequence,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QTransform,
)
from PyQt5.QtWidgets import (
    QApplication,
    QBoxLayout,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSplitter,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .map_processing import (
    GridMap,
    PointCloudMap,
    load_grid_map,
    load_pointcloud,
    pointcloud2_to_xyz,
    pointcloud_to_grid,
    save_grid_map,
    save_pcd,
)
from .map_storage import default_map_directory, unique_map_file

from .nav_math import (
    RenderStyle,
    apply_transform,
    cumulative_distance,
    image_to_bgr,
    make_ribbon,
    occupancy_to_bgr,
    project_ground,
    project_optical,
    transform_matrix,
)
from .features.mapping import (
    MappingController,
    MappingRosAdapter,
    UnavailableMappingRosAdapter,
)
from .features.navigation_stack import NavigationStackController
from .transform_listener import DisplayTransformListener
from .latest_worker import LatestWorker
from .features.localization_tools import (
    LocalizationRosAdapter,
    LocalizationToolsController,
    UnavailableLocalizationRosAdapter,
)
from .features.sensors import (
    SensorDriverController,
    SensorPreviewRosAdapter,
    UnavailableSensorPreviewAdapter,
)
from .ui.mapping_page import MappingPage
from .ui.mapping_workspace import MappingWorkspace
from .ui.page_header import PageHeader
from .ui.navigation_stack_page import NavigationStackPage


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def yaw_quaternion(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class CameraPanel(QWidget):
    """Raw camera display with a resolution-independent Qt vector overlay."""

    def __init__(
        self,
        show_status: bool = True,
        compact: bool = False,
        fill: bool = False,
        camera_topic: str = "/color/image_raw",
        rotate_180: bool = False,
    ) -> None:
        super().__init__()
        self._rotate_180 = rotate_180
        self._show_status = show_status
        self._fill = bool(fill)
        self._camera_topic = camera_topic
        if compact:
            self.setMinimumSize(280, 160)
        else:
            self.setMinimumSize(640, 420)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pixmap: Optional[QPixmap] = None
        self._frame_revision: Optional[int] = None
        self._source_size = (1280, 720)
        self._route = None
        self._route_geometry_key = None
        self._route_fill_path = QPainterPath()
        self._route_center_path = QPainterPath()
        self._route_arrow_path = QPainterPath()
        self._guidance = None
        self.projection_status = ""
        self.status_right_margin = 0
        self._frame_message = "等待相机连接"
        self._distance = 0.0
        self._camera_live = False

    def set_frame(
        self,
        frame_bgr: Optional[np.ndarray],
        route,
        remaining_m: float,
        frame_revision: Optional[int] = None,
        *, guidance=None, projection_status="",
    ) -> None:
        new_frame = frame_bgr is not None and (
            frame_revision is None or frame_revision != self._frame_revision)
        if new_frame:
            frame = np.ascontiguousarray(frame_bgr)
            h, w = frame.shape[:2]
            bgr_format = getattr(QImage, "Format_BGR888", None)
            if bgr_format is not None:
                qimage = QImage(
                    frame.data, w, h, frame.strides[0], bgr_format).copy()
            else:
                rgb = np.ascontiguousarray(frame[:, :, ::-1])
                qimage = QImage(
                    rgb.data, w, h, rgb.strides[0],
                    QImage.Format_RGB888,
                ).copy()
            self._pixmap = QPixmap.fromImage(qimage)
            self._source_size = (w, h)
            self._camera_live = True
            self._frame_revision = frame_revision
        if route is not self._route:
            self._route_geometry_key = None
        self._route = route
        self._distance = remaining_m
        self._guidance = guidance
        self.projection_status = projection_status
        self.update()

    def clear_frame(self, camera_topic: Optional[str] = None) -> None:
        if camera_topic is not None:
            self._camera_topic = camera_topic
        self._pixmap = None
        self._frame_revision = None
        self._route = None
        self._route_geometry_key = None
        self._route_fill_path = QPainterPath()
        self._route_center_path = QPainterPath()
        self._route_arrow_path = QPainterPath()
        self._camera_live = False
        self.update()

    def set_status_visible(self, visible: bool) -> None:
        """Show or hide the navigation card without rebuilding the panel."""
        self._show_status = bool(visible)
        self.update()

    def _video_rect(self) -> QRectF:
        source_w, source_h = self._source_size
        scale_function = max if self._fill else min
        scale = scale_function(
            self.width() / source_w, self.height() / source_h)
        width = source_w * scale
        height = source_h * scale
        return QRectF(
            (self.width() - width) * 0.5,
            (self.height() - height) * 0.5,
            width,
            height,
        )

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#090d12"))
        target = self._video_rect()
        painter.save()
        if self._rotate_180 and self._pixmap is not None:
            center = target.center()
            painter.translate(center)
            painter.rotate(180)
            painter.translate(-center)
        if self._pixmap is not None:
            painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
            painter.drawPixmap(target, self._pixmap, QRectF(self._pixmap.rect()))
        else:
            painter.setPen(QColor("#e5ebef"))
            compact = min(self.width(), self.height()) < 700
            painter.setFont(QFont(
                "Sans Serif",
                8 if compact and self._show_status else (11 if compact else 20),
                QFont.DemiBold,
            ))
            if compact and self._show_status:
                message = "等待相机"
                message_rect = QRectF(self.rect()).adjusted(6, 56, -6, -3)
            elif compact:
                topic_parts = [
                    part for part in self._camera_topic.split("/") if part]
                short_topic = topic_parts[-1] if topic_parts else self._camera_topic
                if len(topic_parts) > 1:
                    short_topic = "…/" + short_topic
                message = f"等待相机\n{short_topic}"
                message_rect = QRectF(self.rect()).adjusted(8, 4, -8, -4)
            else:
                message = f"等待相机 {self._camera_topic}"
                message_rect = target.adjusted(8, 4, -8, -4)
            painter.drawText(
                message_rect,
                Qt.AlignCenter | Qt.TextWordWrap,
                message,
            )

        if self._route is not None:
            self._draw_route(painter, target)
        painter.restore()
        if self._show_status:
            self._draw_status(painter, target)

    def _map_video_point(self, point: np.ndarray, target: QRectF) -> QPointF:
        source_w, source_h = self._source_size
        return QPointF(
            target.left() + float(point[0]) * target.width() / source_w,
            target.top() + float(point[1]) * target.height() / source_h,
        )

    def _draw_route(self, painter: QPainter, target: QRectF) -> None:
        self._ensure_route_geometry(target)
        if self._route_fill_path.isEmpty():
            return
        outline = QPen(QColor(70, 220, 255, 230), 2.0)
        outline.setCapStyle(Qt.RoundCap)
        outline.setJoinStyle(Qt.RoundJoin)
        painter.setPen(outline)
        painter.setBrush(QColor(0, 150, 230, 118))
        painter.drawPath(self._route_fill_path)
        center_pen = QPen(QColor(146, 238, 255, 220), 1.2)
        center_pen.setCapStyle(Qt.RoundCap)
        center_pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(center_pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self._route_center_path)
        arrow_pen = QPen(QColor(225, 253, 255, 245), 3.0)
        arrow_pen.setCapStyle(Qt.RoundCap)
        arrow_pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(arrow_pen)
        painter.drawPath(self._route_arrow_path)

    @staticmethod
    def _sample_route_run(center: np.ndarray, start: int, stop: int) -> np.ndarray:
        """Keep enough screen-space points for curves without drawing thousands."""
        count = stop - start
        if count <= 2:
            return np.arange(start, stop, dtype=np.int64)
        selected = [start]
        last = center[start]
        for index in range(start + 1, stop - 1):
            if float(np.linalg.norm(center[index] - last)) >= 2.5:
                selected.append(index)
                last = center[index]
        selected.append(stop - 1)
        if len(selected) > 240:
            positions = np.linspace(
                0, len(selected) - 1, 240, dtype=np.int64)
            selected = [selected[index] for index in positions]
        return np.asarray(selected, dtype=np.int64)

    def _ensure_route_geometry(self, target: QRectF) -> None:
        key = (
            id(self._route),
            round(target.left(), 2), round(target.top(), 2),
            round(target.width(), 2), round(target.height(), 2),
            self._source_size,
        )
        if key == self._route_geometry_key:
            return
        self._route_geometry_key = key
        self._route_fill_path = QPainterPath()
        self._route_center_path = QPainterPath()
        self._route_arrow_path = QPainterPath()
        if self._route is None:
            return
        center, left, right, valid = self._route
        center = np.asarray(center, dtype=np.float32)
        left = np.asarray(left, dtype=np.float32)
        right = np.asarray(right, dtype=np.float32)
        valid = np.asarray(valid, dtype=bool).reshape(-1)
        if len(center) < 2 or not (
            len(center) == len(left) == len(right) == len(valid)
        ):
            return
        finite = (
            np.all(np.isfinite(center), axis=1)
            & np.all(np.isfinite(left), axis=1)
            & np.all(np.isfinite(right), axis=1)
        )
        valid &= finite
        source_w, source_h = self._source_size
        scale = np.array([
            target.width() / max(source_w, 1),
            target.height() / max(source_h, 1),
        ], dtype=np.float32)
        offset = np.array([target.left(), target.top()], dtype=np.float32)
        mapped_center = center[:, :2] * scale + offset
        mapped_left = left[:, :2] * scale + offset
        mapped_right = right[:, :2] * scale + offset
        from .navigation_display import route_chevrons
        visible = target.intersected(QRectF(self.rect()))
        for arrow in route_chevrons(mapped_center, valid,
                (visible.left(), visible.top(), visible.right(), visible.bottom())):
            self._route_arrow_path.moveTo(float(arrow[0, 0]), float(arrow[0, 1]))
            self._route_arrow_path.lineTo(float(arrow[1, 0]), float(arrow[1, 1]))
            self._route_arrow_path.lineTo(float(arrow[2, 0]), float(arrow[2, 1]))
        # Near-camera projection can produce very large off-screen values.
        # Bound them before handing the path to Qt's rasterizer.
        x_bounds = (
            target.left() - target.width(),
            target.right() + target.width(),
        )
        y_bounds = (
            target.top() - target.height(),
            target.bottom() + target.height(),
        )
        for mapped in (mapped_center, mapped_left, mapped_right):
            mapped[:, 0] = np.clip(mapped[:, 0], *x_bounds)
            mapped[:, 1] = np.clip(mapped[:, 1], *y_bounds)

        padded = np.concatenate(([False], valid, [False]))
        changes = np.flatnonzero(padded[1:] != padded[:-1])
        for start, stop in zip(changes[::2], changes[1::2]):
            if stop - start < 2:
                continue
            indices = self._sample_route_run(mapped_center, start, stop)
            first = mapped_left[indices[0]]
            self._route_fill_path.moveTo(float(first[0]), float(first[1]))
            for point in mapped_left[indices[1:]]:
                self._route_fill_path.lineTo(float(point[0]), float(point[1]))
            for point in mapped_right[indices[::-1]]:
                self._route_fill_path.lineTo(float(point[0]), float(point[1]))
            self._route_fill_path.closeSubpath()

            center_first = mapped_center[indices[0]]
            self._route_center_path.moveTo(
                float(center_first[0]), float(center_first[1]))
            for point in mapped_center[indices[1:]]:
                self._route_center_path.lineTo(
                    float(point[0]), float(point[1]))

    def _draw_status(self, painter: QPainter, target: QRectF) -> None:
        compact = self.width() < 520 or self.height() < 360
        margin = 8 if compact else 22
        width = min(310., max(110., self.width()-margin*2-self.status_right_margin))
        card = QRectF(margin, 8 if compact else 20, width, 94 if compact else 108)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(7, 12, 18, 220))
        painter.drawRoundedRect(card, 12, 12)
        cue = self._guidance
        live = self._camera_live and cue is not None and cue['live']
        left = card.left()+12
        if live:
            painter.save()
            painter.translate(left+15, card.top()+24)
            painter.rotate(-math.degrees(cue['angle']))
            pen = QPen(QColor('#e1fdff'), 3)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(pen)
            arrow = QPainterPath()
            arrow.moveTo(0, 13); arrow.lineTo(0, -12)
            arrow.moveTo(-7, -4); arrow.lineTo(0, -12); arrow.lineTo(7, -4)
            painter.drawPath(arrow)
            painter.restore()
        painter.setPen(QColor('#f4f7f9'))
        painter.setFont(QFont('Sans Serif', 11 if compact else 13, QFont.DemiBold))
        instruction = cue['instruction'] if cue is not None else '等待路线'
        if not self._camera_live:
            instruction = self._frame_message
        painter.drawText(QRectF(left+(38 if live else 0), card.top()+6,
            card.width()-24-(38 if live else 0), 38), Qt.AlignVCenter|Qt.TextWordWrap, instruction)
        show_distance = self._camera_live and (live or (cue is None and self._route is not None))
        distance = f'剩余约 {self._distance:.1f} m' if show_distance else '-- m'
        painter.setFont(QFont('Sans Serif', 10 if compact else 12))
        painter.setPen(QColor('#bcecff'))
        painter.drawText(card.adjusted(12, 44, -10, -24), Qt.AlignLeft|Qt.AlignVCenter, distance)
        message = self.projection_status or ('相机在线 · 路径已显示' if self._route is not None else '等待路线投影')
        if not self._camera_live:
            message = '相机画面未更新 · 路线暂不显示'
        painter.setPen(QColor('#94a5b2'))
        painter.setFont(QFont('Sans Serif', 8 if compact else 9))
        painter.drawText(card.adjusted(12, 66 if compact else 76, -10, -4), Qt.AlignVCenter|Qt.TextWordWrap, message)

    def set_frame_age(self, preview_age, source_age=None):
        from .features.camera_state import camera_message
        self._frame_message = camera_message(preview_age, source_age)
        source_current = source_age is None or -.05 <= source_age <= .5
        self._camera_live = self._pixmap is not None and preview_age is not None and 0 <= preview_age <= .5 and source_current
        if not source_current:
            self._frame_message = '相机时间无效或画面延迟'
        if not self._camera_live:
            self._route = None
            self._route_geometry_key = None
        self.update()


class MapPanel(QWidget):
    """Grid/top-down cloud view with an optional interactive 3-D cloud mode."""

    point_selected = pyqtSignal(float, float)
    height_selected = pyqtSignal(float)
    edit_started = pyqtSignal()
    edit_requested = pyqtSignal(str, str, float, float, float)
    view_zoom_changed = pyqtSignal(int)
    navigation_follow_changed = pyqtSignal(bool)

    def __init__(self, cloud_3d: bool = False) -> None:
        super().__init__()
        self.setMinimumSize(260, 200)
        self.setCursor(Qt.CrossCursor)
        self._mode = "grid"
        self._grid_pixmap: Optional[QPixmap] = None
        self._grid_shape = (0, 0)
        self._grid_origin = np.zeros(2, dtype=np.float32)
        self._grid_resolution = 0.0
        self._grid_origin_yaw = 0.0
        self._cloud_pixmap: Optional[QPixmap] = None
        self._cloud_shape = (0, 0)
        self._cloud_origin = np.zeros(2, dtype=np.float32)
        self._cloud_resolution = 0.0
        self._cloud_3d_enabled = bool(cloud_3d)
        self._cloud_source_points = np.empty((0, 3), dtype=np.float32)
        self._cloud_source_colors: Optional[np.ndarray] = None
        self._cloud_source_color_source = ""
        self._cloud_source_splat_scales: Optional[np.ndarray] = None
        self._cloud_source_splat_rotations: Optional[np.ndarray] = None
        self._cloud_source_splat_opacities: Optional[np.ndarray] = None
        self._cloud_total_points = 0
        self._cloud_points = np.empty((0, 3), dtype=np.float32)
        self._cloud_colors = np.empty((0, 3), dtype=np.uint8)
        self._cloud_display_matrix = np.eye(3, dtype=np.float32)
        self._cloud_display_offset = np.zeros(3, dtype=np.float32)
        self._cloud_alignment = "original"
        self._cloud_axis_order = "XYZ"
        self._cloud_axis_flips = (False, False, False)
        self._cloud_projection = "perspective"
        self._cloud_color_mode = "rgb"
        self._cloud_point_size = 2
        self._cloud_render_revision = 0
        self._cloud_render_key = None
        self._cloud_render_image = None
        self._height_slice = None
        self._height_slice_display = True
        self._height_slice_only = False
        self._height_pick_enabled = False
        self._cloud_center = np.zeros(3, dtype=np.float32)
        self._cloud_extent = np.ones(3, dtype=np.float32)
        self._cloud_bounds_low = np.zeros(3, dtype=np.float32)
        self._cloud_view_target = np.zeros(3, dtype=np.float32)
        self._cloud_azimuth = math.radians(-45.0)
        self._cloud_elevation = math.radians(32.0)
        self._cloud_zoom = 1.0
        self._map_zoom = 1.0
        self._map_pan = QPointF()
        self._view_drag_button = Qt.NoButton
        self._last_mouse_position = None
        self._view_drag_distance = 0.0
        self._touch_positions = {}
        self._reset_view_on_release = False
        self._path: Optional[np.ndarray] = None
        self._motion_trace: Optional[np.ndarray] = None
        self._robot_xy: Optional[np.ndarray] = None
        self._robot_yaw = 0.0
        self._navigation_follow = False
        self._last_follow_transform = QTransform()
        self._waypoints = []
        self._selected_waypoint_index: Optional[int] = None
        self._edit_tool = "navigate"
        self._brush_radius = 0.20
        self._painting = False

    @property
    def display_mode(self) -> str:
        return self._mode

    def set_display_mode(self, mode: str) -> None:
        if mode not in ("grid", "cloud"):
            raise ValueError(f"unknown map display mode: {mode}")
        self._mode = mode
        self.setAttribute(Qt.WA_AcceptTouchEvents, self._cloud_3d_enabled and mode == "cloud")
        self._update_cursor()
        self.update()

    def set_edit_tool(self, tool: str, brush_radius: float) -> None:
        self._view_drag_button = Qt.NoButton
        self._last_mouse_position = None
        self._view_drag_distance = 0.0
        self._painting = False
        if tool != "navigate":
            self.set_navigation_follow(False)
        self._edit_tool = tool
        self._brush_radius = max(0.01, float(brush_radius))
        self._update_cursor()

    def _update_cursor(self) -> None:
        if self._edit_tool == "pan":
            cursor = Qt.OpenHandCursor
        elif self._cloud_3d_enabled and self._mode == "cloud":
            cursor = Qt.OpenHandCursor if self._edit_tool == "navigate" else Qt.CrossCursor
        else:
            cursor = Qt.CrossCursor if self._edit_tool != "navigate" else Qt.PointingHandCursor
        self.setCursor(cursor)

    def reset_cloud_view(self) -> None:
        """Restore an isometric view fitted around the current cloud."""
        self._cloud_view_target = self._cloud_center.copy()
        self._cloud_azimuth = math.radians(-45.0)
        self._cloud_elevation = math.radians(32.0)
        self._cloud_zoom = 1.0
        self._sync_navigation_view()
        self.update()

    def reset_map_view(self) -> None:
        self._map_zoom = 1.0
        self._map_pan = QPointF()
        self.view_zoom_changed.emit(100)
        self.update()

    def fit_view(self) -> None:
        self.set_navigation_follow(False)
        if self._cloud_3d_enabled and self._mode == "cloud":
            self.reset_cloud_view()
        else:
            self.reset_map_view()

    def zoom_view(self, factor: float, anchor: Optional[QPointF] = None) -> None:
        if self._cloud_3d_enabled and self._mode == "cloud":
            self._cloud_zoom = float(np.clip(
                self._cloud_zoom * factor, 0.08, 25.0))
            self.view_zoom_changed.emit(int(round(self._cloud_zoom * 100)))
            self.update()
            return
        target = self._map_rect()
        if target.isEmpty():
            return
        anchor = anchor or QPointF(self.width() * .5, self.height() * .5)
        relative = QPointF(
            (anchor.x() - target.left()) / target.width(),
            (anchor.y() - target.top()) / target.height())
        previous = self._map_zoom
        self._map_zoom = float(np.clip(previous * factor, .25, 20.0))
        if self._map_zoom == previous:
            return
        moved = self._map_rect()
        self._map_pan += anchor - QPointF(
            moved.left() + relative.x() * moved.width(),
            moved.top() + relative.y() * moved.height())
        self._clamp_map_pan()
        self.view_zoom_changed.emit(int(round(self._map_zoom * 100)))
        self.update()

    def view_zoom_percent(self) -> int:
        zoom = (
            self._cloud_zoom
            if self._cloud_3d_enabled and self._mode == "cloud"
            else self._map_zoom)
        return int(round(zoom * 100.0))

    @property
    def cloud_has_rgb(self) -> bool:
        return self._cloud_source_colors is not None

    def set_cloud_display_options(
        self,
        alignment: str,
        axis_order: str,
        axis_flips,
        projection: str,
        color_mode: str,
    ) -> None:
        if alignment != "original":
            raise ValueError(f"unknown cloud alignment: {alignment}")
        if sorted(axis_order) != ["X", "Y", "Z"]:
            raise ValueError(f"invalid cloud axis order: {axis_order}")
        if projection not in ("orthographic", "perspective"):
            raise ValueError(f"unknown cloud projection: {projection}")
        if color_mode not in ("rgb", "height", "intensity"):
            raise ValueError(f"unknown cloud color mode: {color_mode}")
        self._cloud_alignment = alignment
        self._cloud_axis_order = axis_order
        self._cloud_axis_flips = tuple(bool(value) for value in axis_flips)
        self._cloud_projection = projection
        self._cloud_color_mode = color_mode
        self._rebuild_cloud_3d()
        self.reset_cloud_view()

    def set_map(
        self,
        occupancy: Optional[np.ndarray],
        origin: np.ndarray,
        resolution: float,
        origin_yaw: float,
    ) -> None:
        if occupancy is None:
            self._grid_pixmap = None
            self.update()
            return
        colored = occupancy_to_bgr(occupancy)[:, :, ::-1]
        colored = np.ascontiguousarray(np.flipud(colored))
        h, w = colored.shape[:2]
        qimage = QImage(
            colored.data, w, h, colored.strides[0], QImage.Format_RGB888).copy()
        self._grid_pixmap = QPixmap.fromImage(qimage)
        self._grid_shape = (h, w)
        self._grid_origin = np.asarray(origin, dtype=np.float32)[:2]
        self._grid_resolution = float(resolution)
        self._grid_origin_yaw = float(origin_yaw)
        self.update()

    def update_grid_patch(self, patch: np.ndarray, column: int, row: int) -> None:
        """Update changed grid cells; row is bottom-up in map coordinates."""
        colored = np.ascontiguousarray(np.flipud(occupancy_to_bgr(patch)[:, :, ::-1]))
        height, width = colored.shape[:2]
        image = QImage(colored.data, width, height, colored.strides[0], QImage.Format_RGB888).copy()
        image_x = int(column)
        image_y = self._grid_shape[0] - int(row) - height
        painter = QPainter(self._grid_pixmap)
        painter.drawImage(image_x, image_y, image)
        painter.end()
        target = self._map_rect()
        scale_x = target.width() / self._grid_shape[1]
        scale_y = target.height() / self._grid_shape[0]
        dirty = QRectF(target.left() + image_x * scale_x, target.top() + image_y * scale_y,
                       width * scale_x, height * scale_y)
        dirty = self._map_follow_transform().mapRect(dirty).toAlignedRect().adjusted(-1, -1, 1, 1)
        self.update(dirty)

    def set_pointcloud(
        self,
        points: Optional[np.ndarray],
        colors: Optional[np.ndarray] = None,
        splat_scales: Optional[np.ndarray] = None,
        splat_rotations: Optional[np.ndarray] = None,
        splat_opacities: Optional[np.ndarray] = None,
        color_source: str = "",
    ) -> None:
        if points is None or not len(points):
            self._cloud_pixmap = None
            self._cloud_source_points = np.empty((0, 3), dtype=np.float32)
            self._cloud_source_colors = None
            self._cloud_source_color_source = ""
            self._cloud_source_splat_scales = None
            self._cloud_source_splat_rotations = None
            self._cloud_source_splat_opacities = None
            self._cloud_total_points = 0
            self._cloud_points = np.empty((0, 3), dtype=np.float32)
            self._cloud_colors = np.empty((0, 3), dtype=np.uint8)
            self.update()
            return
        source_xyz = np.asarray(points, dtype=np.float32)
        finite = np.all(np.isfinite(source_xyz[:, :3]), axis=1)
        xyz = source_xyz[:, :3] if np.all(finite) else source_xyz[finite, :3]
        if not len(xyz):
            self._cloud_pixmap = None
            self._cloud_source_points = np.empty((0, 3), dtype=np.float32)
            self._cloud_source_colors = None
            self._cloud_source_color_source = ""
            self._cloud_source_splat_scales = None
            self._cloud_source_splat_rotations = None
            self._cloud_source_splat_opacities = None
            self._cloud_total_points = 0
            self._cloud_points = np.empty((0, 3), dtype=np.float32)
            self._cloud_colors = np.empty((0, 3), dtype=np.uint8)
            self.update()
            return
        was_empty = not len(self._cloud_points)
        if colors is not None:
            source_colors = np.asarray(colors, dtype=np.uint8)
            if source_colors.shape != (len(points), 3):
                source_colors = None
            elif not np.all(finite):
                source_colors = source_colors[finite]
        else:
            source_colors = None
        is_splat = all(value is not None for value in (
            splat_scales, splat_rotations, splat_opacities))
        if is_splat:
            source_scales = np.asarray(splat_scales, dtype=np.float32)
            source_rotations = np.asarray(splat_rotations, dtype=np.float32)
            source_opacities = np.asarray(splat_opacities, dtype=np.float32).reshape(-1)
            if (
                source_scales.shape != (len(points), 3)
                or source_rotations.shape != (len(points), 4)
                or len(source_opacities) != len(points)
            ):
                is_splat = False
            elif not np.all(finite):
                source_scales = source_scales[finite]
                source_rotations = source_rotations[finite]
                source_opacities = source_opacities[finite]
        self._cloud_total_points = len(xyz)
        display_limit = 60000 if is_splat else 300000
        if len(xyz) > display_limit and is_splat:
            top_count = int(display_limit * 0.75)
            uniform_count = display_limit - top_count
            importance = source_opacities * np.square(
                np.max(source_scales, axis=1))
            top = np.argpartition(importance, -top_count)[-top_count:]
            uniform = np.linspace(
                0, len(xyz) - 1, uniform_count, dtype=np.int64)
            sample_indices = np.unique(np.concatenate((top, uniform)))
        elif len(xyz) > display_limit:
            sample_step = int(math.ceil(len(xyz) / display_limit))
            sample_indices = np.arange(0, len(xyz), sample_step)
        else:
            sample_indices = np.arange(len(xyz))
        self._cloud_source_points = np.ascontiguousarray(xyz[sample_indices, :3])
        self._cloud_source_colors = (
            None if source_colors is None
            else np.ascontiguousarray(source_colors[sample_indices]))
        self._cloud_source_color_source = (
            str(color_source) if self._cloud_source_colors is not None else "")
        if is_splat:
            self._cloud_source_splat_scales = np.ascontiguousarray(
                source_scales[sample_indices])
            self._cloud_source_splat_rotations = np.ascontiguousarray(
                source_rotations[sample_indices])
            self._cloud_source_splat_opacities = np.ascontiguousarray(
                source_opacities[sample_indices])
        else:
            self._cloud_source_splat_scales = None
            self._cloud_source_splat_rotations = None
            self._cloud_source_splat_opacities = None
        if not self._cloud_3d_enabled:
            render_xyz = self._cloud_source_points
            lower = render_xyz[:, :2].min(axis=0)
            upper = render_xyz[:, :2].max(axis=0)
            extent = np.maximum(upper - lower, 0.05)
            resolution = max(float(np.max(extent)) / 1100.0, 0.01)
            width, height = np.ceil(extent / resolution).astype(int) + 1
            image = np.zeros((int(height), int(width), 3), dtype=np.uint8)
            image[:, :] = (8, 15, 21)
            pixels = np.floor((render_xyz[:, :2] - lower) / resolution).astype(int)
            pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
            pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
            z = render_xyz[:, 2]
            z_low, z_high = np.percentile(z, [3, 97]) if len(z) > 10 else (z.min(), z.max())
            normalized = np.clip((z - z_low) / max(float(z_high - z_low), 1e-6), 0.0, 1.0)
            colors = np.column_stack([
                35 + 210 * normalized,
                205 + 45 * normalized,
                245 - 110 * normalized,
            ]).astype(np.uint8)
            # Draw high points last, making walls and obstacles easier to distinguish.
            order = np.argsort(z)
            ordered_x = pixels[order, 0]
            ordered_y = pixels[order, 1]
            ordered_colors = colors[order]
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                draw_x = np.clip(ordered_x + dx, 0, width - 1)
                draw_y = np.clip(ordered_y + dy, 0, height - 1)
                image[draw_y, draw_x] = ordered_colors
            image = np.ascontiguousarray(np.flipud(image))
            qimage = QImage(
                image.data, image.shape[1], image.shape[0], image.strides[0],
                QImage.Format_RGB888).copy()
            self._cloud_pixmap = QPixmap.fromImage(qimage)
            self._cloud_shape = (int(height), int(width))
            self._cloud_origin = lower.astype(np.float32)
            self._cloud_resolution = resolution
        self._rebuild_cloud_3d()
        if was_empty:
            self.reset_cloud_view()
        self.update()

    def _rebuild_cloud_3d(self) -> None:
        self._cloud_render_revision += 1
        if not len(self._cloud_source_points):
            self._cloud_points = np.empty((0, 3), dtype=np.float32)
            self._cloud_colors = np.empty((0, 3), dtype=np.uint8)
            return

        indices = ["XYZ".index(axis) for axis in self._cloud_axis_order]
        signs = np.array([
            -1.0 if flipped else 1.0 for flipped in self._cloud_axis_flips
        ], dtype=np.float32)
        axis_matrix = np.zeros((3, 3), dtype=np.float32)
        for output_axis, source_axis in enumerate(indices):
            axis_matrix[output_axis, source_axis] = signs[output_axis]
        self._cloud_display_matrix = axis_matrix
        self._cloud_display_offset = np.zeros(3, dtype=np.float32)
        displayed = (
            self._cloud_source_points @ self._cloud_display_matrix.T
            + self._cloud_display_offset
        )
        self._cloud_points = np.ascontiguousarray(displayed.astype(np.float32))
        percentile = [0.5, 99.5] if len(displayed) > 200 else [0.0, 100.0]
        bounds_low, bounds_high = np.percentile(displayed, percentile, axis=0)
        self._cloud_center = ((bounds_low + bounds_high) * 0.5).astype(np.float32)
        self._cloud_extent = np.maximum(
            bounds_high - bounds_low,
            np.array([0.1, 0.1, 0.1], dtype=np.float32),
        ).astype(np.float32)
        self._cloud_bounds_low = bounds_low.astype(np.float32)

        if self._cloud_color_mode == "rgb" and self._cloud_source_colors is not None:
            self._cloud_colors = self._cloud_source_colors.copy()
        else:
            if self._cloud_color_mode == "intensity" and self._cloud_source_color_source == "intensity":
                normalized = np.clip((self._cloud_source_colors[:, 0].astype(np.float32) - 35.0) / 220.0, 0.0, 1.0)
            else:
                height = displayed[:, 2]
                z_low, z_high = np.percentile(height, [3, 97])
                normalized = np.clip(
                    (height - z_low) / max(float(z_high - z_low), 1e-6),
                    0.0, 1.0,
                )
            anchors = [0.0, 0.25, 0.5, 0.75, 1.0]
            self._cloud_colors = np.ascontiguousarray(np.column_stack([
                np.interp(normalized, anchors, [32, 0, 32, 255, 255]),
                np.interp(normalized, anchors, [96, 200, 220, 220, 70]),
                np.interp(normalized, anchors, [255, 255, 90, 40, 50]),
            ]).astype(np.uint8))

    def cloud_display_description(self) -> str:
        alignment = "原始坐标"
        projection = "透视" if self._cloud_projection == "perspective" else "正交"
        if self._cloud_color_mode == "intensity" and self._cloud_source_color_source == "intensity":
            color = "强度伪彩色"
        elif self._cloud_color_mode == "rgb" and self.cloud_has_rgb:
            color = {
                "intensity": "原始强度",
                "gaussian": "基础颜色",
            }.get(self._cloud_source_color_source, "原始 RGB")
        else:
            color = "高度着色"
        flips = [
            f"反{axis}" for axis, enabled
            in zip("XYZ", self._cloud_axis_flips) if enabled
        ]
        axes = self._cloud_axis_order + (" " + "/".join(flips) if flips else "")
        kind = "点云"
        return f"{kind} · {alignment} · {axes} · {projection} · {color}"

    def transform_cloud_points(self, points: np.ndarray) -> np.ndarray:
        """Apply the preview transform to a copy of full-resolution XYZ data."""
        xyz = np.asarray(points, dtype=np.float32)[:, :3]
        return np.ascontiguousarray(
            xyz @ self._cloud_display_matrix.T + self._cloud_display_offset)

    def set_cloud_point_size(self, size):
        self._cloud_point_size = int(size)
        self.update()

    def set_height_slice(self, limits, *, display_coordinates=True, only_retained=False):
        """Preview a conversion interval without changing any source points."""
        self._height_slice = limits
        self._height_slice_display = display_coordinates
        self._height_slice_only = only_retained
        self.update()

    def height_slice_mask(self):
        points = self._cloud_points if self._height_slice_display else self._cloud_source_points
        if self._height_slice is None:
            return np.ones(len(points), dtype=bool)
        low, high = self._height_slice
        return (points[:, 2] >= low) & (points[:, 2] <= high)

    def _pick_cloud_height(self, position):
        if not len(self._cloud_points):
            return None
        x, y, depth = self._project_cloud_points(self._cloud_points)
        distance = np.square(x - position.x()) + np.square(y - position.y())
        visible = np.isfinite(distance) & (distance <= 12 ** 2)
        if self._height_slice_only:
            visible &= self.height_slice_mask()
        indices = np.flatnonzero(visible)
        if not len(indices):
            return None
        nearest = distance[indices].min()
        indices = indices[distance[indices] <= nearest + 9]
        index = indices[np.argmax(depth[indices])]
        points = self._cloud_points if self._height_slice_display else self._cloud_source_points
        return float(points[index, 2])

    def _draw_height_planes(self, painter):
        if self._height_slice is None or not len(self._cloud_points):
            return
        points = self._cloud_points if self._height_slice_display else self._cloud_source_points
        low, high = np.percentile(points[:, :2], [0.5, 99.5], axis=0)
        for height, color, label in zip(self._height_slice,
                (QColor(70, 180, 255), QColor(255, 175, 65)), ("下界", "上界")):
            corners = np.array([[low[0], low[1], height], [high[0], low[1], height],
                                [high[0], high[1], height], [low[0], high[1], height]], dtype=np.float32)
            if not self._height_slice_display:
                corners = self.transform_cloud_points(corners)
            x, y, _depth = self._project_cloud_points(corners)
            if not (np.isfinite(x).all() and np.isfinite(y).all()):
                continue
            painter.setPen(QPen(color, 1.5))
            fill = QColor(color); fill.setAlpha(22)
            painter.setBrush(fill)
            painter.drawPolygon(QPolygonF([QPointF(float(px), float(py)) for px, py in zip(x, y)]))
            painter.setPen(color)
            anchor = 0 if label == "下界" else 2
            painter.drawText(QPointF(float(x[anchor]) + 4, float(y[anchor]) - 5), f"{label} Z={height:.2f} m")
        painter.setBrush(Qt.NoBrush)

    def _cloud_view_basis(self):
        azimuth = self._cloud_azimuth
        elevation = self._cloud_elevation
        right = np.array(
            [-math.sin(azimuth), math.cos(azimuth), 0.0], dtype=np.float32)
        up = np.array([
            -math.sin(elevation) * math.cos(azimuth),
            -math.sin(elevation) * math.sin(azimuth),
            math.cos(elevation),
        ], dtype=np.float32)
        toward_camera = np.array([
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            math.sin(elevation),
        ], dtype=np.float32)
        return right, up, toward_camera

    def _cloud_view_scale(self) -> float:
        longest = max(float(np.linalg.norm(self._cloud_extent)), 0.1)
        minimum_size = max(1.0, min(self.width(), self.height()))
        if self._cloud_projection == "perspective":
            focal_length = minimum_size * 0.82 * self._cloud_zoom
            camera_distance = longest * 1.25
            return focal_length / camera_distance
        fitted = max(1.0, minimum_size * 0.82 / longest)
        return fitted * self._cloud_zoom

    def _project_cloud_points(self, points: np.ndarray):
        right, up, depth_axis = self._cloud_view_basis()
        relative = np.asarray(points, dtype=np.float32) - self._cloud_view_target
        depth = relative @ depth_axis
        if self._cloud_projection == "perspective":
            longest = max(float(np.linalg.norm(self._cloud_extent)), 0.1)
            camera_distance = longest * 1.25
            focal_length = max(
                1.0, min(self.width(), self.height()) * 0.82 * self._cloud_zoom)
            camera_depth = camera_distance - depth
            safe_depth = np.where(camera_depth > longest * 0.02, camera_depth, np.nan)
            screen_x = self.width() * 0.5 + (relative @ right) * focal_length / safe_depth
            screen_y = self.height() * 0.5 - (relative @ up) * focal_length / safe_depth
        else:
            scale = self._cloud_view_scale()
            screen_x = self.width() * 0.5 + relative @ right * scale
            screen_y = self.height() * 0.5 - relative @ up * scale
        return screen_x, screen_y, depth

    @staticmethod
    def _quaternion_rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
        """Convert Gaussian Splat quaternions (w, x, y, z) to matrices."""
        q = np.asarray(quaternions, dtype=np.float32)
        w, x, y, z = q.T
        matrices = np.empty((len(q), 3, 3), dtype=np.float32)
        matrices[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
        matrices[:, 0, 1] = 2.0 * (x * y - z * w)
        matrices[:, 0, 2] = 2.0 * (x * z + y * w)
        matrices[:, 1, 0] = 2.0 * (x * y + z * w)
        matrices[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
        matrices[:, 1, 2] = 2.0 * (y * z - x * w)
        matrices[:, 2, 0] = 2.0 * (x * z - y * w)
        matrices[:, 2, 1] = 2.0 * (y * z + x * w)
        matrices[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
        return matrices

    def _gaussian_screen_radii(self, indices: np.ndarray):
        scales = self._cloud_source_splat_scales[indices]
        rotations = self._quaternion_rotation_matrices(
            self._cloud_source_splat_rotations[indices])
        scaled_axes = rotations * scales[:, None, :]
        display_axes = np.einsum(
            "ij,njk->nik", self._cloud_display_matrix, scaled_axes)
        points = self._cloud_points[indices]
        relative = points - self._cloud_view_target
        right, up, depth_axis = self._cloud_view_basis()
        if self._cloud_projection == "perspective":
            longest = max(float(np.linalg.norm(self._cloud_extent)), 0.1)
            camera_distance = longest * 1.25
            focal = max(
                1.0, min(self.width(), self.height()) * 0.82 * self._cloud_zoom)
            camera_depth = np.maximum(
                camera_distance - relative @ depth_axis, longest * 0.02)
            camera_x = relative @ right
            camera_y = relative @ up
            jacobian_x = (
                right[None, :] * (focal / camera_depth)[:, None]
                + depth_axis[None, :]
                * (focal * camera_x / np.square(camera_depth))[:, None]
            )
            jacobian_y = -(
                up[None, :] * (focal / camera_depth)[:, None]
                + depth_axis[None, :]
                * (focal * camera_y / np.square(camera_depth))[:, None]
            )
        else:
            scale = self._cloud_view_scale()
            jacobian_x = np.broadcast_to(right * scale, (len(indices), 3))
            jacobian_y = np.broadcast_to(-up * scale, (len(indices), 3))
        projected_x = np.einsum("ni,nik->nk", jacobian_x, display_axes)
        projected_y = np.einsum("ni,nik->nk", jacobian_y, display_axes)
        radius_x = np.clip(
            3.0 * np.sqrt(np.sum(np.square(projected_x), axis=1)), 0.7, 32.0)
        radius_y = np.clip(
            3.0 * np.sqrt(np.sum(np.square(projected_y), axis=1)), 0.7, 32.0)
        return radius_x, radius_y

    def _cloud3d_widget_to_world(self, point: QPointF) -> Optional[np.ndarray]:
        """Pick a map-frame XY location on the displayed cloud ground."""
        if not len(self._cloud_points):
            return None
        right, up, depth_axis = self._cloud_view_basis()
        ground_z = float(self._cloud_bounds_low[2])
        relative_z = ground_z - float(self._cloud_view_target[2])
        if self._cloud_projection == "perspective":
            longest = max(float(np.linalg.norm(self._cloud_extent)), 0.1)
            camera_distance = longest * 1.25
            focal_length = max(
                1.0, min(self.width(), self.height()) * 0.82 * self._cloud_zoom)
            normalized_x = (point.x() - self.width() * 0.5) / focal_length
            normalized_y = (self.height() * 0.5 - point.y()) / focal_length
            matrix = np.array([
                right[:2] + normalized_x * depth_axis[:2],
                up[:2] + normalized_y * depth_axis[:2],
            ])
            screen = np.array([
                normalized_x * camera_distance
                - (right[2] + normalized_x * depth_axis[2]) * relative_z,
                normalized_y * camera_distance
                - (up[2] + normalized_y * depth_axis[2]) * relative_z,
            ])
        else:
            scale = self._cloud_view_scale()
            screen = np.array([
                (point.x() - self.width() * 0.5) / scale
                - right[2] * relative_z,
                (self.height() * 0.5 - point.y()) / scale
                - up[2] * relative_z,
            ])
            matrix = np.array([
                [right[0], right[1]],
                [up[0], up[1]],
            ])
        if abs(float(np.linalg.det(matrix))) < 1e-5:
            return None
        relative_xy = np.linalg.solve(matrix, screen)
        displayed = np.array([
            self._cloud_view_target[0] + relative_xy[0],
            self._cloud_view_target[1] + relative_xy[1],
            ground_z,
        ], dtype=np.float32)
        # Display transforms are rigid axis rotations, so their inverse
        # is the transpose.  Return the original map-frame XY used by Nav2.
        source = (
            displayed - self._cloud_display_offset
        ) @ self._cloud_display_matrix
        return source[:2]

    def _orbit_cloud_view(self, dx: float, dy: float) -> None:
        self.set_navigation_follow(False)
        self._cloud_azimuth -= float(dx) * 0.008
        self._cloud_elevation = float(np.clip(
            self._cloud_elevation - float(dy) * 0.008,
            math.radians(-85.0), math.radians(85.0),
        ))
        self.update()

    def _pan_cloud_view(self, dx: float, dy: float) -> None:
        self.set_navigation_follow(False)
        right, up, _depth = self._cloud_view_basis()
        scale = self._cloud_view_scale()
        self._cloud_view_target += (-right * float(dx) + up * float(dy)) / scale
        self.update()

    def _zoom_cloud_view(self, wheel_steps: float) -> None:
        self._cloud_zoom = float(np.clip(
            self._cloud_zoom * math.exp(float(wheel_steps) * 0.16),
            0.08, 25.0,
        ))
        self.update()

    def set_navigation_state(
        self,
        path: Optional[np.ndarray],
        robot_xy: Optional[np.ndarray],
        robot_yaw: float,
    ) -> None:
        self._path = path
        valid = (robot_xy is not None and np.asarray(robot_xy).shape == (2,)
                 and np.isfinite(robot_xy).all() and math.isfinite(robot_yaw))
        self._robot_xy = np.asarray(robot_xy).copy() if valid else None
        self._robot_yaw = float(robot_yaw) if valid else 0.0
        self._sync_navigation_view()
        self.update()

    def set_motion_trace(self, points) -> None:
        self._motion_trace = (
            None if points is None else np.asarray(points, dtype=np.float32))
        self.update()

    @property
    def navigation_follow_enabled(self) -> bool:
        return self._navigation_follow

    def set_navigation_follow(self, enabled: bool) -> None:
        self._navigation_follow = (
            bool(enabled) and self._edit_tool == "navigate")
        if not self._navigation_follow:
            self._last_follow_transform = QTransform()
        self._sync_navigation_view()
        self.navigation_follow_changed.emit(self._navigation_follow)
        self.update()

    def _sync_navigation_view(self) -> None:
        if not self._navigation_follow or self._robot_xy is None:
            return
        if self._cloud_3d_enabled and self._mode == "cloud":
            direction = np.array([
                math.cos(self._robot_yaw), math.sin(self._robot_yaw)])
            points = self._cloud_ground_points_from_xy(np.array([
                self._robot_xy, self._robot_xy + direction]))
            self._cloud_view_target = points[0]
            forward = points[1] - points[0]
            if np.linalg.norm(forward[:2]) > 1e-6:
                self._cloud_azimuth = math.atan2(
                    forward[1], forward[0]) + math.pi

    def _map_follow_transform(self) -> QTransform:
        transform = QTransform()
        if not self._navigation_follow:
            return transform
        if self._robot_xy is None:
            return self._last_follow_transform
        _pixmap, _shape, _origin, resolution, origin_yaw = self._view()
        if resolution <= 0.0:
            return transform
        center = self._world_to_map_widget(self._robot_xy)
        transform.translate(self.width() * .5, self.height() * .60)
        transform.rotate(math.degrees(self._robot_yaw - origin_yaw) - 90.0)
        transform.translate(-center.x(), -center.y())
        self._last_follow_transform = transform
        return transform

    def set_waypoints(self, waypoints) -> None:
        self._waypoints = [np.asarray(point).copy() for point in waypoints]
        if (
            self._selected_waypoint_index is not None
            and self._selected_waypoint_index >= len(self._waypoints)
        ):
            self._selected_waypoint_index = None
        self.update()

    def set_waypoint_selection(self, index: Optional[int]) -> None:
        self._selected_waypoint_index = (
            int(index) if index is not None and 0 <= index < len(self._waypoints)
            else None)
        self.update()

    def _view(self):
        if self._mode == "cloud":
            return (
                self._cloud_pixmap, self._cloud_shape, self._cloud_origin,
                self._cloud_resolution, 0.0)
        return (
            self._grid_pixmap, self._grid_shape, self._grid_origin,
            self._grid_resolution, self._grid_origin_yaw)

    def _base_map_rect(self) -> QRectF:
        pixmap, _shape, _origin, _resolution, _yaw = self._view()
        if pixmap is None:
            return QRectF()
        margin = 12.0
        available_w = max(1.0, self.width() - margin * 2)
        available_h = max(1.0, self.height() - margin * 2)
        scale = min(
            available_w / pixmap.width(),
            available_h / pixmap.height(),
        )
        width = pixmap.width() * scale
        height = pixmap.height() * scale
        return QRectF(
            (self.width() - width) * 0.5,
            (self.height() - height) * 0.5,
            width,
            height,
        )

    def _map_rect(self) -> QRectF:
        fitted = self._base_map_rect()
        if fitted.isEmpty():
            return fitted
        center = fitted.center() + self._map_pan
        width = fitted.width() * self._map_zoom
        height = fitted.height() * self._map_zoom
        return QRectF(
            center.x() - width * .5, center.y() - height * .5,
            width, height)

    def _clamp_map_pan(self) -> None:
        fitted = self._base_map_rect()
        if fitted.isEmpty():
            self._map_pan = QPointF()
            return
        width = fitted.width() * self._map_zoom
        height = fitted.height() * self._map_zoom
        center = fitted.center() + self._map_pan
        visible_x = min(48.0, width * .5)
        visible_y = min(48.0, height * .5)
        center_x = float(np.clip(
            center.x(), visible_x - width * .5,
            self.width() - visible_x + width * .5))
        center_y = float(np.clip(
            center.y(), visible_y - height * .5,
            self.height() - visible_y + height * .5))
        self._map_pan = QPointF(
            center_x - fitted.center().x(),
            center_y - fitted.center().y())

    def _pan_map_view(self, dx: float, dy: float) -> None:
        self.set_navigation_follow(False)
        self._map_pan += QPointF(dx, dy)
        self._clamp_map_pan()
        self.update()

    def world_to_widget(self, xy: np.ndarray) -> QPointF:
        return self._map_follow_transform().map(
            self._world_to_map_widget(xy))

    def _world_to_map_widget(self, xy: np.ndarray) -> QPointF:
        target = self._map_rect()
        _pixmap, (h, w), origin, resolution, origin_yaw = self._view()
        relative = np.asarray(xy) - origin
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        grid_x = (cos_yaw * relative[0] + sin_yaw * relative[1]) / resolution
        grid_y = (-sin_yaw * relative[0] + cos_yaw * relative[1]) / resolution
        return QPointF(
            target.left() + grid_x * target.width() / w,
            target.top() + (h - 1 - grid_y) * target.height() / h,
        )

    def widget_to_world(self, point: QPointF) -> Optional[np.ndarray]:
        if self._cloud_3d_enabled and self._mode == "cloud":
            return self._cloud3d_widget_to_world(point)
        target = self._map_rect()
        inverse, invertible = self._map_follow_transform().inverted()
        if not invertible:
            return None
        point = inverse.map(point)
        pixmap, (h, w), origin, resolution, origin_yaw = self._view()
        if (
            pixmap is None
            or resolution <= 0.0
            or not target.contains(point)
        ):
            return None
        grid_x = (point.x() - target.left()) * w / target.width()
        display_y = (point.y() - target.top()) * h / target.height()
        grid_y = h - 1 - display_y
        local = np.array(
            [grid_x * resolution, grid_y * resolution])
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        return origin + np.array([
            cos_yaw * local[0] - sin_yaw * local[1],
            sin_yaw * local[0] + cos_yaw * local[1],
        ])

    def event(self, event):
        kind = event.type()
        if kind in (QEvent.TouchBegin, QEvent.TouchUpdate, QEvent.TouchEnd, QEvent.TouchCancel):
            if kind in (QEvent.TouchEnd, QEvent.TouchCancel):
                button = self._view_drag_button
                if button != Qt.NoButton:
                    if kind == QEvent.TouchCancel:
                        self._view_drag_distance = 5.0
                    self.mouseReleaseEvent(QMouseEvent(
                        QEvent.MouseButtonRelease, event.touchPoints()[0].pos()
                        if kind == QEvent.TouchEnd else QPointF(), button, Qt.NoButton, Qt.NoModifier))
                self._touch_positions = {}
                self._painting = False
                event.accept()
                return True
            points = {point.id(): point.pos() for point in event.touchPoints()
                      if point.state() != Qt.TouchPointReleased}
            previous = self._touch_positions
            if len(points) == 1:
                position = next(iter(points.values()))
                if kind == QEvent.TouchBegin:
                    self.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, position,
                        Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
                elif points.keys() == previous.keys():
                    self.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, position,
                        Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
                else:
                    self._view_drag_button = Qt.LeftButton
                    self._last_mouse_position = position.toPoint()
                    self._view_drag_distance = 5.0
            else:
                self._painting = False
                self._view_drag_button = Qt.RightButton
                self._view_drag_distance = 5.0
                if points.keys() == previous.keys():
                    first, second = sorted(points)[:2]
                    center = (points[first] + points[second]) * .5
                    old_center = (previous[first] + previous[second]) * .5
                    self._pan_cloud_view(center.x() - old_center.x(), center.y() - old_center.y())
                    old_delta = previous[first] - previous[second]
                    delta = points[first] - points[second]
                    old_distance = math.hypot(old_delta.x(), old_delta.y())
                    distance = math.hypot(delta.x(), delta.y())
                    if old_distance > 0 and distance > 0:
                        self.zoom_view(distance / old_distance, center)
                self.update()
            self._touch_positions = points
            event.accept()
            return True
        return super().event(event)

    def mousePressEvent(self, event) -> None:
        self._reset_view_on_release = False
        if self._height_pick_enabled and self._mode == "cloud" and event.button() == Qt.LeftButton:
            height = self._pick_cloud_height(event.localPos())
            if height is not None:
                self._height_pick_enabled = False
                self.height_selected.emit(height)
            event.accept()
            return
        if self._cloud_3d_enabled and self._mode == "cloud":
            is_orbit = event.button() == Qt.LeftButton and self._edit_tool == "navigate"
            is_pan = (
                event.button() in (Qt.RightButton, Qt.MiddleButton)
                or event.button() == Qt.LeftButton
                and self._edit_tool == "pan")
            if is_orbit or is_pan:
                self._view_drag_button = event.button()
                self._last_mouse_position = event.pos()
                self._view_drag_distance = 0.0
                self.setCursor(Qt.ClosedHandCursor)
                event.accept()
                return
        elif (
            event.button() in (Qt.RightButton, Qt.MiddleButton)
            or event.button() == Qt.LeftButton
            and self._edit_tool == "pan"
        ):
            self._view_drag_button = event.button()
            self._last_mouse_position = event.pos()
            self._view_drag_distance = 0.0
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        if event.button() == Qt.LeftButton:
            world = self.widget_to_world(event.localPos())
            if world is not None:
                if self._edit_tool == "navigate":
                    self.point_selected.emit(float(world[0]), float(world[1]))
                else:
                    self._painting = True
                    self.edit_started.emit()
                    self.edit_requested.emit(
                        self._mode, self._edit_tool, float(world[0]),
                        float(world[1]), self._brush_radius)
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._view_drag_button != Qt.NoButton and self._last_mouse_position is not None:
            delta = event.pos() - self._last_mouse_position
            self._last_mouse_position = event.pos()
            self._view_drag_distance += abs(delta.x()) + abs(delta.y())
            if self._view_drag_distance <= 4.0:
                event.accept()
                return
            if self._cloud_3d_enabled and self._mode == "cloud" and (
                self._view_drag_button == Qt.LeftButton
                and self._edit_tool == "navigate"):
                self._orbit_cloud_view(delta.x(), delta.y())
            elif self._cloud_3d_enabled and self._mode == "cloud":
                self._pan_cloud_view(delta.x(), delta.y())
            else:
                self._pan_map_view(delta.x(), delta.y())
            event.accept()
            return
        if self._painting and event.buttons() & Qt.LeftButton:
            world = self.widget_to_world(event.localPos())
            if world is not None:
                self.edit_requested.emit(
                    self._mode, self._edit_tool, float(world[0]),
                    float(world[1]), self._brush_radius)
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == self._view_drag_button:
            select_point = (
                event.button() == Qt.LeftButton
                and self._edit_tool == "navigate"
                and self._view_drag_distance <= 4.0
            )
            reset_view = self._reset_view_on_release and self._view_drag_distance <= 4.0
            self._reset_view_on_release = False
            self._view_drag_button = Qt.NoButton
            self._last_mouse_position = None
            self._view_drag_distance = 0.0
            self._update_cursor()
            self.update()
            if reset_view:
                self.fit_view()
            elif select_point:
                world = self.widget_to_world(event.localPos())
                if world is not None:
                    self.point_selected.emit(float(world[0]), float(world[1]))
            event.accept()
            return
        if event.button() == Qt.LeftButton:
            self._painting = False
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta:
            self.zoom_view(
                math.exp(float(delta) / 120.0 * .16), event.posF())
            event.accept()
            return
        super().wheelEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            if self._cloud_3d_enabled and self._mode == "cloud":
                self.mousePressEvent(event)
                self._reset_view_on_release = self._edit_tool in ("navigate", "pan")
                event.accept()
                return
            if self._edit_tool not in ("navigate", "pan"):
                self.mousePressEvent(event)
                return
            self.fit_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#111820"))
        if self._cloud_3d_enabled and self._mode == "cloud":
            self._paint_cloud_3d(painter)
            return
        pixmap, _shape, _origin, _resolution, _yaw = self._view()
        if pixmap is None:
            painter.setPen(QColor("#8999a5"))
            painter.setFont(QFont("Sans Serif", 13))
            waiting = "尚未加载点云地图" if self._mode == "cloud" else "等待栅格地图 /map"
            painter.drawText(self.rect(), Qt.AlignCenter, waiting)
            return
        target = self._map_rect()
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        painter.save()
        painter.setTransform(self._map_follow_transform(), True)
        painter.drawPixmap(target, pixmap, QRectF(pixmap.rect()))
        painter.restore()
        painter.setClipRect(
            self.rect() if self._navigation_follow else target)
        self._draw_motion_trace(painter)
        self._draw_global_path(painter)
        self._draw_robot(painter)
        for index, waypoint in enumerate(self._waypoints):
            is_last = index == len(self._waypoints) - 1
            if index == self._selected_waypoint_index:
                color = QColor("#4ed8ff")
            else:
                color = QColor("#ff5f67") if is_last else QColor("#ffc857")
            self._draw_marker(painter, waypoint, color, str(index + 1))
        self._draw_grid_scale(painter)

    def grid_scale_info(self):
        if self._mode != "grid" or self._grid_pixmap is None:
            return None
        pixels_per_meter = self._map_rect().width() / (self._grid_shape[1] * self._grid_resolution)
        target_meters = 120.0 / pixels_per_meter
        power = 10.0 ** math.floor(math.log10(target_meters))
        length = max(value for value in (1.0, 2.0, 5.0) if value <= target_meters / power) * power
        return length, length * pixels_per_meter, self._grid_resolution

    def _draw_grid_scale(self, painter):
        scale = self.grid_scale_info()
        if scale is None:
            return
        length, pixels, cell = scale
        label = f"{length:g} m" if length >= 1 else f"{length*100:g} cm"
        painter.save()
        painter.setClipping(False)
        x, y = 20.0, float(self.height()-28)
        painter.fillRect(QRectF(x-8, y-37, 245, 58), QColor(10, 18, 25, 225))
        painter.setPen(QPen(QColor("#e1eff5"), 2))
        painter.drawLine(QPointF(x,y), QPointF(x+pixels,y))
        painter.drawLine(QPointF(x,y-5), QPointF(x,y+5))
        painter.drawLine(QPointF(x+pixels,y-5), QPointF(x+pixels,y+5))
        painter.setFont(QFont("Sans Serif",9))
        painter.drawText(QPointF(x,y-15), f"{label}    1 格 = {cell*100:g} cm")
        painter.restore()

    def _draw_motion_trace(self, painter: QPainter) -> None:
        if self._motion_trace is None or len(self._motion_trace) < 2:
            return
        path = QPainterPath(self.world_to_widget(self._motion_trace[0, :2]))
        for point in self._motion_trace[1:]:
            path.lineTo(self.world_to_widget(point[:2]))
        painter.setPen(QPen(
            QColor("#e18425"), 3.0, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

    def _paint_cloud_3d(self, painter: QPainter) -> None:
        if not len(self._cloud_points):
            painter.setPen(QColor("#8999a5"))
            painter.setFont(QFont("Sans Serif", 13))
            painter.drawText(self.rect(), Qt.AlignCenter, "尚未加载点云地图")
            return

        width, height = max(1, self.width()), max(1, self.height())
        render_key = (self._cloud_render_revision, width, height,
                      self._cloud_projection, self._cloud_azimuth, self._cloud_elevation,
                      self._cloud_zoom, tuple(self._cloud_view_target), self._cloud_point_size,
                      self._height_slice, self._height_slice_display, self._height_slice_only,
                      self._view_drag_button != Qt.NoButton)
        render_gaussians = self._cloud_source_splat_scales is not None
        if render_gaussians or render_key != self._cloud_render_key:
            image = np.empty((height, width, 3), dtype=np.uint8)
            image[:, :] = (8, 14, 20)
            # Keep orbit/pan responsive; release redraws the full display sample.
            step = (max(1, math.ceil(len(self._cloud_points) / 20000))
                    if self._view_drag_button != Qt.NoButton else 1)
            screen_x, screen_y, depth = self._project_cloud_points(self._cloud_points[::step])
            render_gaussians = (
                self._cloud_source_splat_scales is not None
                and self._view_drag_button == Qt.NoButton
            )
            projectable = np.isfinite(screen_x) & np.isfinite(screen_y)
            retained = self.height_slice_mask()[::step]
            if self._height_slice is not None and self._height_slice_only:
                projectable &= retained
            display_colors = self._cloud_colors[::step]
            if self._height_slice is not None:
                display_colors = display_colors.copy()
                display_colors[~retained] = (45, 55, 65)
            pixel_x = np.rint(screen_x[projectable]).astype(np.int32)
            pixel_y = np.rint(screen_y[projectable]).astype(np.int32)
            projected_depth = depth[projectable]
            projected_colors = display_colors[projectable]
            visible = (
                (pixel_x >= 0) & (pixel_x < width)
                & (pixel_y >= 0) & (pixel_y < height)
            )
            if np.any(visible) and not render_gaussians:
                pixel_x = pixel_x[visible]
                pixel_y = pixel_y[visible]
                visible_depth = projected_depth[visible]
                colors = projected_colors[visible].astype(np.float32)
                depth_span = max(float(np.ptp(visible_depth)), 1e-6)
                light = 0.82 + 0.18 * (
                    (visible_depth - float(visible_depth.min())) / depth_span)
                colors = np.clip(colors * light[:, None], 0, 255).astype(np.uint8)
                order = np.argsort(visible_depth)
                ordered_x = pixel_x[order]
                ordered_y = pixel_y[order]
                ordered_colors = colors[order]
                point_size = self._cloud_point_size
                for offset_x in range(point_size):
                    for offset_y in range(point_size):
                        draw_x = np.clip(ordered_x + offset_x, 0, width - 1)
                        draw_y = np.clip(ordered_y + offset_y, 0, height - 1)
                        image[draw_y, draw_x] = ordered_colors

            qimage = QImage(
                image.data, width, height, image.strides[0],
                QImage.Format_RGB888,
            ).copy()
            self._cloud_render_image = qimage
            self._cloud_render_key = render_key
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        painter.drawImage(self.rect(), self._cloud_render_image)
        if render_gaussians and self._view_drag_button == Qt.NoButton:
            self._draw_gaussian_splats(
                painter, screen_x, screen_y, depth, projectable)
        self._draw_cloud_3d_reference(painter)
        self._draw_height_planes(painter)
        self._draw_cloud_3d_navigation(painter)

        compact = width < 520 or height < 360
        painter.setPen(QColor("#c4d3dc"))
        painter.setFont(QFont("Sans Serif", 10, QFont.DemiBold))
        kind = "三维点云"
        painter.drawText(
            QRectF(18, 14, width - 36, 28),
            Qt.AlignLeft | Qt.AlignVCenter,
            (
                f"{kind} · {len(self._cloud_points):,} 点"
                if compact else
                f"{kind} · 渲染 {len(self._cloud_points):,} / "
                f"{self._cloud_total_points:,}"
            ),
        )
        if not compact:
            painter.setPen(QColor("#71bed8"))
            painter.setFont(QFont("Sans Serif", 9))
            painter.drawText(
                QRectF(18, 40, width - 36, 24),
                Qt.AlignLeft | Qt.AlignVCenter,
                self.cloud_display_description(),
            )
        painter.setPen(QColor("#91a5b2"))
        painter.setFont(QFont("Sans Serif", 9))
        painter.drawText(
            QRectF(18, height - 42, width - 36, 28),
            Qt.AlignLeft | Qt.AlignVCenter,
            (
                "单指/左键拖动 · 右键平移 · 滚轮缩放"
                if compact else
                "左键拖动旋转  ·  右键/中键拖动平移  ·  "
                "滚轮缩放  ·  双击复位  ·  单指按当前模式拖动"
            ),
        )

    def _draw_gaussian_splats(
        self,
        painter: QPainter,
        screen_x: np.ndarray,
        screen_y: np.ndarray,
        depth: np.ndarray,
        projectable: np.ndarray,
    ) -> None:
        margin = 34.0
        visible = (
            projectable
            & (screen_x >= -margin) & (screen_x < self.width() + margin)
            & (screen_y >= -margin) & (screen_y < self.height() + margin)
            & (self._cloud_source_splat_opacities >= 0.01)
        )
        indices = np.flatnonzero(visible)
        if not len(indices):
            return
        radius_x, radius_y = self._gaussian_screen_radii(indices)
        order = np.argsort(depth[indices])
        indices = indices[order]
        radius_x = radius_x[order]
        radius_y = radius_y[order]
        colors = self._cloud_colors[indices]
        opacities = self._cloud_source_splat_opacities[indices]
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.setPen(Qt.NoPen)
        for index, rx, ry, color, opacity in zip(
            indices, radius_x, radius_y, colors, opacities,
        ):
            alpha = int(np.clip(float(opacity) * 205.0, 2.0, 245.0))
            painter.setBrush(QColor(
                int(color[0]), int(color[1]), int(color[2]), alpha))
            painter.drawEllipse(QRectF(
                float(screen_x[index] - rx), float(screen_y[index] - ry),
                float(rx * 2.0), float(ry * 2.0),
            ))

    def _draw_cloud_3d_reference(self, painter: QPainter) -> None:
        """Draw a ground grid and XYZ axes to make depth unambiguous."""
        half = max(float(self._cloud_extent[0]), float(self._cloud_extent[1])) * 0.55
        half = max(half, 0.5)
        ground_z = float(self._cloud_bounds_low[2])
        center_x, center_y = self._cloud_center[:2]
        ticks = np.linspace(-half, half, 11)
        painter.setPen(QPen(QColor(87, 112, 126, 70), 1.0))
        for tick in ticks:
            lines = (
                np.array([
                    [center_x - half, center_y + tick, ground_z],
                    [center_x + half, center_y + tick, ground_z],
                ]),
                np.array([
                    [center_x + tick, center_y - half, ground_z],
                    [center_x + tick, center_y + half, ground_z],
                ]),
            )
            for line in lines:
                x, y, _depth = self._project_cloud_points(line)
                painter.drawLine(QPointF(x[0], y[0]), QPointF(x[1], y[1]))

        origin = np.array([center_x, center_y, ground_z], dtype=np.float32)
        axis_length = max(half * 0.28, float(self._cloud_extent[2]), 0.3)
        axes = (
            (np.array([axis_length, 0.0, 0.0]), QColor("#ff5f67"), "X"),
            (np.array([0.0, axis_length, 0.0]), QColor("#66e09a"), "Y"),
            (np.array([0.0, 0.0, axis_length]), QColor("#4ed8ff"), "Z"),
        )
        for vector, color, label in axes:
            x, y, _depth = self._project_cloud_points(
                np.vstack((origin, origin + vector)))
            painter.setPen(QPen(color, 2.2))
            painter.drawLine(QPointF(x[0], y[0]), QPointF(x[1], y[1]))
            painter.drawText(QPointF(x[1] + 4, y[1] - 3), label)

    def _cloud_ground_points_from_xy(self, xy: np.ndarray) -> np.ndarray:
        """Place map-frame XY coordinates on the displayed cloud ground."""
        points_xy = np.asarray(xy, dtype=np.float32).reshape(-1, 2)
        ground_z = float(self._cloud_bounds_low[2])
        matrix = self._cloud_display_matrix
        denominator = float(matrix[2, 2])
        if abs(denominator) < 1e-6:
            return np.column_stack((
                points_xy,
                np.full(len(points_xy), ground_z + 0.04, dtype=np.float32),
            ))
        source_z = (
            ground_z - self._cloud_display_offset[2]
            - points_xy[:, 0] * matrix[2, 0]
            - points_xy[:, 1] * matrix[2, 1]
        ) / denominator
        source = np.column_stack((points_xy, source_z)).astype(np.float32)
        displayed = self.transform_cloud_points(source)
        displayed[:, 2] += 0.04
        return displayed

    def _draw_cloud_3d_navigation(self, painter: QPainter) -> None:
        """Overlay the active global path and robot on the 3-D ground plane."""
        if self._motion_trace is not None and len(self._motion_trace) >= 2:
            trace_points = self._cloud_ground_points_from_xy(
                self._motion_trace[:, :2])
            screen_x, screen_y, _depth = self._project_cloud_points(
                trace_points)
            trace = QPainterPath()
            started = False
            for x, y in zip(screen_x, screen_y):
                if not math.isfinite(x) or not math.isfinite(y):
                    started = False
                elif started:
                    trace.lineTo(float(x), float(y))
                else:
                    trace.moveTo(float(x), float(y))
                    started = True
            painter.setPen(QPen(
                QColor("#e18425"), 3.0, Qt.SolidLine, Qt.RoundCap))
            painter.setBrush(Qt.NoBrush)
            painter.drawPath(trace)
        if self._path is not None and len(self._path) >= 2:
            path_points = self._cloud_ground_points_from_xy(self._path[:, :2])
            screen_x, screen_y, _depth = self._project_cloud_points(path_points)
            finite = np.isfinite(screen_x) & np.isfinite(screen_y)
            valid_indices = np.flatnonzero(finite)
            if len(valid_indices) >= 2:
                path = QPainterPath(QPointF(
                    float(screen_x[valid_indices[0]]),
                    float(screen_y[valid_indices[0]]),
                ))
                for index in valid_indices[1:]:
                    path.lineTo(QPointF(
                        float(screen_x[index]), float(screen_y[index])))
                painter.setPen(QPen(
                    QColor("#00d5ff"), 3.2, Qt.SolidLine, Qt.RoundCap))
                painter.setBrush(Qt.NoBrush)
                painter.drawPath(path)

        for index, waypoint in enumerate(self._waypoints):
            point_3d = self._cloud_ground_points_from_xy(
                np.asarray(waypoint, dtype=np.float32)[:2])
            screen_x, screen_y, _depth = self._project_cloud_points(point_3d)
            if not np.isfinite(screen_x[0]) or not np.isfinite(screen_y[0]):
                continue
            point = QPointF(float(screen_x[0]), float(screen_y[0]))
            is_last = index == len(self._waypoints) - 1
            if index == self._selected_waypoint_index:
                color = QColor("#4ed8ff")
            else:
                color = QColor("#ff5f67") if is_last else QColor("#ffc857")
            painter.setPen(QPen(QColor("#ffffff"), 1.8))
            painter.setBrush(color)
            painter.drawEllipse(point, 6, 6)
            painter.setPen(QColor("#ffffff"))
            painter.drawText(point + QPointF(8, -7), str(index + 1))

        if self._robot_xy is None:
            return
        direction = np.array([
            math.cos(self._robot_yaw), math.sin(self._robot_yaw)],
            dtype=np.float32)
        robot_points = self._cloud_ground_points_from_xy(np.vstack((
            self._robot_xy[:2], self._robot_xy[:2] + direction,
        )))
        screen_x, screen_y, _depth = self._project_cloud_points(robot_points)
        if not np.all(np.isfinite(screen_x)) or not np.all(np.isfinite(screen_y)):
            return
        center = np.array([screen_x[0], screen_y[0]], dtype=float)
        heading = np.array(
            [screen_x[1] - screen_x[0], screen_y[1] - screen_y[0]],
            dtype=float)
        length = float(np.linalg.norm(heading))
        if length < 1e-5:
            heading = np.array([0.0, -1.0])
        else:
            heading /= length
        side = np.array([-heading[1], heading[0]])
        arrow = QPolygonF([
            QPointF(*(center + heading * 10.0)),
            QPointF(*(center - heading * 7.0 + side * 6.0)),
            QPointF(*(center - heading * 4.0)),
            QPointF(*(center - heading * 7.0 - side * 6.0)),
        ])
        painter.setPen(QPen(QColor("#073346"), 1.8))
        painter.setBrush(QColor("#ffffff"))
        painter.drawPolygon(arrow)

    def _draw_global_path(self, painter: QPainter) -> None:
        if self._path is None or len(self._path) < 2:
            return
        path = QPainterPath(self.world_to_widget(self._path[0]))
        for point in self._path[1:]:
            path.lineTo(self.world_to_widget(point))
        painter.setPen(QPen(QColor("#00b9f2"), 3.2, Qt.SolidLine, Qt.RoundCap))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

    def _draw_robot(self, painter: QPainter) -> None:
        if self._robot_xy is None:
            return
        center = self.world_to_widget(self._robot_xy)
        _pixmap, _shape, _origin, _resolution, origin_yaw = self._view()
        heading = -math.pi / 2 if self._navigation_follow else origin_yaw - self._robot_yaw
        base = np.array([[12, 0], [-8, -7], [-5, 0], [-8, 7]], dtype=float)
        rotation = np.array([
            [math.cos(heading), -math.sin(heading)],
            [math.sin(heading), math.cos(heading)],
        ])
        polygon = QPolygonF([
            QPointF(center.x() + p[0], center.y() + p[1])
            for p in base @ rotation.T
        ])
        painter.setPen(QPen(QColor("#053347"), 2))
        painter.setBrush(QColor("#ffffff"))
        painter.drawPolygon(polygon)

    def _draw_marker(
        self,
        painter: QPainter,
        xy: Optional[np.ndarray],
        color: QColor,
        label: str,
    ) -> None:
        if xy is None:
            return
        point = self.world_to_widget(xy)
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(color)
        painter.drawEllipse(point, 7, 7)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Sans Serif", 9, QFont.DemiBold))
        painter.drawText(point + QPointF(10, -8), label)


class QtNavRosNode(Node):
    """ROS-facing state and commands consumed by the Qt widgets."""

    def __init__(self) -> None:
        super().__init__("nav_qt_nav")
        self.declare_parameter("camera_topic", "/color/image_raw")
        self.declare_parameter("camera_info_topic", "/color/camera_info")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("path_topic_fallbacks", ["/global_plan"])
        self.declare_parameter("pointcloud_topic", "")
        self.declare_parameter("pointcloud_map_path", "")
        self.declare_parameter("map_storage_dir", "")
        self.declare_parameter("robot_profile", "")
        self.robot_profile = str(self.get_parameter("robot_profile").value)
        self.declare_parameter("mapping_backend", "")
        self.mapping_backend = str(self.get_parameter("mapping_backend").value)
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
        self.declare_parameter("fullscreen_on_small_screen", True)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("operating_mode", "robot")
        self.declare_parameter("phone_remote_enabled", False)
        self.declare_parameter("phone_remote_bind", "0.0.0.0")
        self.declare_parameter("phone_remote_port", 8765)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ground_frame = str(self.get_parameter("ground_frame").value)
        self.ground_height_m = float(self.get_parameter("ground_height_m").value)
        self.ground_render_offset_m = float(self.get_parameter("ground_render_offset_m").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.camera_frame_override = str(self.get_parameter("camera_frame").value)
        self.projection_mode = str(self.get_parameter("projection_mode").value)
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
            raise ValueError("AR 路线宽度、范围或地面高度配置无效")
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        configured_fallbacks = [
            str(topic).strip()
            for topic in self.get_parameter("path_topic_fallbacks").value
            if str(topic).strip()
        ]
        self.path_topics = tuple(dict.fromkeys(
            [self.path_topic, *configured_fallbacks, "/plan", "/global_plan"]
        ))
        self.pointcloud_topic = str(
            self.get_parameter("pointcloud_topic").value)
        self.fullscreen_on_small_screen = bool(
            self.get_parameter("fullscreen_on_small_screen").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.operating_mode = str(
            self.get_parameter("operating_mode").value)
        if self.operating_mode not in ("walking", "robot"):
            raise ValueError("operating_mode must be walking or robot")
        self.phone_remote_enabled = bool(
            self.get_parameter("phone_remote_enabled").value)
        self.phone_remote_bind = str(
            self.get_parameter("phone_remote_bind").value)
        self.phone_remote_port = int(
            self.get_parameter("phone_remote_port").value)
        self.map_storage_dir = default_map_directory(str(
            self.get_parameter("map_storage_dir").value))
        self._last_camera_message_time = 0.0
        self._last_lidar_message_time = 0.0
        self._odometry_received = {"dlio": 0., "fast_livo": 0., "external": 0.}
        self._odometry_backend = "dlio"
        self._fast_navigation_odometry = False
        self._navigation_health_subscription = None
        self.require_localizer_quality = False

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
        self.tf_listener = DisplayTransformListener(
            self.tf_buffer, use_sim_time=bool(self.get_parameter("use_sim_time").value))
        self._navigation_lidar_topic = (
            "/foo/livox/lidar_192_168_128_3" if self.robot_profile == "woosh"
            else "/livox/lidar/pointcloud")
        self._navigation_input_subscriptions = []
        self._last_scan_message_time = 0.
        self._planning_inputs = ()
        self._amcl_initial_pose_time = 0.
        self.amcl_initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self._navigation_lidar_subscription = self.tf_listener.node.create_subscription(
            PointCloud2, self._navigation_lidar_topic,
            self._on_navigation_lidar_data, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.path_subscriptions = []
        for topic in self.path_topics:
            subscription = self.create_subscription(
                Path,
                topic,
                lambda msg, source_topic=topic: self._on_path(
                    msg, source_topic),
                reliable_qos,
            )
            self.path_subscriptions.append(subscription)
        if self.pointcloud_topic:
            self.create_subscription(
                PointCloud2, self.pointcloud_topic,
                self._on_pointcloud, sensor_qos)
        self.navigation_client = ActionClient(
            self, ComputePathThroughPoses, "/compute_path_through_poses")
        self.robot_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses")
        from lifecycle_msgs.srv import GetState
        from .features.navigation_readiness import NavigationReadiness
        self._navigation_readiness = NavigationReadiness(
            # Lifecycle replies must progress even while Qt is rendering a map.
            lambda service: self.tf_listener.node.create_client(GetState, service),
            GetState.Request,
            localization_ready=lambda: (
                not self.require_localizer_quality
                or self.localization_tools.health.valid(
                    self.get_clock().now().nanoseconds)),
        )
        for backend, topic in (("dlio", "/dlio/odom_node/odom"),
                               ("fast_livo", "/aft_mapped_to_init")):
            self.create_subscription(
                Odometry, topic,
                lambda msg, key=backend: self._on_odometry(msg, key), sensor_qos)
        self.create_timer(.2, self._monitor_localization)
        from .features.wtrtk import RtkRosAdapter
        from .features.rtk import RtkMonitorAdapter
        from .features.field_performance import FieldPerformance
        from .features.trajectory_test import TrajectoryTest
        self.trajectory_test = TrajectoryTest()
        self.rtk_live = RtkRosAdapter(self)
        self.rtk_monitor = RtkMonitorAdapter(self)
        self.performance = FieldPerformance(self)
        self.mapping = MappingRosAdapter(self)
        self.sensor_preview = SensorPreviewRosAdapter(self)
        self.localization_tools = LocalizationRosAdapter(self, quality_node=self.tf_listener.node)
        from .features.outdoor import OutdoorAdapter
        self.outdoor_adapter = OutdoorAdapter(
            self, initial_pose_adapter=self.localization_tools)
        self.outdoor = self.outdoor_adapter.state
        self.map_services = self.outdoor_adapter.services
        from .features.walking_guidance import WalkingGuidance
        from .features.robot_navigation import RobotNavigation
        self.walking = WalkingGuidance(
            self.navigation_client, self._walking_changed,
            ready=self.navigation_route_ready,
            arrival_radius=.25 if self.robot_profile == "woosh" else .8)
        self.robot = RobotNavigation(
            self.robot_client, self._robot_changed,
            ready=self.navigation_route_ready)
        self._planned_waypoints = []
        from std_msgs.msg import Bool, String
        from std_srvs.srv import SetBool
        from .features.motion_session import MotionSession
        self.motion_permit_pub = self.create_publisher(
            Bool, "/nav/motion_permit", 1)
        self.motion_session = MotionSession(
            self.create_client(SetBool, "/nav/arm_motion"),
            lambda: SetBool.Request(data=True))
        self.create_subscription(
            String, "/nav/motion_status",
            lambda message: self.motion_session.update(
                message.data, self.get_clock().now().nanoseconds),
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.latest_image: Optional[np.ndarray] = None
        self.image_revision = 0
        self._image_worker = LatestWorker(lambda sample: image_to_bgr(sample[0]))
        self.tf_listener.node.create_subscription(
            Image, self.camera_topic, self._on_image, sensor_qos)
        from .image_time_buffer import ImageTimeBuffer
        self._image_time_buffer = ImageTimeBuffer()
        self.camera_frame = ""
        self.image_stamp = None
        self._image_received_at = 0.
        self.camera_error = ""
        self.path_error = ""
        self.camera_calibration_error = ""
        self.camera_distortion = ()
        self.camera_distortion_model = "plumb_bob"
        self.ar_status = dict(code="navigation", message="导航运行后显示相机路线")
        self._guidance_progress = 0.
        self._path_received_at = 0.
        self.camera_matrix: Optional[np.ndarray] = None
        self.camera_info_size: Optional[Tuple[int, int]] = None
        self.path = np.empty((0, 3), dtype=np.float32)
        self.path_frame = self.map_frame
        self.active_path_topic = ""
        self.path_message_count = 0
        self._route_ribbon_key = None
        self._route_ribbon = None
        self.occupancy: Optional[np.ndarray] = None
        self.grid_frame = self.map_frame
        self.map_origin = np.zeros(2, dtype=np.float32)
        self.map_origin_yaw = 0.0
        self.map_resolution = 0.0
        self.map_revision = 0
        self.latest_pointcloud: Optional[np.ndarray] = None
        self.pointcloud_frame = self.map_frame
        self.pointcloud_revision = 0
        self._route_wait_status = None
        self.navigation_status = "请在地图上添加至少一个途径点"
        self.navigation_active = False
        self.navigation_result_status: Optional[int] = None
        self.navigation_result_revision = 0
        self.goal_handle = None
        self.cancel_pending = False
        self.get_logger().info(
            f"Qt navigation ready: camera={self.camera_topic}, "
            f"camera_info={self.camera_info_topic}, map={self.map_topic}, "
            f"path_topics={list(self.path_topics)}, "
            "actions=/compute_path_through_poses,/navigate_through_poses")
        self._route_readiness_timer = self.tf_listener.node.create_timer(
            .2, self.navigation_route_ready)

    def set_clock_mode(self, use_sim_time: bool) -> None:
        from rclpy.parameter import Parameter
        value = Parameter("use_sim_time", value=use_sim_time)
        self.set_parameters([value])
        self.tf_listener.node.set_parameters([value])

    def configure_navigation(self, config) -> None:
        if self._navigation_health_subscription is not None:
            self.tf_listener.node.destroy_subscription(self._navigation_health_subscription)
            self._navigation_health_subscription = None
        for owner, subscription in self._navigation_input_subscriptions:
            owner.destroy_subscription(subscription)
        self._navigation_input_subscriptions.clear()
        self._last_scan_message_time = 0.
        self._planning_inputs = ()
        self._fast_navigation_odometry = False
        if config is None:  # The stack has stopped or is shutting down.
            self.require_localizer_quality = False
            self._navigation_readiness.reset()
            return
        self.set_clock_mode(config.use_sim_time)
        self._odometry_backend = config.odometry_backend
        self._fast_navigation_odometry = config.robot_profile == "woosh" and config.odometry_backend == "fast_livo"
        if self._fast_navigation_odometry:
            self._navigation_input_subscriptions.append((self, self.create_subscription(
                Odometry, "/nav/woosh/odom", lambda msg: self._on_odometry(msg, "fast_livo"),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))))
            self._navigation_health_subscription = self.tf_listener.node.create_subscription(
                Odometry, "/nav/woosh/odom", self._on_fast_navigation_health,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        input_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        self._navigation_input_subscriptions.append((self.tf_listener.node, self.tf_listener.node.create_subscription(
            LaserScan, config.scan_topic, self._on_navigation_scan_data, input_qos)))
        if config.odometry_backend == "external":
            self._navigation_input_subscriptions.append((self, self.create_subscription(
                Odometry, config.external_odom_topic, self._on_external_odometry, input_qos)))
        lidar_topic = config.dlio_pointcloud_topic.strip()
        if lidar_topic != self._navigation_lidar_topic:
            self.tf_listener.node.destroy_subscription(self._navigation_lidar_subscription)
            self._navigation_lidar_topic = lidar_topic
            self._last_lidar_message_time = 0.0
            self._navigation_lidar_subscription = self.tf_listener.node.create_subscription(
                PointCloud2, lidar_topic, self._on_navigation_lidar_data,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=2))
        self.require_localizer_quality = config.localization_backend == "pointcloud_localizer"
        if config.navigation_backend == "walking" and not config.use_sim_time:
            self._planning_inputs = (
                ("scan", "等待实时 scan；路线规划已暂停"),
                ("external_odom" if config.odometry_backend == "external" else "dlio", "等待有效里程计；路线规划已暂停"),
                ("localizer" if self.require_localizer_quality else "external_localization", "等待地图定位，请使用重新定位"),
            )
        self._navigation_readiness.configure(
            config.navigation_backend, mode=self.operating_mode)
        self._odometry_received = {"dlio": 0., "fast_livo": 0., "external": 0.}

    def _on_fast_navigation_health(self, message) -> None:
        # This callback runs on the existing private TF receiver, never the GUI.
        if not self._fast_navigation_odometry or message.header.frame_id != "odom" or message.child_frame_id != self.base_frame:
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp > 0 and -.1 <= age <= 1.:
            self._odometry_received["fast_livo"] = time.monotonic()

    def _on_odometry(self, message, backend: str) -> None:
        if backend == "fast_livo" and self._fast_navigation_odometry and message.child_frame_id != self.base_frame:
            return  # Navigation consumes the measured propagated base pose, not the delayed scan pose.
        if backend == self._odometry_backend:
            self.outdoor_adapter.odometry(message)
        pose = message.pose.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if (
            all(math.isfinite(value) for value in values)
            and .99 < sum(value * value for value in values[3:]) < 1.01
        ):
            self.trajectory_test.feed("odometry", dict(
                source=backend,
                stamp=(message.header.stamp.sec
                       + message.header.stamp.nanosec * 1e-9),
                app_received_at=self.get_clock().now().nanoseconds / 1e9,
                frame=message.header.frame_id,
                child_frame=message.child_frame_id,
                position=list(values[:3]),
                quaternion=list(values[3:]),
                covariance=list(message.pose.covariance),
            ))
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp > 0 and -.1 <= age <= 1.:
            self._odometry_received[backend] = time.monotonic()

    def amcl_initial_pose_ready(self):
        return self.amcl_initial_pose_pub.get_subscription_count() > 0

    def publish_amcl_initial_pose(self, position, yaw, frame):
        if not self.amcl_initial_pose_ready():
            return False, "等待 AMCL 初始位姿接收端"
        message = PoseWithCovarianceStamped()
        message.header.frame_id = frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = map(float, position)
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = yaw_quaternion(yaw)
        message.pose.covariance[0] = message.pose.covariance[7] = .25
        message.pose.covariance[35] = math.radians(15.) ** 2
        self.amcl_initial_pose_pub.publish(message)
        self._amcl_initial_pose_time = time.monotonic()
        return True, "初始位姿已发送，等待 AMCL 确认定位"

    def _on_navigation_scan_data(self, message) -> None:
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp > 0 and -.1 <= age <= 2.:
            self._last_scan_message_time = time.monotonic()

    def _on_external_odometry(self, message) -> None:
        if message.header.frame_id == "odom":
            self._on_odometry(message, "external")

    def _navigation_tf_ready(self, target, source):
        try:
            transform = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return False
        stamp = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp == 0 or -.5 <= age <= 2.

    def navigation_component_ready(self, key: str) -> bool:
        if key == "scan":
            return self._last_scan_message_time > 0 and time.monotonic() - self._last_scan_message_time < 2.
        if key == "external_odom":
            return (time.monotonic() - self._odometry_received["external"] < 1.
                    and self._navigation_tf_ready("odom", self.base_frame))
        if key == "external_localization":
            return self._navigation_tf_ready(self.map_frame, "odom")
        if key == "dlio":
            return (time.monotonic() - self._odometry_received[self._odometry_backend] < 1.
                    and self._navigation_tf_ready("odom", self.base_frame))
        if key == "localizer":
            return (self.localization_tools.health.valid(self.get_clock().now().nanoseconds)
                    and self._navigation_tf_ready(self.map_frame, "odom"))
        if key == "nav2":
            return self.navigation_route_ready()
        return True

    def navigation_route_ready(self) -> bool:
        ready = self._navigation_readiness.ready(
            self.navigation_client.server_is_ready(),
            self.robot_client.server_is_ready())
        for key, reason in self._planning_inputs:
            if not self.navigation_component_ready(key):
                self._navigation_readiness.reason = reason
                return False
        return ready

    @property
    def navigation_readiness_reason(self) -> str:
        return self._navigation_readiness.reason

    def _monitor_localization(self) -> None:
        self.poll_navigation()
        if self._route_wait_status is not None:
            if self.navigation_status != self._route_wait_status:
                self._route_wait_status = None
            elif self.navigation_route_ready():
                self.navigation_status = "路线服务已就绪，可以规划路线"
                self._route_wait_status = None
            else:
                self.navigation_status = self.navigation_readiness_reason
                self._route_wait_status = self.navigation_status
        if (self.navigation_active and self.operating_mode == "robot" and self.require_localizer_quality and
                not self.localization_tools.health.valid(self.get_clock().now().nanoseconds)):
            self.cancel_navigation()
            self.navigation_status = "地图定位失效，已请求取消导航；请重新定位"

    def destroy_node(self):
        self.rtk_live.stop()
        self.outdoor_adapter.close()
        self.trajectory_test.close()
        self.mapping.close()
        self.tf_listener.close()
        self._image_worker.close()
        return super().destroy_node()

    def _on_image(self, msg: Image) -> None:
        received = time.monotonic()
        self._last_camera_message_time = received
        self._image_worker.submit((msg, received))

    def poll_image(self) -> None:
        from .image_time_buffer import DisplayFrame
        result = self._image_worker.take()
        if result is not None:
            (msg, received), image, error = result
            if error:
                self.camera_error = f"相机格式错误: {error}"
                self.navigation_status = self.camera_error
            else:
                self.camera_error = ""
                self._image_time_buffer.append(DisplayFrame(
                    image, msg.header.stamp, received,
                    self.camera_frame_override or msg.header.frame_id,
                    msg.header.frame_id))
        def ready(frame):
            if not self.navigation_active or len(self.path) < 2:
                return True
            return (
                self.lookup_matrix(self.path_frame, self.base_frame, frame.stamp) is not None
                and self.lookup_matrix(frame.frame, self.path_frame, frame.stamp) is not None)
        frame = self._image_time_buffer.select(
            self.get_clock().now().nanoseconds, time.monotonic(), ready)
        if frame is None or frame.image is self.latest_image:
            return
        self.latest_image = frame.image
        self.image_stamp = frame.stamp
        self._image_received_at = frame.received
        self.camera_frame = frame.frame
        self.image_revision += 1

    def _on_camera_info(self, msg: CameraInfo) -> None:
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

    def _on_navigation_lidar_data(self, _msg: PointCloud2) -> None:
        self._last_lidar_message_time = time.monotonic()

    def navigation_sensor_data_active(self, key: str) -> bool:
        """Return true only when real sensor messages arrived recently."""
        last_message = {
            "lidar": self._last_lidar_message_time,
            "camera": self._last_camera_message_time,
        }.get(key, 0.0)
        return last_message > 0.0 and time.monotonic() - last_message < 2.0

    def _on_path(self, msg: Path, source_topic: Optional[str] = None) -> None:
        if not self.navigation_active:
            if self.walking.phase != "preview":
                self.clear_navigation_path()
            return
        if self.operating_mode == "walking" and self.walking.phase in ("guiding", "lost", "deviated"):
            # The accepted planning action owns the hand-guided route.
            return
        source_topic = str(source_topic or getattr(
            self, "path_topic", "/plan"))
        path_topics = tuple(getattr(self, "path_topics", (source_topic,)))
        active_topic = str(getattr(self, "active_path_topic", ""))
        if active_topic and active_topic != source_topic:
            try:
                source_priority = path_topics.index(source_topic)
            except ValueError:
                source_priority = len(path_topics)
            try:
                active_priority = path_topics.index(active_topic)
            except ValueError:
                active_priority = len(path_topics)
            if source_priority > active_priority:
                return
        if not msg.poses:
            self.clear_navigation_path()
            self.active_path_topic = source_topic
            return
        self.path = np.array([
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ], dtype=np.float32).reshape(-1, 3)
        if not np.isfinite(self.path).all():
            self.clear_navigation_path()
            self.path_error = "规划路线包含无效坐标，暂不显示"
            return
        self.path_error = ""
        self._guidance_progress = 0.
        self._path_received_at = time.monotonic()
        self.path_frame = (
            msg.header.frame_id
            or (msg.poses[0].header.frame_id if msg.poses else "")
            or self.map_frame
        )
        topic_changed = active_topic != source_topic
        self.active_path_topic = source_topic
        self.path_message_count = int(getattr(
            self, "path_message_count", 0)) + 1
        self._route_ribbon_key = None
        if topic_changed and hasattr(self, "get_logger"):
            self.get_logger().info(
                f"Navigation path received from {source_topic}: "
                f"{len(self.path)} poses, frame={self.path_frame}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        expected = int(msg.info.width * msg.info.height)
        data = np.asarray(msg.data, dtype=np.int8)
        if expected == 0 or len(data) != expected:
            return
        self.occupancy = data.reshape(msg.info.height, msg.info.width).copy()
        self.grid_frame = msg.header.frame_id or self.map_frame
        self.map_resolution = float(msg.info.resolution)
        origin = msg.info.origin
        self.map_origin = np.array(
            [origin.position.x, origin.position.y], dtype=np.float32)
        q = origin.orientation
        self.map_origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.map_revision += 1

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        try:
            self.latest_pointcloud = pointcloud2_to_xyz(msg)
            self.pointcloud_frame = msg.header.frame_id or self.map_frame
            self.pointcloud_revision += 1
        except (ValueError, TypeError) as exc:
            self.navigation_status = f"点云格式错误: {exc}"

    def lookup_matrix(self, target: str, source: str, stamp=None) -> Optional[np.ndarray]:
        if target == source:
            return np.eye(4)
        try:
            query = Time.from_msg(stamp) if stamp is not None else Time()
            transform = self.tf_buffer.lookup_transform(target, source, query)
            return transform_matrix(transform.transform)
        except TransformException:
            return None

    def projected_route(self, image_shape):
        from .ar_projection import projected_route
        return projected_route(self, image_shape)

    def path_in_frame(self, target: str) -> Optional[np.ndarray]:
        if not len(self.path):
            return self.path.copy()
        matrix = self.lookup_matrix(target, self.path_frame)
        return None if matrix is None else apply_transform(self.path, matrix)

    def map_navigation_state(self):
        path = self.path_in_frame(self.grid_frame)
        path_xy = None if path is None or not len(path) else path[:, :2]
        if self.require_localizer_quality and not (
            self.navigation_component_ready("localizer")
            and self.navigation_component_ready("dlio")
        ):
            return path_xy, None, 0.0
        matrix = self.lookup_matrix(self.grid_frame, self.base_frame)
        if matrix is None:
            return path_xy, None, 0.0
        robot_xy = matrix[:2, 3].copy()
        robot_yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return path_xy, robot_xy, robot_yaw

    def remaining_distance(self) -> float:
        path = self.path_in_frame(self.base_frame)
        if path is None or not len(path):
            return cumulative_distance(self.path)
        nearest = int(np.argmin(np.linalg.norm(path[:, :2], axis=1)))
        return cumulative_distance(path[nearest:])

    def clear_navigation_path(self) -> None:
        self.path = np.empty((0, 3), dtype=np.float32)
        self._route_ribbon_key = None
        self._route_ribbon = None
        self.path_frame = self.map_frame
        self.active_path_topic = ""
        self.path_error = ""
        self._guidance_progress = 0.
        self.ar_status = dict(code="route", message="等待规划路线")

    def send_navigation_waypoints(self, waypoints) -> bool:
        if not waypoints:
            self.navigation_status = "请先添加途径点"
            return False
        if not self.navigation_route_ready():
            self.navigation_status = (
                self._navigation_readiness.reason
                or "路线规划服务尚未就绪")
            self._route_wait_status = self.navigation_status
            return False
        if (getattr(self, "require_localizer_quality", False) and
                not self.localization_tools.health.valid(self.get_clock().now().nanoseconds)):
            self.navigation_status = "地图定位尚未有效，请先完成定位"
            return False
        if self.robot.busy:
            self.navigation_status = "请先取消当前机器人任务并等待确认"
            return False
        if self.walking_pose()[0] is None:
            self.navigation_status = "当前位置未确认，请先完成定位"
            return False
        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.use_start = False
        goal_msg.planner_id = "GridBased"
        for index, waypoint in enumerate(waypoints):
            if index + 1 < len(waypoints):
                direction = waypoints[index + 1] - waypoint
            elif index > 0:
                direction = waypoint - waypoints[index - 1]
            else:
                direction = np.array([1.0, 0.0])
            heading = math.atan2(float(direction[1]), float(direction[0]))
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.grid_frame
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.orientation.z, pose.pose.orientation.w = yaw_quaternion(heading)
            goal_msg.goals.append(pose)
        self.walking.frame = self.grid_frame
        self._planned_waypoints = [
            np.asarray(point, dtype=np.float32) for point in waypoints]
        self.cancel_pending = False
        self.navigation_result_status = None
        return self.walking.plan(goal_msg)

    def walking_pose(self):
        _path, xy, yaw = self.map_navigation_state()
        return xy, yaw

    def _walking_changed(self, session) -> None:
        if self.operating_mode == "robot" and self.robot.busy:
            return
        self.navigation_status = session.status
        if self.robot_profile == "woosh" and self.operating_mode == "walking" and session.phase == "preview":
            self.navigation_status = f"路线已生成，约 {session.remaining:.1f} 米；请在导航页点击“开始导航”，使用手柄控制小车行走"
        self.navigation_active = session.phase in (
            "guiding", "lost", "deviated")
        self._guidance_progress = session.progress
        self.path = session.path
        self.path_frame = session.frame
        self.goal_handle = session.handle
        self.navigation_result_status = session.result
        self.navigation_result_revision = session.result_revision
        self._route_ribbon_key = None

    def begin_guidance(self) -> bool:
        if not self.navigation_route_ready():
            self.navigation_status = (
                self._navigation_readiness.reason or "路线服务尚未激活")
            self._route_wait_status = self.navigation_status
            return False
        current, _yaw = self.walking_pose()
        if current is None:
            self.navigation_status = "当前位置未确认，请保持设备稳定"
            return False
        if self.operating_mode == "robot":
            motion_session = getattr(self, "motion_session", None)
            if motion_session is None or not motion_session.available():
                self.navigation_status = "底盘未连接或反馈未就绪，请检查底盘连接"
                return False
            if self.walking.phase != "preview" or not self._planned_waypoints:
                return False
            goal = NavigateThroughPoses.Goal()
            previous = current
            for xy in self._planned_waypoints:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = self.grid_frame
                pose.pose.position.x = float(xy[0])
                pose.pose.position.y = float(xy[1])
                heading = math.atan2(
                    float(xy[1] - previous[1]),
                    float(xy[0] - previous[0]))
                pose.pose.orientation.z, pose.pose.orientation.w = (
                    yaw_quaternion(heading))
                previous = xy
                goal.poses.append(pose)
            return self.robot.start(goal)
        return self.walking.begin()

    def _robot_changed(self, session) -> None:
        if self.operating_mode != "robot":
            return
        self.navigation_status = session.status
        self.navigation_active = session.phase == "running"
        self.goal_handle = session.handle
        self.navigation_result_status = session.result
        self.navigation_result_revision = session.result_revision
        if session.phase != "running" and session.phase != "preview":
            self.clear_navigation_path()

    def poll_navigation(self) -> None:
        self.walking.poll()
        self.robot.poll()
        if self.operating_mode == "walking":
            self.walking.update_pose(*self.walking_pose())
        elif self.robot.phase == "running" and self.walking_pose()[0] is None:
            self.robot.cancel()
        from std_msgs.msg import Bool
        motion_allowed = bool(
            self.operating_mode == "robot"
            and self.robot.phase == "running"
            and self.walking_pose()[0] is not None)
        self.motion_permit_pub.publish(Bool(data=motion_allowed))
        error = self.motion_session.poll(
            self.robot.generation, motion_allowed)
        if error:
            self.robot.cancel()
            self.navigation_status = error

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS future propagates transport errors here.
            self._set_navigation_terminal(
                -1, f"导航目标发送失败: {exc}")
            return
        if not goal_handle.accepted:
            self._set_navigation_terminal(-2, "导航目标被 Nav2 拒绝")
            return
        # The user may have exited while the asynchronous request was pending.
        if not self.navigation_active:
            goal_handle.cancel_goal_async()
            self.clear_navigation_path()
            return
        self.goal_handle = goal_handle
        self.navigation_status = "导航目标已接受"
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback) -> None:
        if not self.navigation_active:
            return
        current = feedback.feedback.number_of_poses_remaining
        self.navigation_status = f"导航中 · 剩余 {current} 个途径点"

    def _on_navigation_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self._set_navigation_terminal(-3, f"读取导航结果失败: {exc}")
            return
        labels = {
            4: "导航成功",
            5: "导航已取消",
            6: "导航失败",
        }
        self._set_navigation_terminal(
            status, labels.get(status, f"导航结束，状态码 {status}"))

    def _set_navigation_terminal(self, status: int, text: str) -> None:
        """Publish one terminal-state revision for the Qt page controller."""
        self.navigation_status = text
        self.navigation_active = False
        self.clear_navigation_path()
        self.goal_handle = None
        self.cancel_pending = False
        self.navigation_result_status = int(status)
        self.navigation_result_revision += 1

    def cancel_navigation(self) -> None:
        self.navigation_active = False
        if hasattr(self, "walking"):
            self.walking.cancel()
        if hasattr(self, "robot") and self.robot.busy:
            self.cancel_pending = True
            if self.robot.cancel():
                self.navigation_status = "正在取消机器人任务，等待 Nav2 确认"
            else:
                self.navigation_status = "机器人任务取消未确认，请检查任务状态"
            return
        self.clear_navigation_path()
        self.cancel_pending = False
        self.goal_handle = None
        self.navigation_status = "已退出导航"

    def _on_cancel_response(self, future) -> None:
        try:
            response = future.result()
            accepted = bool(getattr(response, "goals_canceling", ()))
        except Exception as exc:
            self.cancel_pending = False
            self.navigation_status = f"导航取消未确认：{exc}"
            return
        if accepted:
            self.navigation_status = "Nav2 已接受取消，等待任务结束"
        else:
            self.cancel_pending = False
            self.navigation_status = "Nav2 未确认取消，请检查任务状态"

    def guidance_state(self):
        from .navigation_display import guidance_state
        return guidance_state(self)

    def _grounded_route(self, stamp):
        from .ar_projection import grounded_route
        return grounded_route(self, stamp)


class ActiveNavigationPage(QWidget):
    """Distraction-free camera view used while a task is active."""

    def __init__(
        self, exit_callback, mode_callback=None, begin_callback=None,
        camera_topic: str = "/color/image_raw", rotate_180: bool = False,
    ) -> None:
        super().__init__()
        self._mode_callback = mode_callback
        self.camera_panel = CameraPanel(
            show_status=True, fill=True, camera_topic=camera_topic, rotate_180=rotate_180)
        self.camera_panel.setParent(self)
        self.map_panel = MapPanel(cloud_3d=True)
        self.map_panel.setObjectName("activeMap")
        self.map_panel.set_edit_tool("navigate", 0.20)
        self.map_panel.setParent(self)
        self.grid_mode_button = QPushButton("栅格")
        self.grid_mode_button.setObjectName("mapModeButton")
        self.grid_mode_button.setCheckable(True)
        self.grid_mode_button.setChecked(True)
        self.grid_mode_button.clicked.connect(lambda: self.set_map_mode("grid"))
        self.grid_mode_button.setParent(self)
        self.cloud_mode_button = QPushButton("点云")
        self.cloud_mode_button.setObjectName("mapModeButton")
        self.cloud_mode_button.setCheckable(True)
        self.cloud_mode_button.clicked.connect(lambda: self.set_map_mode("cloud"))
        self.cloud_mode_button.setParent(self)
        self.follow_button = QPushButton("跟随")
        self.follow_button.setObjectName("mapModeButton")
        self.follow_button.setCheckable(True)
        self.follow_button.toggled.connect(
            self.map_panel.set_navigation_follow)
        self.map_panel.navigation_follow_changed.connect(self.follow_button.setChecked)
        self.follow_button.setParent(self)
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_group.addButton(self.grid_mode_button)
        self.mode_group.addButton(self.cloud_mode_button)
        self.exit_button = QPushButton("退出导航")
        self.exit_button.setObjectName("exitNavigationButton")
        self.exit_button.clicked.connect(exit_callback)
        self.exit_button.setParent(self)
        self.begin_button = QPushButton("开始导航")
        self.begin_button.setObjectName("primaryButton")
        self.begin_button.clicked.connect(begin_callback)
        self.begin_button.setParent(self)
        self.begin_button.hide()
        self.result_overlay = QLabel(self)
        self.result_overlay.setObjectName("navigationResultOverlay")
        self.result_overlay.setAlignment(Qt.AlignCenter)
        self.result_overlay.setWordWrap(True)
        self.result_overlay.hide()

    def show_navigation_result(self, status: Optional[int], text: str) -> None:
        if status == 4:
            title = "✓ 已到达目的地"
            detail = "导航任务完成，点击“返回地图”结束本次导航"
            border = "#43d17d"
            background = "rgba(10, 42, 27, 235)"
        elif status == 5:
            title = "导航已取消"
            detail = "点击“返回地图”结束本次导航"
            border = "#ffc857"
            background = "rgba(49, 39, 13, 235)"
        else:
            title = "导航已结束"
            detail = text
            border = "#ff737b"
            background = "rgba(50, 19, 24, 235)"
        self.result_overlay.setText(f"{title}\n{detail}")
        self.result_overlay.setStyleSheet(
            "QLabel#navigationResultOverlay {"
            f"background: {background}; border: 2px solid {border};"
            "border-radius: 16px; color: #f5f8fa;"
            "font-size: 18px; font-weight: 700; padding: 18px;"
            "}")
        self.result_overlay.show()
        self.result_overlay.raise_()
        self.exit_button.setText("返回地图")

    def clear_navigation_result(self) -> None:
        self.result_overlay.hide()
        self.result_overlay.clear()
        self.exit_button.setText("退出导航")

    def set_preview_available(self, available: bool) -> None:
        self.begin_button.setVisible(bool(available))
        if available:
            self.begin_button.raise_()

    def set_map_mode(self, mode: str, notify: bool = True) -> None:
        self.map_panel.set_display_mode(mode)
        self.grid_mode_button.setChecked(mode == "grid")
        self.cloud_mode_button.setChecked(mode == "cloud")
        if notify and self._mode_callback is not None:
            self._mode_callback(mode)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        compact = min(self.width(), self.height()) < 700
        self.camera_panel.setGeometry(self.rect())
        if self.width() < self.height():
            map_w = min(260, max(180, int(self.width() * 0.44)))
        else:
            map_w = min(370, max(220, int(self.width() * 0.34)))
        map_h = int(map_w * 0.68)
        map_x = 10 if compact else 26
        map_y = self.height() - map_h - (78 if compact else 88)
        self.camera_panel.set_status_visible(True)
        self.map_panel.setGeometry(map_x, map_y, map_w, map_h)
        self.grid_mode_button.setGeometry(map_x + 10, map_y + 8, 58, 30)
        self.cloud_mode_button.setGeometry(map_x + 72, map_y + 8, 58, 30)
        self.follow_button.setGeometry(map_x + 134, map_y + 8, 58, 30)
        button_w = min(190, max(150, self.width() - 120))
        button_h = 36 if compact else 40
        inset_x, inset_y = (8, 6) if compact else (22, 18)
        self.exit_button.setGeometry(
            self.width() - button_w - inset_x, inset_y, button_w, button_h)
        self.begin_button.setGeometry(
            self.width() - button_w - inset_x,
            inset_y + button_h + 8,
            button_w,
            button_h,
        )
        self.exit_button.setToolTip("结束当前导航并返回地图")
        self.camera_panel.status_right_margin = button_w + inset_x + 8
        overlay_w = min(500, max(240, self.width() - 28))
        overlay_h = 110 if compact else 126
        self.result_overlay.setGeometry(
            (self.width() - overlay_w) // 2,
            max(74, int(self.height() * 0.18)),
            overlay_w,
            overlay_h,
        )
        self.map_panel.raise_()
        self.grid_mode_button.raise_()
        self.cloud_mode_button.raise_()
        self.follow_button.raise_()
        self.exit_button.raise_()
        if not self.begin_button.isHidden():
            self.begin_button.raise_()
        if not self.result_overlay.isHidden():
            self.result_overlay.raise_()


class NavigationWindow(QMainWindow):
    def __init__(self, node: QtNavRosNode) -> None:
        super().__init__()
        self.node = node
        self._mapping_start_error = ""
        self.map_storage_dir = default_map_directory(str(
            getattr(node, "map_storage_dir", "")))
        self.map_storage_dir.mkdir(parents=True, exist_ok=True)
        self.waypoints = []
        self.waypoint_edit_index: Optional[int] = None
        self.last_map_revision = -1
        self.last_pointcloud_revision = -1
        self.last_navigation_result_revision = int(
            getattr(node, "navigation_result_revision", 0))
        self.local_grid: Optional[GridMap] = None
        self.local_cloud: Optional[PointCloudMap] = None
        self._grid_original: Optional[np.ndarray] = None
        self._cloud_original: Optional[np.ndarray] = None
        self._cloud_original_colors: Optional[np.ndarray] = None
        self._grid_edit_revision = 0
        self._cloud_edit_revision = 0
        self._rendered_grid_edit_revision = -1
        self._rendered_cloud_edit_revision = -1
        self._undo_stack = []
        self.sensor_driver_controller = SensorDriverController(self)
        self.sensor_processes = self.sensor_driver_controller.processes
        self.navigation_stack_controller = NavigationStackController(
            self.sensor_driver_controller,
            sensor_active=self._navigation_sensor_active,
            parent=self,
            component_ready=getattr(node, "navigation_component_ready", None),
        )
        if hasattr(node, "_navigation_readiness"):
            self.navigation_stack_controller.navigation_log = (
                node._navigation_readiness.observe_log)
        localization_adapter = getattr(
            node, "localization_tools", UnavailableLocalizationRosAdapter())
        self.localization_tools_controller = LocalizationToolsController(
            localization_adapter, parent=self)
        self.sensor_preview_adapter = getattr(
            node, "sensor_preview", UnavailableSensorPreviewAdapter())
        self.sensor_state_labels = {}
        self.sensor_start_buttons = {}
        self.sensor_stop_buttons = {}
        self.sensor_log_views = {}
        self.sensor_topic_combos = {}
        self.sensor_preview_status_labels = {}
        self.last_sensor_preview_revisions = dict(
            self.sensor_preview_adapter.revisions)
        self._cached_route = None
        self._ar_frame_key = None
        self._cached_remaining_distance = 0.0
        self._cached_guidance = None
        self._last_route_refresh = 0.0
        self._last_map_navigation_refresh = 0.0
        self._motion_trace = []
        self._navigation_overlay_enabled = False
        self._compact_mode: Optional[bool] = None
        self._layout_profile = ""
        self.setWindowTitle("NAV 导航控制台")
        screen = QApplication.primaryScreen()
        available = screen.availableGeometry() if screen is not None else None
        initial_width = min(1500, available.width()) if available else 1500
        initial_height = min(900, available.height()) if available else 900
        self.resize(initial_width, initial_height)
        # Fractional scaling can expose a 480x800 panel as roughly 320x533
        # logical pixels, so keep the hard minimum below that logical size.
        self.setMinimumSize(320, 280)
        self._build_ui()
        self.fullscreen_shortcut = QShortcut(QKeySequence("F11"), self)
        self.fullscreen_shortcut.activated.connect(self.toggle_fullscreen)
        self.leave_fullscreen_shortcut = QShortcut(QKeySequence("Escape"), self)
        self.leave_fullscreen_shortcut.activated.connect(self.showNormal)
        self.leave_fullscreen_shortcut.setEnabled(False)
        self._set_compact_mode(min(initial_width, initial_height) < 700)
        self._load_configured_pointcloud()
        self.pending_navigation_result_status: Optional[int] = None
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setTimerType(Qt.PreciseTimer)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(33)

    def _build_ui(self) -> None:
        """Build three distinct workspaces: navigation, map tools, active nav."""
        root = QWidget()
        layout = QVBoxLayout(root)
        self.setup_layout = layout
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        # Actions use a second row so all controls remain reachable at 480 px.
        header = QVBoxLayout()
        self.setup_header = header
        title = QLabel("NAV")
        self.main_title_label = title
        title.setObjectName("title")
        subtitle = QLabel("实时相机 · 栅格地图 · Nav2 全局导航")
        subtitle.setObjectName("subtitle")
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        title_row = QHBoxLayout()
        title_row.addWidget(title, 1)
        self.setup_home_button = QPushButton("系统首页")
        self.setup_home_button.setObjectName("backButton")
        self.setup_home_button.clicked.connect(self.show_system_home)
        title_row.addWidget(self.setup_home_button)
        title_box.addLayout(title_row)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header_actions = QHBoxLayout()
        header_actions.setSpacing(6)
        header_actions.addStretch(1)
        self.open_navigation_stack_button = QPushButton("导航系统")
        self.open_navigation_stack_button.setObjectName("workspaceButton")
        self.open_navigation_stack_button.clicked.connect(
            self.show_navigation_stack)
        header_actions.addWidget(self.open_navigation_stack_button)
        self.open_relocalization_button = QPushButton("重新定位")
        self.open_relocalization_button.setObjectName("workspaceButton")
        self.open_relocalization_button.clicked.connect(
            self.show_relocalization)
        header_actions.addWidget(self.open_relocalization_button)
        self.open_sensor_tools_button = QPushButton("传感器管理")
        self.open_sensor_tools_button.setObjectName("workspaceButton")
        self.open_sensor_tools_button.clicked.connect(self.show_sensor_tools)
        header_actions.addWidget(self.open_sensor_tools_button)
        self.open_mapping_button = QPushButton("开始建图")
        self.open_mapping_button.setObjectName("workspaceButton")
        self.open_mapping_button.clicked.connect(self.start_mapping_from_home)
        header_actions.addWidget(self.open_mapping_button)
        self.open_map_tools_button = QPushButton("地图处理")
        self.open_map_tools_button.setObjectName("workspaceButton")
        self.open_map_tools_button.clicked.connect(self.show_map_tools)
        header_actions.addWidget(self.open_map_tools_button)
        self.open_operations_button = QPushButton("工具")
        self.open_operations_button.setObjectName("workspaceButton")
        self.open_operations_button.clicked.connect(self.show_operations)
        header_actions.addWidget(self.open_operations_button)
        self.ros_status = QLabel("ROS 2 ONLINE")
        self.ros_status.setObjectName("onlineChip")
        header_actions.addWidget(self.ros_status)
        self.fullscreen_button = QPushButton("全屏")
        self.fullscreen_button.setObjectName("secondaryButton")
        self.fullscreen_button.setToolTip("F11 切换全屏，Esc 返回窗口")
        self.fullscreen_button.clicked.connect(self.toggle_fullscreen)
        header_actions.addWidget(self.fullscreen_button)
        self.exit_app_button = QPushButton("退出程序")
        self.exit_app_button.setObjectName("dangerButton")
        self.exit_app_button.clicked.connect(self.close)
        header_actions.addWidget(self.exit_app_button)
        # The system home owns these task entries; navigation keeps its own preparation controls.
        for button in (self.open_sensor_tools_button, self.open_mapping_button,
                       self.open_map_tools_button, self.open_operations_button):
            button.hide()
        header.addLayout(header_actions)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        self.setup_splitter = splitter
        map_card = QFrame()
        map_card.setObjectName("sidePanel")
        map_layout = QVBoxLayout(map_card)
        self.setup_map_layout = map_layout
        map_layout.setContentsMargins(16, 16, 16, 16)
        map_layout.setSpacing(10)
        map_header = QHBoxLayout()
        map_title = QLabel("导航地图")
        map_title.setObjectName("sectionTitle")
        map_header.addWidget(map_title)
        map_header.addStretch(1)
        self.grid_mode_button = QPushButton("栅格")
        self.grid_mode_button.setObjectName("mapModeButton")
        self.grid_mode_button.setCheckable(True)
        self.grid_mode_button.setChecked(True)
        self.cloud_mode_button = QPushButton("点云")
        self.cloud_mode_button.setObjectName("mapModeButton")
        self.cloud_mode_button.setCheckable(True)
        self.map_mode_group = QButtonGroup(self)
        self.map_mode_group.setExclusive(True)
        self.map_mode_group.addButton(self.grid_mode_button)
        self.map_mode_group.addButton(self.cloud_mode_button)
        self.grid_mode_button.clicked.connect(
            lambda: self.set_navigation_map_mode("grid"))
        self.cloud_mode_button.clicked.connect(
            lambda: self.set_navigation_map_mode("cloud"))
        map_header.addWidget(self.grid_mode_button)
        map_header.addWidget(self.cloud_mode_button)
        map_layout.addLayout(map_header)
        self.map_panel = MapPanel(cloud_3d=True)
        self.map_panel.setObjectName("mapPanel")
        self.map_panel.point_selected.connect(self.on_map_point)
        self.map_panel.set_edit_tool("navigate", 0.20)
        map_layout.addWidget(self.map_panel, 1)

        hint = QLabel(
            "单击添加途径点；点云模式可左键拖动旋转、右键平移、滚轮缩放")
        hint.setObjectName("hint")
        map_layout.addWidget(hint)
        self.setup_map_card = map_card
        splitter.addWidget(map_card)

        side = QFrame()
        side.setObjectName("sidePanel")
        side_layout = QVBoxLayout(side)
        self.setup_side_layout = side_layout
        side_layout.setContentsMargins(14, 14, 14, 14)
        side_layout.setSpacing(10)
        camera_title = QLabel("实时相机")
        camera_title.setObjectName("sectionTitle")
        side_layout.addWidget(camera_title)
        camera_topic = str(getattr(
            self.node, "camera_topic", "/color/image_raw"))
        self.camera_panel = CameraPanel(
            show_status=False, compact=True, camera_topic=camera_topic,
            rotate_180=getattr(self.node, "robot_profile", "") == "woosh")
        self.camera_panel.setObjectName("card")
        self.camera_panel.setMinimumHeight(170)
        self.camera_panel.setMaximumHeight(260)
        side_layout.addWidget(self.camera_panel)

        self.waypoint_title = QLabel("途径点（尚未添加）")
        self.waypoint_title.setObjectName("sectionTitle")
        side_layout.addWidget(self.waypoint_title)
        self.waypoint_list = QListWidget()
        self.waypoint_list.setObjectName("waypointList")
        self.waypoint_list.setMinimumHeight(130)
        self.waypoint_list.currentRowChanged.connect(
            self._on_waypoint_selection_changed)
        self.waypoint_list.itemDoubleClicked.connect(
            self.begin_waypoint_reselection)
        side_layout.addWidget(self.waypoint_list, 1)

        waypoint_actions = QHBoxLayout()
        self.edit_waypoint_button = QPushButton("重新选点")
        self.edit_waypoint_button.setObjectName("secondaryButton")
        self.edit_waypoint_button.setEnabled(False)
        self.edit_waypoint_button.clicked.connect(
            self.begin_waypoint_reselection)
        self.delete_waypoint_button = QPushButton("删除选中点")
        self.delete_waypoint_button.setObjectName("dangerButton")
        self.delete_waypoint_button.setEnabled(False)
        self.delete_waypoint_button.clicked.connect(
            self.delete_selected_waypoint)
        waypoint_actions.addWidget(self.edit_waypoint_button)
        waypoint_actions.addWidget(self.delete_waypoint_button)
        side_layout.addLayout(waypoint_actions)

        buttons = QHBoxLayout()
        self.clear_button = QPushButton("清空途径点")
        self.clear_button.setObjectName("dangerButton")
        self.clear_button.clicked.connect(self.clear_selection)
        self.start_button = QPushButton("规划路线" if getattr(self.node, "robot_profile", "") == "woosh" else "开始导航")
        self.start_button.setObjectName("primaryButton")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_navigation)
        buttons.addWidget(self.clear_button)
        buttons.addWidget(self.start_button)
        side_layout.addLayout(buttons)
        self.nav_status = QLabel(self.node.navigation_status)
        self.nav_status.setWordWrap(True)
        self.nav_status.setObjectName("statusBar")
        side_layout.addWidget(self.nav_status)

        self.setup_side_panel = side
        splitter.addWidget(side)
        splitter.setSizes([1080, 380])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        layout.addWidget(splitter, 1)
        self.setup_page = root
        self.active_page = ActiveNavigationPage(
            self.exit_navigation,
            self.set_navigation_map_mode,
            self.begin_navigation_guidance,
            camera_topic=camera_topic,
            rotate_180=getattr(self.node, "robot_profile", "") == "woosh",
        )
        if getattr(self.node, "operating_mode", "robot") == "walking":
            self.active_page.begin_button.setToolTip("使用手柄控制行走，显示方向和到达提示")
        self.map_tools_page = self._build_map_tools_page()
        self.sensor_tools_page = self._build_sensor_tools_page()
        self.sensor_monitor_page = self._build_sensor_monitor_page()
        self.navigation_stack_page = NavigationStackPage(
            self.navigation_stack_controller,
            self.map_storage_dir,
            self._lidar_launch_arguments,
            self._camera_launch_arguments,
            self.localization_tools_controller,
            robot_profile=getattr(self.node, "robot_profile", ""),
            operating_mode=getattr(self.node, "operating_mode", "robot"),
        )
        self.navigation_stack_page.return_requested.connect(
            self.show_system_home)
        self.navigation_stack_page.localization_map_selected.connect(
            self.load_navigation_localization_map)
        from .ui.relocalization import ManualRelocalizationPage
        self.relocalization_page = ManualRelocalizationPage(self)
        self.relocalization_page.back_requested.connect(
            self.show_navigation_setup)
        self.relocalization_page.submit_requested.connect(
            self.submit_manual_relocalization)
        self.relocalization_page.automatic_requested.connect(
            self.trigger_automatic_relocalization)
        self.relocalization_page.timeout_changed.connect(
            self.relocalization_timeout_changed)
        self.relocalization_page.navigation_requested.connect(
            self.show_navigation_setup)
        self.relocalization_page.export_requested.connect(
            self.show_relocalization_diagnostics)
        self.localization_tools_controller.operation_state_changed.connect(
            self._relocalization_operation_changed)
        mapping_adapter = getattr(
            self.node, "mapping", UnavailableMappingRosAdapter())
        self.mapping_controller = MappingController(
            mapping_adapter, parent=self)
        self.mapping_page = MappingPage(
            self.mapping_controller, MapPanel, self.map_storage_dir)
        self.mapping_workspace = MappingWorkspace(self.mapping_page)
        self.mapping_workspace.return_requested.connect(self.show_system_home)
        self.mapping_workspace.settings_requested.connect(self.show_mapping_settings)
        mapping_backend = getattr(self.node, "mapping_backend", "")
        if mapping_backend:
            self.mapping_page.algorithm_combo.setCurrentIndex(
                self.mapping_page.algorithm_combo.findData(mapping_backend))
        self.navigation_stack_controller.start_guard = lambda: (
            not self.mapping_controller.process.active, "建图正在运行，请先停止建图")
        self.mapping_controller.start_guard = lambda: (
            self.navigation_stack_controller.overall_state not in ("starting", "running", "stopping"),
            "导航系统尚未停止，请先停止导航系统")
        if hasattr(self.node, "configure_navigation"):
            self.navigation_stack_controller.configure_navigation = self.node.configure_navigation
            self.mapping_controller.configure_clock = self.node.set_clock_mode
        self.mapping_page.return_requested.connect(self.show_mapping)
        self.pages = QStackedWidget()
        self.pages.addWidget(self.setup_page)
        self.pages.addWidget(self.sensor_tools_page)
        self.pages.addWidget(self.sensor_monitor_page)
        self.pages.addWidget(self.navigation_stack_page)
        self.pages.addWidget(self.relocalization_page)
        self.pages.addWidget(self.mapping_page)
        self.pages.addWidget(self.mapping_workspace)
        self.pages.addWidget(self.active_page)
        from .ui.operations import OperationsController
        self.operations_page = OperationsController(self)
        self.sensor_pages.addTab(self.operations_page.rtk, "RTK")
        self.sensor_monitor_tabs.addTab(self.operations_page.trajectory, "轨迹测试")
        from .ui.workspace_page import WorkspacePage
        self.map_workspace_page = WorkspacePage("NAV · 地图管理", self.show_system_home,
            subtitle="地图包 · 地图处理 · 处理进度")
        self.map_pages = QTabWidget()
        self.map_pages.addTab(self.operations_page.library, "地图包")
        self.map_pages.addTab(self.map_tools_page, "地图处理")
        self.map_pages.addTab(self.operations_page.map_tasks, "处理进度")
        self.map_workspace_page.body.addWidget(self.map_pages)
        self.recordings_workspace_page = WorkspacePage("NAV · 录包与回放", self.show_system_home,
            subtitle="采集 · 回放 · 录包管理")
        self.recordings_workspace_page.body.addWidget(self.operations_page.bags)
        self.logs_workspace_page = WorkspacePage("NAV · 日志管理", self.show_system_home)
        self.logs_workspace_page.body.addWidget(self.operations_page.logs)
        self.diagnostics_workspace_page = WorkspacePage("NAV · 系统诊断", self.show_system_home)
        self.diagnostics_workspace_page.body.addWidget(self.operations_page.diagnostics_page)
        self.phone_workspace_page = WorkspacePage("NAV · 手机遥控", self.show_system_home)
        self.phone_workspace_page.body.addWidget(self.operations_page.phone)
        self.task_workspaces = (self.map_workspace_page, self.recordings_workspace_page,
            self.logs_workspace_page, self.diagnostics_workspace_page, self.phone_workspace_page)
        for page in self.task_workspaces:
            self.pages.addWidget(page)
        from .ui.manual_drive import ManualDrivePage
        self.manual_drive_page = ManualDrivePage(self)
        self.pages.addWidget(self.manual_drive_page)
        from .ui.system_home import SystemHomePage, SystemSettingsPage
        self.system_home_page = SystemHomePage()
        self.system_home_page.mapping_requested.connect(self.start_mapping_from_home)
        self.system_home_page.navigation_requested.connect(self.enter_navigation)
        self.system_home_page.maps_requested.connect(self.show_map_library)
        self.system_home_page.settings_requested.connect(self.show_system_settings)
        self.system_home_page.phone_requested.connect(self.show_phone_remote)
        self.system_home_page.drive_requested.connect(self.show_manual_drive)
        self.system_home_page.mapping_stop_requested.connect(self.stop_home_mapping)
        self.system_home_page.navigation_stop_requested.connect(self.stop_home_navigation)
        self.system_home_page.map_tasks_requested.connect(self.show_home_map_tasks)
        self.system_home_page.mapping_settings_requested.connect(self.show_mapping_settings)
        from .ui.navigation_access import NavigationAccessPage
        self.navigation_access_page = NavigationAccessPage(
            self.navigation_stack_page, self.show_navigation_stack,
            getattr(self.node, "robot_profile", ""), self)
        self.pages.addWidget(self.navigation_access_page)
        self.navigation_stack_page.access_requested.connect(self.show_navigation_access)
        self.navigation_stack_controller.overall_state_changed.connect(self._navigation_stack_status_changed)
        self.system_settings_page = SystemSettingsPage(self)
        self.pages.insertWidget(0, self.system_home_page)
        self.pages.insertWidget(1, self.system_settings_page)
        self.pages.setCurrentWidget(self.system_home_page)
        self._refresh_system_home()
        self._closing_after_record = False
        self.operations_page.controller.record_process.state_changed.connect(
            lambda state: self._record_close_state("record", state))
        self.setCentralWidget(self.pages)

    def _build_sensor_tools_page(self) -> QWidget:
        """Build the local driver launcher and its live log console."""
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        header = PageHeader("NAV · 传感器", self.show_system_home,
            destination="返回系统首页", subtitle="雷达 · 相机 · IMU · RTK")
        self.sensor_tools_header = header
        monitor_button = QPushButton("传感器数据")
        monitor_button.setObjectName("workspaceButton")
        monitor_button.clicked.connect(self.show_sensor_monitor)
        header.add_action(monitor_button)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        self.sensor_tools_splitter = splitter
        controls = QFrame()
        controls.setObjectName("sidePanel")
        controls.setMinimumWidth(440)
        self.sensor_tools_controls = controls
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(16, 16, 16, 16)
        controls_layout.setSpacing(12)
        control_title = QLabel("驱动控制")
        control_title.setObjectName("sectionTitle")
        controls_layout.addWidget(control_title)
        self.sensor_control_tabs = QTabWidget()
        self.sensor_control_tabs.addTab(self._build_lidar_controls(), "雷达")
        self.sensor_control_tabs.addTab(self._build_camera_controls(), "相机")
        controls_layout.addWidget(self.sensor_control_tabs, 1)
        splitter.addWidget(controls)

        log_card = QFrame()
        log_card.setObjectName("sidePanel")
        log_layout = QVBoxLayout(log_card)
        log_layout.setContentsMargins(16, 16, 16, 16)
        log_header = QHBoxLayout()
        log_title = QLabel("启动日志")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        clear_button = QPushButton("清空日志")
        clear_button.setObjectName("secondaryButton")
        clear_button.clicked.connect(self._clear_sensor_logs)
        log_header.addWidget(clear_button)
        log_layout.addLayout(log_header)
        self.sensor_log_tabs = QTabWidget()
        for key, label in (("all", "全部"), ("lidar", "雷达"), ("camera", "相机")):
            view = QPlainTextEdit()
            view.setObjectName("sensorLog")
            view.setReadOnly(True)
            view.setLineWrapMode(QPlainTextEdit.NoWrap)
            view.document().setMaximumBlockCount(3000)
            self.sensor_log_views[key] = view
            self.sensor_log_tabs.addTab(view, label)
        log_layout.addWidget(self.sensor_log_tabs, 1)
        splitter.addWidget(log_card)
        splitter.setSizes([500, 900])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        self.sensor_pages = QTabWidget()
        self.sensor_pages.addTab(splitter, "雷达 · 相机 · IMU")
        layout.addWidget(self.sensor_pages, 1)

        self.sensor_driver_controller.log_received.connect(
            self._append_sensor_log)
        self.sensor_driver_controller.state_changed.connect(
            self._handle_sensor_state)
        for key in ("lidar", "camera"):
            self._handle_sensor_state(key, "stopped")
        return root

    def _build_sensor_monitor_page(self) -> QWidget:
        """Build topic-selectable live views for lidar, camera, and IMU."""
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        header = PageHeader("NAV · 传感器数据", self.leave_sensor_monitor,
            destination="返回传感器管理", subtitle="雷达点云 · 相机视频 · IMU · 轨迹测试")
        self.sensor_monitor_header = header
        refresh_button = QPushButton("刷新话题")
        refresh_button.setObjectName("secondaryButton")
        refresh_button.clicked.connect(self.refresh_sensor_topics)
        header.add_action(refresh_button)
        layout.addLayout(header)

        self.sensor_monitor_tabs = QTabWidget()
        self.sensor_monitor_tabs.setObjectName("sensorMonitorTabs")
        self.sensor_monitor_tabs.addTab(
            self._build_lidar_monitor_tab(), "雷达点云")
        self.sensor_monitor_tabs.addTab(
            self._build_camera_monitor_tab(), "相机视频")
        self.sensor_monitor_tabs.addTab(
            self._build_imu_monitor_tab(), "IMU")
        layout.addWidget(self.sensor_monitor_tabs, 1)
        return root

    def _sensor_topic_controls(self, kind: str, placeholder: str) -> QWidget:
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel("话题")
        label.setObjectName("sectionTitle")
        combo = QComboBox()
        combo.setEditable(True)
        combo.setInsertPolicy(QComboBox.NoInsert)
        combo.lineEdit().setPlaceholderText(placeholder)
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        start = QPushButton("开始显示")
        start.setObjectName("primaryButton")
        start.clicked.connect(
            lambda: self._start_sensor_preview(kind))
        stop = QPushButton("停止显示")
        stop.setObjectName("dangerButton")
        stop.clicked.connect(
            lambda: self._stop_sensor_preview(kind))
        layout.addWidget(label)
        layout.addWidget(combo, 1)
        layout.addWidget(stop)
        layout.addWidget(start)
        self.sensor_topic_combos[kind] = combo
        return root

    def _sensor_preview_status(self, kind: str, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("sensorPreviewStatus")
        label.setWordWrap(True)
        self.sensor_preview_status_labels[kind] = label
        return label

    def _build_lidar_monitor_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.addWidget(self._sensor_topic_controls(
            "lidar", "选择 PointCloud2 或 Livox CustomMsg 话题"))
        layout.addWidget(self._sensor_preview_status(
            "lidar", "请选择 PointCloud2/Livox CustomMsg 话题并点击开始显示"))
        self.sensor_preview_cloud_panel = MapPanel(cloud_3d=True)
        self.sensor_preview_cloud_panel.setObjectName("sensorPreviewCloud")
        self.sensor_preview_cloud_panel.set_display_mode("cloud")
        layout.addWidget(self.sensor_preview_cloud_panel, 1)
        hint = QLabel(
            "左键拖动旋转 · 右键/中键平移 · 滚轮缩放 · 双击恢复视角")
        hint.setObjectName("hint")
        layout.addWidget(hint)
        return root

    def _build_camera_monitor_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.addWidget(self._sensor_topic_controls(
            "camera", "选择或输入 sensor_msgs/msg/Image 话题"))
        layout.addWidget(self._sensor_preview_status(
            "camera", "请选择 Image 话题并点击开始显示"))
        self.sensor_preview_camera_panel = CameraPanel(
            show_status=False,
            camera_topic=str(getattr(
                self.node, "camera_topic", "/color/image_raw")),
        )
        self.sensor_preview_camera_panel.setObjectName("sensorPreviewCamera")
        layout.addWidget(self.sensor_preview_camera_panel, 1)
        return root

    def _build_imu_monitor_tab(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.addWidget(self._sensor_topic_controls(
            "imu", "选择或输入 sensor_msgs/msg/Imu 话题"))
        layout.addWidget(self._sensor_preview_status(
            "imu", "请选择 Imu 话题并点击开始显示"))
        self.sensor_preview_imu_text = QPlainTextEdit()
        self.sensor_preview_imu_text.setObjectName("imuData")
        self.sensor_preview_imu_text.setReadOnly(True)
        self.sensor_preview_imu_text.setPlainText(
            "等待 IMU 数据…\n\n"
            "将显示姿态四元数、角速度、线加速度和协方差。")
        layout.addWidget(self.sensor_preview_imu_text, 1)
        return root

    def _sensor_action_row(self, key: str, start_text: str) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 8, 0, 0)
        status = QLabel("未启动")
        status.setObjectName("sensorStatus")
        start = QPushButton(start_text)
        start.setObjectName("primaryButton")
        stop = QPushButton("停止")
        stop.setObjectName("dangerButton")
        start.clicked.connect(lambda: self._start_sensor(key))
        stop.clicked.connect(lambda: self._stop_sensor(key))
        layout.addWidget(status)
        layout.addStretch(1)
        layout.addWidget(stop)
        layout.addWidget(start)
        self.sensor_state_labels[key] = status
        self.sensor_start_buttons[key] = start
        self.sensor_stop_buttons[key] = stop
        return widget

    def _build_lidar_controls(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lidar_xfer_format = QComboBox()
        self.lidar_xfer_format.addItem("PointCloud2 + CustomMsg", 4)
        self.lidar_xfer_format.addItem("PointCloud2", 0)
        form.addRow("输出格式", self.lidar_xfer_format)
        self.lidar_publish_frequency = QDoubleSpinBox()
        self.lidar_publish_frequency.setRange(1.0, 100.0)
        self.lidar_publish_frequency.setDecimals(1)
        self.lidar_publish_frequency.setValue(10.0)
        self.lidar_publish_frequency.setSuffix(" Hz")
        form.addRow("发布频率", self.lidar_publish_frequency)
        self.lidar_frame_id = QLineEdit("livox_frame")
        form.addRow("坐标系 frame_id", self.lidar_frame_id)
        self.lidar_multi_topic = QCheckBox("每台雷达使用独立话题")
        if getattr(self.node, "robot_profile", "") == "woosh":
            self.lidar_xfer_format.setCurrentIndex(self.lidar_xfer_format.findData(0))
            self.lidar_frame_id.setText("base_footprint")
            self.lidar_multi_topic.setChecked(True)
        form.addRow("多话题", self.lidar_multi_topic)
        config_row = QWidget()
        config_layout = QHBoxLayout(config_row)
        config_layout.setContentsMargins(0, 0, 0, 0)
        self.lidar_config_path = QLineEdit()
        self.lidar_config_path.setPlaceholderText("留空使用 MID360_config.json")
        browse = QPushButton("选择")
        browse.setObjectName("secondaryButton")
        browse.clicked.connect(self._choose_lidar_config)
        config_layout.addWidget(self.lidar_config_path, 1)
        config_layout.addWidget(browse)
        form.addRow("配置文件", config_row)
        layout.addLayout(form)
        hint = QLabel(
            "对应 livox_ros_driver2/msg_MID360_launch.py。参数会作为 ROS 2 "
            "launch 参数传入，不会修改驱动配置文件。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addStretch(1)
        layout.addWidget(self._sensor_action_row("lidar", "雷达启动"))
        return root

    def _build_camera_controls(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.camera_name_input = QLineEdit("camera")
        self.camera_namespace_input = QLineEdit("camera")
        self.camera_serial_input = QLineEdit()
        self.camera_serial_input.setPlaceholderText("留空自动选择设备")
        form.addRow("相机名称", self.camera_name_input)
        form.addRow("命名空间", self.camera_namespace_input)
        form.addRow("设备序列号", self.camera_serial_input)
        self.camera_enable_color = QCheckBox("彩色图像")
        self.camera_enable_depth = QCheckBox("深度图像")
        self.camera_enable_gyro = QCheckBox("陀螺仪")
        self.camera_enable_accel = QCheckBox("加速度计")
        for checkbox in (
            self.camera_enable_color, self.camera_enable_depth,
            self.camera_enable_gyro, self.camera_enable_accel,
        ):
            checkbox.setChecked(True)
        stream_row = QWidget()
        stream_layout = QVBoxLayout(stream_row)
        stream_layout.setContentsMargins(0, 0, 0, 0)
        stream_layout.setSpacing(4)
        first_stream_row = QHBoxLayout()
        first_stream_row.addWidget(self.camera_enable_color)
        first_stream_row.addWidget(self.camera_enable_depth)
        first_stream_row.addStretch(1)
        second_stream_row = QHBoxLayout()
        second_stream_row.addWidget(self.camera_enable_gyro)
        second_stream_row.addWidget(self.camera_enable_accel)
        second_stream_row.addStretch(1)
        stream_layout.addLayout(first_stream_row)
        stream_layout.addLayout(second_stream_row)
        form.addRow("数据流", stream_row)
        self.camera_enable_sync = QCheckBox("同步彩色与深度")
        self.camera_enable_sync.setChecked(True)
        self.camera_align_depth = QCheckBox("深度对齐到彩色图")
        self.camera_align_depth.setChecked(True)
        self.camera_pointcloud = QCheckBox("由相机生成点云")
        form.addRow("同步", self.camera_enable_sync)
        form.addRow("深度对齐", self.camera_align_depth)
        form.addRow("点云", self.camera_pointcloud)
        self.camera_unite_imu = QComboBox()
        self.camera_unite_imu.addItem("不合并 (0)", "0")
        self.camera_unite_imu.addItem("复制插值 (1)", "1")
        self.camera_unite_imu.addItem("线性插值 (2)", "2")
        self.camera_unite_imu.setCurrentIndex(2)
        form.addRow("IMU 合并方式", self.camera_unite_imu)
        layout.addLayout(form)
        hint = QLabel(
            "对应 realsense2_camera/d435i.launch.py。默认开启 RGB、深度与 IMU，"
            "发布的彩色图可供导航界面订阅。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addStretch(1)
        layout.addWidget(self._sensor_action_row("camera", "相机启动"))
        return root

    def _lidar_launch_arguments(self):
        return self.sensor_driver_controller.lidar_launch_arguments(
            self.lidar_xfer_format.currentData(),
            self.lidar_multi_topic.isChecked(),
            self.lidar_publish_frequency.value(),
            self.lidar_frame_id.text(),
            self.lidar_config_path.text(),
        )

    def _camera_launch_arguments(self):
        return self.sensor_driver_controller.camera_launch_arguments(
            self.camera_name_input.text(),
            self.camera_namespace_input.text(),
            self.camera_serial_input.text(),
            self.camera_enable_color.isChecked(),
            self.camera_enable_depth.isChecked(),
            self.camera_enable_gyro.isChecked(),
            self.camera_enable_accel.isChecked(),
            self.camera_unite_imu.currentData(),
            self.camera_enable_sync.isChecked(),
            self.camera_align_depth.isChecked(),
            self.camera_pointcloud.isChecked(),
        )

    def _choose_lidar_config(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self, "选择 Livox 配置", self.lidar_config_path.text(),
            "JSON 配置 (*.json);;所有文件 (*)")
        if path:
            self.lidar_config_path.setText(path)

    def _start_sensor(self, key: str) -> None:
        builder = (
            self._lidar_launch_arguments
            if key == "lidar" else self._camera_launch_arguments)
        self.sensor_driver_controller.start(key, builder())

    def _stop_sensor(self, key: str) -> None:
        self.sensor_driver_controller.stop(key)

    def _handle_sensor_state(self, key: str, state: str) -> None:
        labels = {
            "starting": ("启动中…", "#ffd166"),
            "running": ("运行中", "#66e09a"),
            "stopping": ("停止中…", "#ffd166"),
            "stopped": ("未启动", "#91a2ad"),
            "error": ("启动失败", "#ff7f88"),
        }
        text, color = labels.get(state, (state, "#91a2ad"))
        label = self.sensor_state_labels.get(key)
        if label is not None:
            label.setText(f"● {text}")
            label.setStyleSheet(f"color: {color}; font-weight: 700;")
        running = state in ("starting", "running", "stopping")
        if key in self.sensor_start_buttons:
            self.sensor_start_buttons[key].setEnabled(not running)
        if key in self.sensor_stop_buttons:
            self.sensor_stop_buttons[key].setEnabled(
                state in ("starting", "running"))

    def _append_sensor_log(self, key: str, text: str) -> None:
        if not text:
            return
        clean = ANSI_ESCAPE_RE.sub("", text).replace("\r", "").rstrip("\n")
        if not clean:
            return
        self.sensor_log_views[key].appendPlainText(clean)
        prefix = "雷达" if key == "lidar" else "相机"
        self.sensor_log_views["all"].appendPlainText(
            "\n".join(f"[{prefix}] {line}" for line in clean.splitlines()))

    def _clear_sensor_logs(self) -> None:
        for view in self.sensor_log_views.values():
            view.clear()

    def _build_map_tools_page(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        splitter = QSplitter(Qt.Horizontal)
        self.map_tools_splitter = splitter
        self.editor_map_panel = MapPanel(cloud_3d=True)
        self.editor_map_panel.setObjectName("mapEditorPanel")
        self.editor_map_panel.edit_started.connect(self.snapshot_map_edit)
        self.editor_map_panel.edit_requested.connect(self.edit_map_at)
        view = QWidget()
        view_layout = QVBoxLayout(view)
        view_layout.setContentsMargins(0, 0, 0, 0)
        gestures = QHBoxLayout()
        gestures.setSpacing(5)
        self.editor_gesture_mode = QLabel("单指：旋转")
        self.editor_gesture_mode.setStyleSheet("font-size: 13px;")
        gestures.addWidget(self.editor_gesture_mode, 1)
        self.editor_rotate_button = QPushButton("旋转")
        self.editor_pan_button = QPushButton("平移")
        self.editor_zoom_out_button = QPushButton("−")
        self.editor_zoom_in_button = QPushButton("＋")
        self.editor_fit_button = QPushButton("复位")
        for button, callback in (
                (self.editor_rotate_button, lambda: self.edit_tool_combo.setCurrentIndex(self.edit_tool_combo.findData("navigate"))),
                (self.editor_pan_button, lambda: self.edit_tool_combo.setCurrentIndex(self.edit_tool_combo.findData("pan"))),
                (self.editor_zoom_out_button, lambda: self.editor_map_panel.zoom_view(.8)),
                (self.editor_zoom_in_button, lambda: self.editor_map_panel.zoom_view(1.25)),
                (self.editor_fit_button, self.editor_map_panel.fit_view)):
            button.setStyleSheet("min-width: 28px; min-height: 34px; padding: 0 7px; font-size: 13px;")
            button.clicked.connect(callback)
            gestures.addWidget(button)
        view_layout.addLayout(gestures)
        view_layout.addWidget(self.editor_map_panel, 1)
        splitter.addWidget(view)

        controls = QFrame()
        controls.setObjectName("sidePanel")
        controls.setMinimumWidth(360)
        controls.setMaximumWidth(440)
        self.map_tools_controls = controls
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(18, 18, 18, 18)
        controls_layout.setSpacing(13)

        mode_header = QHBoxLayout()
        mode_title = QLabel("地图类型")
        mode_title.setObjectName("sectionTitle")
        mode_header.addWidget(mode_title)
        mode_header.addStretch(1)
        self.editor_grid_mode_button = QPushButton("栅格地图")
        self.editor_grid_mode_button.setObjectName("mapModeButton")
        self.editor_grid_mode_button.setCheckable(True)
        self.editor_grid_mode_button.setChecked(True)
        self.editor_cloud_mode_button = QPushButton("点云地图")
        self.editor_cloud_mode_button.setObjectName("mapModeButton")
        self.editor_cloud_mode_button.setCheckable(True)
        self.editor_mode_group = QButtonGroup(self)
        self.editor_mode_group.setExclusive(True)
        self.editor_mode_group.addButton(self.editor_grid_mode_button)
        self.editor_mode_group.addButton(self.editor_cloud_mode_button)
        self.editor_grid_mode_button.clicked.connect(
            lambda: self.set_editor_map_mode("grid"))
        self.editor_cloud_mode_button.clicked.connect(
            lambda: self.set_editor_map_mode("cloud"))
        mode_header.addWidget(self.editor_grid_mode_button)
        mode_header.addWidget(self.editor_cloud_mode_button)
        controls_layout.addLayout(mode_header)
        self.reset_cloud_view_button = QPushButton("重置三维视角")
        self.reset_cloud_view_button.setObjectName("secondaryButton")
        self.reset_cloud_view_button.clicked.connect(
            self.editor_map_panel.reset_cloud_view)
        controls_layout.addWidget(self.reset_cloud_view_button)
        cloud_view_hint = QLabel(
            "点云模式：左键旋转，右键/中键平移，滚轮缩放，双击复位")
        cloud_view_hint.setObjectName("hint")
        cloud_view_hint.setWordWrap(True)
        controls_layout.addWidget(cloud_view_hint)

        display_title = QLabel("点云显示（不修改原始数据）")
        display_title.setObjectName("toolGroupTitle")
        controls_layout.addWidget(display_title)
        axis_row = QHBoxLayout()
        axis_row.addWidget(QLabel("轴顺序"))
        self.cloud_axis_combo = QComboBox()
        for order in ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"):
            self.cloud_axis_combo.addItem(order, order)
        axis_row.addWidget(self.cloud_axis_combo, 1)
        self.cloud_flip_x = QCheckBox("反X")
        self.cloud_flip_y = QCheckBox("反Y")
        self.cloud_flip_z = QCheckBox("反Z")
        axis_row.addWidget(self.cloud_flip_x)
        axis_row.addWidget(self.cloud_flip_y)
        axis_row.addWidget(self.cloud_flip_z)
        controls_layout.addLayout(axis_row)

        render_row = QHBoxLayout()
        render_row.addWidget(QLabel("投影"))
        self.cloud_projection_combo = QComboBox()
        self.cloud_projection_combo.addItem("透视", "perspective")
        self.cloud_projection_combo.addItem("正交", "orthographic")
        render_row.addWidget(self.cloud_projection_combo, 1)
        render_row.addWidget(QLabel("颜色"))
        self.cloud_color_combo = QComboBox()
        self.cloud_color_combo.addItem("高度伪彩色", "height")
        self.cloud_color_combo.addItem("强度伪彩色", "intensity")
        self.cloud_color_combo.addItem("原始 RGB / 灰度强度", "rgb")
        self.cloud_color_combo.model().item(1).setEnabled(False)
        render_row.addWidget(self.cloud_color_combo, 1)
        controls_layout.addLayout(render_row)

        self.cloud_axis_combo.currentIndexChanged.connect(
            self.update_cloud_display_options)
        self.cloud_projection_combo.currentIndexChanged.connect(
            self.update_cloud_display_options)
        self.cloud_color_combo.currentIndexChanged.connect(
            self.update_cloud_display_options)
        self.cloud_flip_x.toggled.connect(self.update_cloud_display_options)
        self.cloud_flip_y.toggled.connect(self.update_cloud_display_options)
        self.cloud_flip_z.toggled.connect(self.update_cloud_display_options)

        source_title = QLabel("文件")
        source_title.setObjectName("toolGroupTitle")
        controls_layout.addWidget(source_title)
        file_buttons = QHBoxLayout()
        self.load_grid_button = QPushButton("加载 YAML/PGM")
        self.load_grid_button.setObjectName("secondaryButton")
        self.load_grid_button.clicked.connect(self.load_grid_dialog)
        self.load_cloud_button = QPushButton("加载 PCD/PLY")
        self.load_cloud_button.setObjectName("secondaryButton")
        self.load_cloud_button.clicked.connect(self.load_pointcloud_dialog)
        file_buttons.addWidget(self.load_grid_button)
        file_buttons.addWidget(self.load_cloud_button)
        controls_layout.addLayout(file_buttons)
        self.save_map_button = QPushButton("保存当前地图")
        self.save_map_button.setObjectName("primaryButton")
        self.save_map_button.clicked.connect(self.save_current_map)
        controls_layout.addWidget(self.save_map_button)

        convert_title = QLabel("点云转栅格")
        convert_title.setObjectName("toolGroupTitle")
        controls_layout.addWidget(convert_title)
        conversion_hint = QLabel("以下参数在重新转换点云时生效，不代表已加载栅格的生成参数。")
        conversion_hint.setObjectName("hint")
        conversion_hint.setWordWrap(True)
        controls_layout.addWidget(conversion_hint)
        convert_frame_row = QHBoxLayout()
        convert_frame_row.addWidget(QLabel("转换坐标"))
        self.convert_coordinates_combo = QComboBox()
        self.convert_coordinates_combo.addItem(
            "当前显示坐标（手动轴变换）", "display")
        self.convert_coordinates_combo.addItem("原始 XYZ", "original")
        convert_frame_row.addWidget(self.convert_coordinates_combo, 1)
        controls_layout.addLayout(convert_frame_row)
        resolution_row = QHBoxLayout()
        resolution_row.addWidget(QLabel("栅格分辨率"))
        self.grid_resolution_spin = QDoubleSpinBox()
        self.grid_resolution_spin.setRange(0.01, 1.0)
        self.grid_resolution_spin.setSingleStep(0.01)
        self.grid_resolution_spin.setDecimals(2)
        self.grid_resolution_spin.setValue(0.05)
        self.grid_resolution_spin.setSuffix(" m/格")
        resolution_row.addWidget(self.grid_resolution_spin)
        controls_layout.addLayout(resolution_row)
        inflation_row = QHBoxLayout()
        inflation_row.addWidget(QLabel("障碍外扩"))
        self.grid_inflation_spin = QDoubleSpinBox()
        self.grid_inflation_spin.setRange(0.0, 0.5)
        self.grid_inflation_spin.setDecimals(2)
        self.grid_inflation_spin.setSingleStep(0.01)
        self.grid_inflation_spin.setValue(0.0)
        self.grid_inflation_spin.setSuffix(" m")
        self.grid_inflation_spin.setToolTip("0 保留原始障碍轮廓；导航时的安全膨胀由代价地图另行配置")
        inflation_row.addWidget(self.grid_inflation_spin)
        controls_layout.addLayout(inflation_row)
        self.grid_settings_label = QLabel()
        self.grid_settings_label.setObjectName("hint")
        self.grid_settings_label.setWordWrap(True)
        controls_layout.addWidget(self.grid_settings_label)
        self.grid_resolution_spin.valueChanged.connect(self.update_grid_settings_label)
        self.grid_inflation_spin.valueChanged.connect(self.update_grid_settings_label)
        self.update_grid_settings_label()
        height_row = QHBoxLayout()
        height_row.addWidget(QLabel("障碍高度 Z"))
        self.z_min_spin = QDoubleSpinBox()
        self.z_min_spin.setRange(-1000.0, 1000.0)
        self.z_min_spin.setValue(0.20)
        self.z_min_spin.setDecimals(2)
        self.z_max_spin = QDoubleSpinBox()
        self.z_max_spin.setRange(-1000.0, 1000.0)
        self.z_max_spin.setValue(1.50)
        self.z_max_spin.setDecimals(2)
        height_row.addWidget(self.z_min_spin)
        height_row.addWidget(QLabel("至"))
        height_row.addWidget(self.z_max_spin)
        controls_layout.addLayout(height_row)
        self.height_preview_check = QCheckBox("预览高度截取（区间外变灰）")
        self.height_preview_check.setToolTip("只影响显示，不修改原始点云")
        self.height_only_check = QCheckBox("仅显示区间内点云")
        self.height_only_check.setEnabled(False)
        controls_layout.addWidget(self.height_preview_check)
        controls_layout.addWidget(self.height_only_check)
        plane_row = QHBoxLayout()
        for text, target in (("点选下平面", "lower"), ("点选上平面", "upper")):
            button = QPushButton(text)
            button.setObjectName("secondaryButton")
            button.clicked.connect(lambda _checked=False, target=target: self.pick_height_plane(target))
            plane_row.addWidget(button)
        controls_layout.addLayout(plane_row)
        self.height_preview_status = QLabel("加载 PCD 后可预览高度区间；蓝色下界、橙色上界")
        self.height_preview_status.setWordWrap(True)
        self.height_preview_status.setObjectName("hint")
        controls_layout.addWidget(self.height_preview_status)
        self._height_pick_target = None
        self.editor_map_panel.height_selected.connect(self.height_plane_selected)
        self.z_min_spin.valueChanged.connect(self.height_limits_changed)
        self.z_max_spin.valueChanged.connect(self.height_limits_changed)
        self.height_preview_check.toggled.connect(self.update_height_preview)
        self.height_preview_check.toggled.connect(self.height_only_check.setEnabled)
        self.height_only_check.toggled.connect(self.update_height_preview)
        self.convert_coordinates_combo.currentIndexChanged.connect(self.update_height_preview)
        floor_row = QHBoxLayout()
        floor_row.addWidget(QLabel("地面高度 Z"))
        self.floor_z_spin = QDoubleSpinBox()
        self.floor_z_spin.setRange(-1000.0, 1000.0)
        self.floor_z_spin.setDecimals(2)
        self.floor_z_spin.setSuffix(" m")
        floor_row.addWidget(self.floor_z_spin, 1)
        floor_pick = QPushButton("点选地面")
        floor_pick.clicked.connect(lambda: self.pick_height_plane("ground"))
        floor_row.addWidget(floor_pick)
        controls_layout.addLayout(floor_row)
        tolerance_row = QHBoxLayout()
        tolerance_row.addWidget(QLabel("地面厚度 ±"))
        self.floor_tolerance_spin = QDoubleSpinBox()
        self.floor_tolerance_spin.setRange(.01, .50)
        self.floor_tolerance_spin.setDecimals(2)
        self.floor_tolerance_spin.setSingleStep(.01)
        self.floor_tolerance_spin.setValue(.06)
        self.floor_tolerance_spin.setSuffix(" m")
        tolerance_row.addWidget(self.floor_tolerance_spin, 1)
        controls_layout.addLayout(tolerance_row)
        floor_hint = QLabel("黑＝障碍，白＝实测地面，灰＝未知。地面高度与障碍高度使用上方转换坐标；地面层须低于障碍下界。")
        floor_hint.setObjectName("hint")
        floor_hint.setWordWrap(True)
        controls_layout.addWidget(floor_hint)
        self.convert_button = QPushButton("转换栅格 · 原有投影")
        self.convert_button.setObjectName("primaryButton")
        self.convert_button.clicked.connect(self.convert_pointcloud)
        controls_layout.addWidget(self.convert_button)
        self.terrain_convert_button = QPushButton("转换栅格 · 局部高程通行性")
        self.terrain_convert_button.setObjectName("primaryButton")
        self.terrain_convert_button.clicked.connect(self.convert_terrain_grid)
        controls_layout.addWidget(self.terrain_convert_button)
        terrain_hint = QLabel("原有投影使用上方高度参数；局部高程分析完整编辑后点云，自动确定地面参考，保留当前转换坐标。两种方法共用栅格分辨率。")
        terrain_hint.setWordWrap(True)
        terrain_hint.setObjectName("hint")
        controls_layout.addWidget(terrain_hint)

        edit_title = QLabel("地图编辑")
        edit_title.setObjectName("toolGroupTitle")
        controls_layout.addWidget(edit_title)
        self.edit_tool_combo = QComboBox()
        self.edit_tool_combo.addItem("浏览（不修改）", "navigate")
        self.edit_tool_combo.addItem("平移视图（不修改）", "pan")
        self.edit_tool_combo.addItem("栅格：绘制障碍物", "occupied")
        self.edit_tool_combo.addItem("栅格：绘制自由区域", "free")
        self.edit_tool_combo.addItem("栅格：绘制未知区域", "unknown")
        self.edit_tool_combo.addItem("点云：添加障碍点", "cloud_add")
        self.edit_tool_combo.addItem("点云：擦除点", "cloud_erase")
        self.edit_tool_combo.currentIndexChanged.connect(self.update_edit_tool)
        controls_layout.addWidget(self.edit_tool_combo)
        brush_row = QHBoxLayout()
        brush_row.addWidget(QLabel("画笔半径"))
        self.brush_radius_spin = QDoubleSpinBox()
        self.brush_radius_spin.setRange(0.02, 5.0)
        self.brush_radius_spin.setSingleStep(0.05)
        self.brush_radius_spin.setValue(0.20)
        self.brush_radius_spin.setSuffix(" m")
        self.brush_radius_spin.valueChanged.connect(self.update_edit_tool)
        brush_row.addWidget(self.brush_radius_spin)
        controls_layout.addLayout(brush_row)
        edit_buttons = QHBoxLayout()
        self.undo_button = QPushButton("撤销上一笔")
        self.undo_button.setObjectName("secondaryButton")
        self.undo_button.clicked.connect(self.undo_map_edit)
        self.reset_map_button = QPushButton("恢复加载版本")
        self.reset_map_button.setObjectName("secondaryButton")
        self.reset_map_button.clicked.connect(self.reset_map_edits)
        edit_buttons.addWidget(self.undo_button)
        edit_buttons.addWidget(self.reset_map_button)
        controls_layout.addLayout(edit_buttons)

        self.map_status = QLabel(
            "地图处理与导航任务相互独立。编辑保存后返回导航使用。")
        self.map_status.setObjectName("mapStatus")
        self.map_status.setWordWrap(True)
        controls_layout.insertWidget(controls_layout.indexOf(self.convert_button), self.map_status)
        controls_layout.addStretch(1)
        self.update_cloud_display_options(show_status=False)

        from .ui.touch_scroll import enable_touch_scroll
        controls_scroll = QScrollArea()
        controls_scroll.setObjectName("transparentScroll")
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setWidget(controls)
        enable_touch_scroll(controls_scroll)
        splitter.addWidget(controls_scroll)
        splitter.setSizes([1050, 400])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        layout.addWidget(splitter, 1)
        return root

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == QEvent.WindowStateChange:
            self.fullscreen_button.setText("退出全屏" if self.isFullScreen() else "全屏")
            self.leave_fullscreen_shortcut.setEnabled(self.isFullScreen())

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if hasattr(self, "setup_splitter"):
            self._set_compact_mode(min(self.width(), self.height()) < 700)

    def _set_compact_mode(self, compact: bool) -> None:
        """Switch every workspace between desktop and portrait layouts."""
        compact = bool(compact)
        if not compact:
            profile = "desktop"
        elif self.width() < self.height():
            profile = "compact_portrait"
        else:
            profile = "compact_landscape"
        if self._layout_profile == profile:
            return
        self._compact_mode = compact
        self._layout_profile = profile
        portrait = profile == "compact_portrait"
        landscape = profile == "compact_landscape"

        if landscape:
            self.main_title_label.setText("NAV")
            self.open_navigation_stack_button.setText("导航")
            self.open_sensor_tools_button.setText("传感器")
            self.open_mapping_button.setText("开始建图")
            self.open_map_tools_button.setText("地图")
            self.ros_status.setText("ROS")
            self.exit_app_button.setText("退出程序")
        else:
            self.main_title_label.setText("NAV")
            self.open_navigation_stack_button.setText("导航系统")
            self.open_sensor_tools_button.setText("传感器管理")
            self.open_mapping_button.setText("开始建图")
            self.open_map_tools_button.setText("地图处理")
            self.ros_status.setText("ROS 2 ONLINE")
            self.exit_app_button.setText("退出程序")
        self.main_title_label.setVisible(
            not (landscape and self.width() < 650))
        self.fullscreen_button.setVisible(not compact)

        margins = (8, 6, 8, 8) if compact else (22, 18, 22, 22)
        spacing = 7 if compact else 14
        for page in (
            self.setup_page,
            self.map_tools_page,
            self.sensor_tools_page,
            self.sensor_monitor_page,
        ):
            page_layout = page.layout()
            if page_layout is not None:
                page_layout.setContentsMargins(*margins)
                page_layout.setSpacing(spacing)

        for label in self.findChildren(QLabel):
            if label.objectName() == "subtitle":
                label.setVisible(not compact)

        header_direction = (
            QBoxLayout.LeftToRight if landscape else QBoxLayout.TopToBottom)
        self.setup_header.setDirection(header_direction)
        for header in (self.sensor_tools_header, self.sensor_monitor_header):
            header.setDirection(QBoxLayout.LeftToRight)

        orientation = Qt.Vertical if portrait else Qt.Horizontal
        self.setup_splitter.setOrientation(orientation)
        self.sensor_tools_splitter.setOrientation(orientation)
        self.map_tools_splitter.setOrientation(orientation)

        self.setup_side_panel.setMinimumWidth(0)
        self.sensor_tools_controls.setMinimumWidth(0 if compact else 440)
        self.map_tools_controls.setMinimumWidth(0 if compact else 360)
        self.map_tools_controls.setMaximumWidth(16777215 if compact else 440)

        self.map_panel.setMinimumSize(180 if compact else 260, 170 if compact else 200)
        self.editor_map_panel.setMinimumSize(
            180 if compact else 260, 170 if compact else 200)
        camera_min_height = 64 if landscape else (96 if compact else 160)
        camera_max_height = 86 if landscape else (150 if compact else 260)
        self.camera_panel.setMinimumSize(
            160 if landscape else (180 if compact else 280),
            camera_min_height,
        )
        self.camera_panel.setMinimumHeight(camera_min_height)
        self.camera_panel.setMaximumHeight(camera_max_height)
        self.waypoint_list.setMinimumHeight(
            42 if landscape else (68 if compact else 130))
        self.active_page.camera_panel.setMinimumSize(
            120 if compact else 640, 76 if compact else 420)
        self.active_page.map_panel.setMinimumSize(
            180 if compact else 260, 170 if compact else 200)

        if portrait:
            self.setup_splitter.setSizes([360, 300])
            self.sensor_tools_splitter.setSizes([390, 260])
            self.map_tools_splitter.setSizes([350, 310])
        elif landscape:
            self.setup_splitter.setSizes([480, 310])
            self.sensor_tools_splitter.setSizes([450, 340])
            self.map_tools_splitter.setSizes([430, 360])
        else:
            self.setup_splitter.setSizes([1080, 380])
            self.sensor_tools_splitter.setSizes([500, 900])
            self.map_tools_splitter.setSizes([1050, 400])

        for page in self.task_workspaces:
            page.set_compact_mode(compact)
        self.system_home_page.set_compact_mode(compact)
        self.system_settings_page.set_compact_mode(compact, portrait=portrait)
        self.manual_drive_page.set_compact_mode(compact, portrait=portrait)
        self.operations_page.bags.set_compact_mode(compact, portrait=portrait)
        self.mapping_page.set_compact_mode(compact, portrait=portrait)
        self.mapping_workspace.set_compact_mode(compact, portrait=portrait)
        self.navigation_stack_page.set_compact_mode(
            compact, portrait=portrait)
        side_margins = (7, 6, 7, 6) if landscape else (
            (10, 9, 10, 9) if compact else (14, 14, 14, 14))
        self.setup_side_layout.setContentsMargins(*side_margins)
        self.setup_side_layout.setSpacing(4 if landscape else (7 if compact else 10))
        self.setup_map_layout.setContentsMargins(
            *(8, 7, 8, 7) if compact else (16, 16, 16, 16))
        self.setup_map_layout.setSpacing(5 if compact else 10)
        self.setStyleSheet(self._style_sheet(compact))
        self.updateGeometry()

    @staticmethod
    def _style_sheet(compact: bool = False) -> str:
        base = """
        QMainWindow, QWidget { background: #0b1016; color: #eaf0f4; }
        QLabel#title { font-size: 25px; font-weight: 700; letter-spacing: 2px; }
        QLabel#subtitle { color: #728493; font-size: 12px; }
        QLabel#onlineChip {
            color: #66e09a; background: #10291d; border: 1px solid #245d3c;
            border-radius: 12px; padding: 7px 14px; font-weight: 700;
        }
        QWidget#card, QFrame#sidePanel {
            background: #101720; border: 1px solid #26323d; border-radius: 14px;
        }
        QWidget#mapPanel, QWidget#mapEditorPanel,
        QWidget#sensorPreviewCloud, QWidget#sensorPreviewCamera {
            border: 1px solid #293946; border-radius: 9px;
        }
        QWidget#mapEditorPanel { background: #0d141b; }
        QWidget#activeMap {
            background: #111820; border: 2px solid rgba(255, 255, 255, 75);
            border-radius: 12px;
        }
        QLabel#sectionTitle { font-size: 18px; font-weight: 650; }
        QLabel#toolGroupTitle {
            color: #7fcce6; font-size: 12px; font-weight: 700;
            padding-top: 5px;
        }
        QLabel#hint { color: #8ca0ae; font-size: 12px; }
        QLabel#coordinate {
            color: #d6e0e6; background: #151f29; border-radius: 7px; padding: 8px;
        }
        QLabel#statusBar {
            color: #8ed9f4; background: #0d1d27; border-radius: 7px; padding: 10px;
        }
        QLabel#mapStatus {
            color: #9fb2bf; background: #111b24; border-radius: 6px; padding: 6px;
            font-size: 11px;
        }
        QLabel#sensorPreviewStatus {
            color: #8ed9f4; background: #0d1d27; border-radius: 7px;
            padding: 8px; font-size: 12px;
        }
        QPushButton, QPushButton#primaryButton, QPushButton#secondaryButton, QPushButton#workspaceButton, QPushButton#backButton, QPushButton#dangerButton, QPushButton#exitNavigationButton {
            min-height: 38px; border-radius: 7px; padding: 0 15px;
            font-size: 13px; font-weight: 650;
            background: #18232d; border: 1px solid #354653; color: #dbe4e9;
        }
        QPushButton:hover, QPushButton#secondaryButton:hover, QPushButton#backButton:hover { background: #233441; border-color: #527084; }
        QPushButton:pressed, QPushButton#secondaryButton:pressed, QPushButton#backButton:pressed { background: #101d27; border-color: #76b4ce; }
        QPushButton:disabled, QPushButton#secondaryButton:disabled, QPushButton#backButton:disabled, QPushButton#workspaceButton:disabled { background: #141d25; border-color: #293640; color: #71838c; }
        QPushButton#primaryButton { background: #00a9df; color: #041018; border: 1px solid #00a9df; }
        QPushButton#primaryButton:hover { background: #25c2f2; }
        QPushButton#primaryButton:disabled { background: #28404b; border-color: #354653; color: #71838c; }
        QPushButton#primaryButton:pressed { background: #0084b1; border-color: #0084b1; }
        QPushButton#secondaryButton, QPushButton#backButton {
            background: #18232d; border: 1px solid #354653; color: #dbe4e9;
        }
        QPushButton#workspaceButton {
            background: #17232d; border: 1px solid #3b5666; color: #bcecff;
        }
        QPushButton#workspaceButton:hover { background: #203441; }
        QPushButton#workspaceButton:pressed { background: #102632; border-color: #76b4ce; }
        QPushButton#mapModeButton {
            min-height: 24px; max-height: 28px; min-width: 48px; padding: 0 8px;
            background: rgba(12, 20, 27, 220); border: 1px solid #40525e;
            color: #aebdc6; font-size: 11px;
        }
        QPushButton#mapModeButton:checked {
            background: #00a9df; border-color: #32c5ef; color: #031016;
        }
        QComboBox, QDoubleSpinBox, QLineEdit {
            min-height: 30px; background: #17222c; border: 1px solid #344652;
            border-radius: 5px; padding: 0 6px; color: #dbe4e9;
        }
        QGroupBox {
            border: 1px solid #2c3b46; border-radius: 8px;
            margin-top: 9px; padding-top: 8px; font-weight: 650;
        }
        QTabWidget::pane {
            border: 1px solid #2c3b46; border-radius: 7px; background: #0e161e;
        }
        QTabBar::tab {
            min-width: 82px; min-height: 30px; padding: 2px 12px;
            background: #141f28; color: #9eb0bb;
            border: 1px solid #2c3b46;
        }
        QTabBar::tab:selected { background: #174a60; color: #ffffff; }
        QPlainTextEdit#sensorLog {
            background: #080d12; color: #b9d9c5; border: none;
            font-family: Monospace; font-size: 12px; padding: 8px;
            selection-background-color: #245b70;
        }
        QPlainTextEdit#imuData {
            background: #080d12; color: #d4e6ec;
            border: 1px solid #293946; border-radius: 9px;
            font-family: Monospace; font-size: 15px; padding: 18px;
            selection-background-color: #245b70;
        }
        QListWidget#waypointList {
            background: #111b24; border: 1px solid #2d3e4a;
            border-radius: 7px; padding: 5px; color: #dbe4e9;
            outline: none;
        }
        QListWidget#waypointList::item {
            min-height: 34px; border-radius: 5px; padding: 2px 8px;
        }
        QListWidget#waypointList::item:selected {
            background: #174a60; color: #ffffff;
        }
        QCheckBox { color: #dbe4e9; spacing: 4px; }
        QCheckBox::indicator {
            width: 15px; height: 15px; border: 1px solid #48606e;
            border-radius: 3px; background: #111a22;
        }
        QCheckBox::indicator:checked { background: #00a9df; }
        QPushButton#dangerButton, QPushButton#exitNavigationButton {
            background: #24151d; border: 1px solid #6d3540; color: #ff8d96;
        }
        QPushButton#dangerButton:hover, QPushButton#exitNavigationButton:hover {
            background: #5b252d; border-color: #ff8d96; color: white;
        }
        QPushButton#dangerButton:disabled, QPushButton#exitNavigationButton:disabled {
            background: #19161c; border-color: #3d3038; color: #78636a;
        }
        QSplitter::handle { background: transparent; width: 12px; }
        """
        if not compact:
            return base
        return base + """
        QLabel#title { font-size: 18px; letter-spacing: 1px; }
        QLabel#sectionTitle { font-size: 14px; }
        QLabel#hint { font-size: 10px; }
        QLabel#onlineChip {
            border-radius: 8px; padding: 4px 7px; font-size: 9px;
        }
        QPushButton, QPushButton#primaryButton, QPushButton#secondaryButton, QPushButton#workspaceButton, QPushButton#backButton, QPushButton#dangerButton, QPushButton#exitNavigationButton {
            min-height: 34px; border-radius: 6px; padding: 0 7px;
            font-size: 11px;
        }
        QPushButton#mapModeButton {
            min-height: 26px; max-height: 30px; min-width: 42px;
            padding: 0 5px; font-size: 10px;
        }
        QComboBox, QDoubleSpinBox, QLineEdit {
            min-height: 28px; font-size: 11px;
        }
        QTabBar::tab {
            min-width: 48px; min-height: 28px; padding: 1px 5px;
            font-size: 10px;
        }
        QPlainTextEdit#sensorLog { font-size: 10px; padding: 4px; }
        QPlainTextEdit#imuData { font-size: 11px; padding: 8px; }
        QListWidget#waypointList::item { min-height: 29px; padding: 1px 5px; }
        QScrollArea#transparentScroll { border: none; background: transparent; }
        QSplitter::handle { width: 6px; height: 6px; }
        """

    def _load_configured_pointcloud(self) -> None:
        if not hasattr(self.node, "get_parameter"):
            return
        configured = str(self.node.get_parameter("pointcloud_map_path").value)
        if not configured:
            return
        path = FilePath(configured).expanduser()
        if not path.exists():
            self.map_status.setText(f"配置的点云文件不存在：{path}")
            return
        try:
            self._set_local_cloud(load_pointcloud(path), reset_original=True)
            self.map_status.setText(
                f"已自动加载点云：{path.name} · {len(self.local_cloud.points):,} 点 · "
                f"{self.editor_map_panel.cloud_display_description()}")
        except (OSError, ValueError) as exc:
            self.map_status.setText(f"自动加载点云失败：{exc}")

    def start_mapping_from_home(self) -> None:
        self._mapping_start_error = ""
        if self.mapping_controller.process.active:
            self.show_mapping()
            return
        if self._navigation_service_active():
            self._mapping_start_error = "无法开始建图：导航后台服务尚未停止，请先点击“停止导航”。"
            self.show_system_home()
            return
        self.mapping_page.activate()
        success, status = self.mapping_page.start_mapping()
        if success:
            self.show_mapping()
        else:
            self.operations_page._release_task_owner("mapping", "stopped")
            self._mapping_start_error = "建图未启动：" + status
            if "后台地图任务" in status:
                self._mapping_start_error += "；请点击“查看/暂停”，暂停占用任务后再开始。"
            self.show_system_home()

    def _navigation_service_active(self):
        return (self.navigation_stack_controller.overall_state in ("starting", "running", "stopping")
                or any(process.active for process in self.navigation_stack_controller.processes.values()))

    def show_home_map_tasks(self):
        self.show_map_progress()
        snapshot = self.operations_page.task_coordinator.snapshot()
        page = self.operations_page.map_tasks
        page.refresh(snapshot["jobs"], snapshot.get("error", ""))
        for index in range(page.rows.count()):
            job = page.jobs[page.rows.item(index).data(Qt.UserRole)]
            if job["state"] in ("running", "cancelling"):
                page.rows.setCurrentRow(index)
                break

    def stop_home_mapping(self):
        self.operations_page.confirm(
            "停止建图服务？尚未保存的地图不会自动写入文件；需要保存时，请先进入建图页选择“保存并结束”。",
            self.mapping_controller.stop, "停止建图服务")

    def stop_home_navigation(self):
        self.operations_page.confirm("停止导航后台服务？当前定位和路径规划将结束。",
                                     self.navigation_stack_controller.stop, "停止导航服务")

    def show_mapping(self) -> None:
        """Open daily mapping without restarting the shared mapper."""
        self.pages.setCurrentWidget(self.mapping_workspace)

    def show_mapping_settings(self) -> None:
        self.mapping_page.activate()
        self.pages.setCurrentWidget(self.mapping_page)

    def _navigation_stack_status_changed(self, state):
        if state in ("stopped", "error"):
            self._clear_navigation_overlays()
        if state == "stopped":
            self._mapping_start_error = ""
        if state == "running":
            self.node.navigation_status = "导航系统已就绪，请添加途径点"
            self.nav_status.setText(self.node.navigation_status)

    def show_navigation_access(self) -> None:
        self.pages.setCurrentWidget(self.navigation_access_page)

    def show_navigation_stack(self) -> None:
        """Open the complete navigation runtime launcher."""
        self.navigation_stack_page.activate()
        self.pages.setCurrentWidget(self.navigation_stack_page)

    def show_map_tools(self) -> None:
        """Open the standalone map processing workspace."""
        self.map_pages.setCurrentWidget(self.map_tools_page)
        self.pages.setCurrentWidget(self.map_workspace_page)

    def show_sensor_tools(self) -> None:
        """Open the standalone sensor driver workspace."""
        self._sync_sensor_driver_history()
        self.pages.setCurrentWidget(self.sensor_tools_page)

    def _sync_sensor_driver_history(self) -> None:
        for view in self.sensor_log_views.values():
            view.clear()
        for key, text in self.sensor_driver_controller.log_events:
            self._append_sensor_log(key, text)
        for key, state in self.sensor_driver_controller.states.items():
            self._handle_sensor_state(key, state)

    def show_sensor_monitor(self) -> None:
        """Open live sensor views and refresh compatible ROS topics."""
        self.refresh_sensor_topics()
        self.pages.setCurrentWidget(self.sensor_monitor_page)

    def leave_sensor_monitor(self) -> None:
        """Release preview-only subscriptions and return to driver control."""
        self.sensor_preview_adapter.stop()
        for kind in ("lidar", "camera", "imu"):
            self._clear_sensor_preview(kind)
        self.pages.setCurrentWidget(self.sensor_tools_page)

    def refresh_sensor_topics(self) -> None:
        """Populate selectors from the ROS graph while preserving manual text."""
        grouped = {"lidar": [], "camera": [], "imu": []}
        try:
            discovered = self.sensor_preview_adapter.available_topics()
            for kind in grouped:
                grouped[kind] = list(discovered.get(kind, ()))
        except (RuntimeError, TypeError, ValueError) as exc:
            for label in self.sensor_preview_status_labels.values():
                label.setText(f"读取 ROS 2 话题失败：{exc}")
            return
        configured = {
            "camera": str(getattr(self.node, "camera_topic", "")),
            "lidar": str(getattr(self.node, "pointcloud_topic", "")),
            "imu": "",
        }
        active_topics = self.sensor_preview_adapter.topics
        for kind, combo in self.sensor_topic_combos.items():
            current = combo.currentText().strip()
            candidates = list(grouped[kind])
            for candidate in (configured[kind], active_topics.get(kind, ""), current):
                if candidate and candidate not in candidates:
                    candidates.append(candidate)
            candidates.sort()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(candidates)
            preferred = current or configured[kind]
            if preferred:
                combo.setCurrentText(preferred)
            combo.blockSignals(False)
            if candidates:
                self.sensor_preview_status_labels[kind].setText(
                    f"发现 {len(grouped[kind])} 个兼容话题，可选择后开始显示")
            else:
                self.sensor_preview_status_labels[kind].setText(
                    "暂未发现兼容话题，也可以手动输入完整话题名称")

    def _start_sensor_preview(self, kind: str) -> None:
        combo = self.sensor_topic_combos[kind]
        topic = combo.currentText().strip()
        self._clear_sensor_preview(kind, topic)
        success, status = self.sensor_preview_adapter.start(kind, topic)
        self.sensor_preview_status_labels[kind].setText(status)
        revisions = self.sensor_preview_adapter.revisions
        self.last_sensor_preview_revisions[kind] = int(
            revisions.get(kind, 0))
        if success:
            tab_indexes = {"lidar": 0, "camera": 1, "imu": 2}
            self.sensor_monitor_tabs.setCurrentIndex(tab_indexes[kind])

    def _stop_sensor_preview(self, kind: str) -> None:
        self.sensor_preview_adapter.stop(kind)
        self._clear_sensor_preview(kind)
        self.sensor_preview_status_labels[kind].setText("已停止显示")

    def _clear_sensor_preview(
        self, kind: str, selected_topic: str = "",
    ) -> None:
        if kind == "lidar":
            self.sensor_preview_cloud_panel.set_pointcloud(
                np.empty((0, 3), dtype=np.float32))
            self.sensor_preview_cloud_panel.set_display_mode("cloud")
        elif kind == "camera":
            topic = selected_topic or self.sensor_topic_combos[
                "camera"].currentText().strip()
            self.sensor_preview_camera_panel.clear_frame(topic or None)
        else:
            self.sensor_preview_imu_text.setPlainText("等待 IMU 数据…")

    @staticmethod
    def _format_imu_data(data) -> str:
        stamp_sec, stamp_nanosec = data["stamp"]
        orientation = data["orientation"]
        angular = data["angular_velocity"]
        linear = data["linear_acceleration"]

        def vector(values) -> str:
            return "  ".join(f"{value:+.6f}" for value in values)

        def covariance(values) -> str:
            rows = [values[index:index + 3] for index in range(0, 9, 3)]
            return "\n".join(
                "    " + "  ".join(f"{value:+.5e}" for value in row)
                for row in rows)

        return (
            f"Frame ID       {data['frame_id'] or '(empty)'}\n"
            f"Timestamp      {stamp_sec}.{stamp_nanosec:09d}\n\n"
            "Orientation quaternion (x, y, z, w)\n"
            f"  {vector(orientation)}\n\n"
            "Angular velocity (rad/s)\n"
            f"  x / y / z    {vector(angular)}\n\n"
            "Linear acceleration (m/s²)\n"
            f"  x / y / z    {vector(linear)}\n\n"
            "Orientation covariance\n"
            f"{covariance(data['orientation_covariance'])}\n\n"
            "Angular velocity covariance\n"
            f"{covariance(data['angular_velocity_covariance'])}\n\n"
            "Linear acceleration covariance\n"
            f"{covariance(data['linear_acceleration_covariance'])}"
        )

    def _refresh_sensor_previews(self) -> None:
        if self.pages.currentWidget() is not self.sensor_monitor_page:
            return
        revisions = self.sensor_preview_adapter.revisions
        errors = self.sensor_preview_adapter.errors
        topics = self.sensor_preview_adapter.topics
        for kind in ("lidar", "camera", "imu"):
            error = errors.get(kind, "")
            if error:
                self.sensor_preview_status_labels[kind].setText(error)
            revision = int(revisions.get(kind, 0))
            if revision == self.last_sensor_preview_revisions.get(kind, 0):
                continue
            self.last_sensor_preview_revisions[kind] = revision
            topic = topics.get(kind, "")
            if kind == "lidar":
                cloud = self.sensor_preview_adapter.cloud
                if cloud is not None:
                    self.sensor_preview_cloud_panel.set_pointcloud(cloud)
                    self.sensor_preview_cloud_panel.set_display_mode("cloud")
                    self.sensor_preview_status_labels[kind].setText(
                        f"正在显示 {topic} · {len(cloud):,} 点")
            elif kind == "camera":
                image = self.sensor_preview_adapter.image
                if image is not None:
                    self.sensor_preview_camera_panel.set_frame(
                        image, None, 0.0)
                    height, width = image.shape[:2]
                    self.sensor_preview_status_labels[kind].setText(
                        f"正在显示 {topic} · {width}×{height}")
            else:
                data = self.sensor_preview_adapter.imu
                if data is not None:
                    self.sensor_preview_imu_text.setPlainText(
                        self._format_imu_data(data))
                    self.sensor_preview_status_labels[kind].setText(
                        f"正在显示 {topic} · 已接收 {revision:,} 帧")

    def show_system_home(self) -> None:
        """Return to the system entry without stopping a running task."""
        self._refresh_system_home()
        self.pages.setCurrentWidget(self.system_home_page)

    def show_system_settings(self) -> None:
        self.pages.setCurrentWidget(self.system_settings_page)

    def show_operations(self) -> None:
        self.show_system_settings()

    def show_map_library(self) -> None:
        self.map_pages.setCurrentWidget(self.operations_page.library)
        self.pages.setCurrentWidget(self.map_workspace_page)

    def show_map_progress(self) -> None:
        self.map_pages.setCurrentWidget(self.operations_page.map_tasks)
        self.pages.setCurrentWidget(self.map_workspace_page)

    def show_recordings(self) -> None:
        self.pages.setCurrentWidget(self.recordings_workspace_page)

    def show_logs(self) -> None:
        self.operations_page.logs.refresh()
        self.pages.setCurrentWidget(self.logs_workspace_page)

    def show_diagnostics(self) -> None:
        self.operations_page.diagnostics_page.refresh()
        self.pages.setCurrentWidget(self.diagnostics_workspace_page)

    def show_manual_drive(self) -> None:
        self.pages.setCurrentWidget(self.manual_drive_page)

    def show_phone_remote(self) -> None:
        self.pages.setCurrentWidget(self.phone_workspace_page)

    def _refresh_system_home(self) -> None:
        home = self.system_home_page
        mapping = self.mapping_controller.process.active
        navigation = self.navigation_stack_controller.overall_state
        state = "建图中" if mapping else {
            "starting": "导航准备中", "running": "导航服务运行中",
            "stopping": "正在停止导航", "error": "导航需要检查", "stopped": "待命",
        }[navigation]
        profile = "WOOSH" if self.mapping_controller.backend.key.startswith("woosh_") else "机器人"
        home.state.setText(f"{profile} · {state}")
        mapping_state = self.mapping_page.process_state
        labels = dict(starting="启动中", running="运行中", stopping="正在停止", stopped="未启动", error="异常")
        home.mapping_state.setText("建图服务：" + labels[mapping_state])
        saving = self.mapping_controller.ros.save_in_progress or self.mapping_page._finishing_attempt is not None
        home.mapping_stop_button.setEnabled(mapping_state in ("starting", "running") and not saving)
        if saving:
            home.mapping_state.setText("建图服务：正在保存，请等待完成")
        nav_active = self._navigation_service_active()
        nav_label = "残留组件运行中" if nav_active and navigation in ("error", "stopped") else labels[navigation]
        home.navigation_state.setText("导航服务：" + nav_label)
        home.navigation_stop_button.setEnabled(nav_active and navigation != "stopping")
        snapshot = self.operations_page.task_coordinator.snapshot()
        active_jobs = [job for job in snapshot["jobs"] if job["state"] in ("running", "cancelling")]
        waiting = sum(job["state"] in ("queued", "waiting_foreground") for job in snapshot["jobs"])
        task_text = ("异常，请查看" if snapshot.get("error") else
                     "正在暂停" if any(job["state"] == "cancelling" for job in active_jobs) else
                     "正在处理" if active_jobs else "等待处理" if waiting else
                     "空闲" if snapshot.get("loaded") else "正在读取状态")
        home.task_state.setText(f"地图后台：{task_text}" + (f" · 排队 {waiting}" if waiting else ""))
        home.task_state.setToolTip(snapshot.get("error", ""))
        if active_jobs and not mapping and not nav_active:
            home.state.setText(f"{profile} · 后台地图处理中")
        home.mapping_button.setText("继续建图" if mapping else "开始建图")
        selected = self.local_grid if self.local_grid is not None else self.local_cloud
        name = (selected.path.name if selected is not None and selected.path is not None else
                "当前地图（尚未保存）" if selected is not None else "尚未选择地图")
        home.map_name.setText(name)
        normal_status = ("正在采集地图，完成后请保存并结束" if mapping else
                         "导航服务占用中，建图前请先停止导航" if nav_active else
                         "地图已选择，可进入导航准备" if selected is not None else
                         "准备设备后，采集新地图或选择已有地图")
        home.status.setText(self._mapping_start_error or normal_status)
        blocked = bool(self._mapping_start_error)
        if home.status.property("blocked") != blocked:
            home.status.setProperty("blocked", blocked)
            home.status.style().unpolish(home.status)
            home.status.style().polish(home.status)

    def enter_navigation(self) -> None:
        self.show_navigation_setup()
        if self.navigation_stack_controller.overall_state in ("starting", "running"):
            return
        page = self.navigation_stack_page
        if (not page.navigation_map.text().strip() or
                page.localization_backend.currentData() == "pointcloud_localizer" and not page.localization_map.text().strip()):
            self.node.navigation_status = "请先在地图管理中选择完整地图包，再进入导航"
            self.nav_status.setText(self.node.navigation_status)
            return
        try:
            grid = load_grid_map(page.navigation_map.text())
            self._set_local_grid(grid, reset_original=True)
            self.node.grid_frame = grid.frame_id
        except (OSError, ValueError) as exc:
            self.node.navigation_status = f"导航栅格读取失败：{exc}"
            self.nav_status.setText(self.node.navigation_status)
            return
        success, status = page.start_stack()
        if not success and self.navigation_stack_controller.overall_state == "stopped":
            self.operations_page._release_task_owner("navigation", "stopped")
        self.node.navigation_status = status
        self.nav_status.setText(status)

    def show_navigation_setup(self) -> None:
        """Return to AR navigation without starting or cancelling a task."""
        self.pages.setCurrentWidget(self.setup_page)

    def navigation_localization_backend(self):
        config = self.navigation_stack_controller.active_config
        if config is not None and self.navigation_stack_controller.overall_state in ("starting", "running", "stopping"):
            return config.localization_backend
        return self.navigation_stack_page.localization_backend.currentData()

    def _relocalization_map_identity(self) -> tuple[str, str]:
        if self.navigation_localization_backend() in ("amcl", "external"):
            return self.node.grid_frame, str(self.local_grid.path) if self.local_grid is not None else ""
        row = getattr(
            getattr(self.node.localization_tools, "health", None),
            "sample", {},
        )
        return str(row.get("map_frame", "")), str(row.get("map_sha256", ""))

    def show_relocalization(self) -> None:
        """Open two-point relocalization for the map loaded by Localizer."""
        if self.local_grid is None:
            self.relocalization_page.clear_scene()
            self.relocalization_page.status.setText(
                "请先在导航主页加载与定位服务一致的栅格地图")
        else:
            frame_id, map_sha256 = self._relocalization_map_identity()
            scene_key = (frame_id, map_sha256, id(self.local_grid))
            self.relocalization_page.bind_scene(
                scene_key, "当前导航地图", self.local_grid)
            if self.local_cloud is not None:
                self.relocalization_page.set_pointcloud(self.local_cloud)
        self.pages.setCurrentWidget(self.relocalization_page)
        self._refresh_relocalization()

    def submit_manual_relocalization(self) -> None:
        page = self.relocalization_page
        if not page.has_complete_selection:
            page.set_operation_error("请先依次选择当前位置和设备正前方")
            return
        frame_id, map_sha256 = self._relocalization_map_identity()
        if self.navigation_localization_backend() == "amcl":
            accepted, message = self.node.publish_amcl_initial_pose(
                page.position_point, page.candidate_yaw, frame_id)
            if not accepted: page.set_operation_error(message)
            self._refresh_relocalization()
            return
        accepted, message = self.localization_tools_controller.submit_initial_pose(
            frame_id,
            page.position_point,
            page.direction_point,
            map_sha256,
            page.minimum_direction_distance,
            page.timeout_seconds,
        )
        if not accepted:
            page.set_operation_error(message)
        self._refresh_relocalization()

    def trigger_automatic_relocalization(self) -> None:
        accepted, message = (
            self.localization_tools_controller.trigger_global_relocalization(
                timeout_s=self.relocalization_page.timeout_seconds))
        if not accepted:
            self.relocalization_page.set_operation_error(message)
        self._refresh_relocalization()

    def relocalization_timeout_changed(self, timeout_s: int) -> None:
        if self.navigation_localization_backend() != "pointcloud_localizer":
            return
        accepted, message = (
            self.localization_tools_controller.request_relocalization_timeout(
                timeout_s))
        if not accepted:
            self.relocalization_page.set_operation_error(message)
        self._refresh_relocalization()

    def show_relocalization_diagnostics(self) -> None:
        self.show_diagnostics()

    def _relocalization_operation_changed(
        self, operation: str, state: str,
    ) -> None:
        if operation == "manual_relocalize":
            if state == "success":
                self.relocalization_page.set_operation_matched()
            elif state == "error":
                self.relocalization_page.set_operation_error(
                    self.localization_tools_controller.status)
        elif operation == "global_relocalize" and state == "error":
            self.relocalization_page.set_operation_error(
                self.localization_tools_controller.status)
        self._refresh_relocalization()

    def _refresh_relocalization(self) -> None:
        if self.pages.currentWidget() is not self.relocalization_page:
            return
        controller = self.localization_tools_controller
        page = self.relocalization_page
        backend = self.navigation_localization_backend()
        if backend in ("amcl", "external"):
            _path, robot_xy, robot_yaw = self.node.map_navigation_state()
            page.set_current_pose(robot_xy, robot_yaw)
            ready = backend == "amcl" and self.node.amcl_initial_pose_ready()
            page.set_runtime(submit_ready=ready and self.local_grid is not None,
                readiness_reason="使用两点设置 AMCL 初始位姿" if ready else "等待已有定位或 AMCL 接收端",
                global_pending=False, manual_pending=False, automatic_ready=False,
                current_pose_available=robot_xy is not None, manual_state="idle",
                navigation_ready=self.node.navigation_route_ready())
            if robot_xy is None and backend == "amcl" and self.node._amcl_initial_pose_time:
                page.status.setText("初始位姿已发送，等待 AMCL 确认 map→odom")
            return
        frame_id, map_sha256 = self._relocalization_map_identity()
        submit_ready, reason = controller.initial_pose_readiness(
            frame_id, map_sha256)
        timeout_ready, timeout_reason = controller.relocalization_timeout_readiness(
            page.timeout_seconds)
        if not timeout_ready and submit_ready:
            submit_ready, reason = False, timeout_reason
        _path, robot_xy, robot_yaw = self.node.map_navigation_state()
        page.set_current_pose(robot_xy, robot_yaw)
        stack_active = self.navigation_stack_controller.overall_state in ("starting", "running")
        matching_ready = stack_active and self.local_grid is not None and submit_ready
        page.set_runtime(
            submit_ready=matching_ready,
            readiness_reason=reason,
            global_pending=controller.global_relocalize_pending,
            manual_pending=controller.manual_relocalize_pending,
            automatic_ready=matching_ready,
            current_pose_available=robot_xy is not None,
            manual_state=controller.manual_relocalize_state,
            progress_text=controller.relocalization_progress(),
            progress_metrics=controller.relocalization_metrics(),
            elapsed_s=controller.active_relocalization_elapsed_s,
            timeout_s=controller.active_relocalization_timeout_s,
            navigation_ready=self.node.navigation_route_ready(),
        )

    def _navigation_sensor_active(self, key: str) -> bool:
        """Use message freshness, not process/topic existence, as readiness."""
        checker = getattr(self.node, "navigation_sensor_data_active", None)
        return bool(checker(key)) if callable(checker) else False

    def set_navigation_map_mode(self, mode: str) -> None:
        """Set map type for navigation setup and the active mini-map only."""
        self.map_panel.set_display_mode(mode)
        self.active_page.set_map_mode(mode, notify=False)
        self.grid_mode_button.setChecked(mode == "grid")
        self.cloud_mode_button.setChecked(mode == "cloud")

    def load_navigation_localization_map(self, path: str) -> bool:
        """Use the Localizer PCD as the AR navigation point-cloud map."""
        raw_path = str(path).strip()
        if not raw_path:
            return False
        source = FilePath(raw_path).expanduser()
        try:
            source = source.resolve()
            if source.suffix.lower() not in (".pcd", ".ply"):
                raise ValueError("定位地图必须是 PCD/PLY 点云文件")
            current_path = (
                self.local_cloud.path.resolve()
                if self.local_cloud is not None
                and self.local_cloud.path is not None else None)
            if current_path != source:
                cloud = load_pointcloud(source)
                self._set_local_cloud(cloud, reset_original=True)
            self.set_navigation_map_mode("cloud")
            cloud = self.local_cloud
            if cloud is not None:
                self.map_status.setText(
                    f"AR 导航已加载定位地图：{source.name} · "
                    f"{len(cloud.points):,} 点 · "
                    f"{self.map_panel.cloud_display_description()}")
            return cloud is not None
        except (OSError, ValueError) as exc:
            self.map_status.setText(f"AR 导航加载定位地图失败：{exc}")
            return False

    def set_editor_map_mode(self, mode: str) -> None:
        """Set map type in Map Studio without changing navigation views."""
        self.editor_map_panel.set_display_mode(mode)
        self.editor_grid_mode_button.setChecked(mode == "grid")
        self.editor_cloud_mode_button.setChecked(mode == "cloud")
        self.update_edit_tool()

    def set_map_mode(self, mode: str) -> None:
        """Backward-compatible alias for the navigation map selector."""
        self.set_navigation_map_mode(mode)

    def update_cloud_display_options(
        self, *_args, show_status: bool = True,
    ) -> None:
        display_options = dict(
            alignment="original",
            axis_order=str(self.cloud_axis_combo.currentData()),
            axis_flips=(
                self.cloud_flip_x.isChecked(),
                self.cloud_flip_y.isChecked(),
                self.cloud_flip_z.isChecked(),
            ),
            projection=str(self.cloud_projection_combo.currentData()),
            color_mode=str(self.cloud_color_combo.currentData()),
        )
        self.editor_map_panel.set_cloud_display_options(**display_options)
        # Map Studio may deliberately inspect height colors, but AR navigation
        # should consistently preserve the map's RGB/intensity appearance.
        navigation_options = dict(display_options)
        navigation_options["color_mode"] = "rgb"
        navigation_options["alignment"] = "original"
        self.map_panel.set_cloud_display_options(**navigation_options)
        self.active_page.map_panel.set_cloud_display_options(
            **navigation_options)
        display_transform_active = (
            self.cloud_axis_combo.currentData() != "XYZ"
            or self.cloud_flip_x.isChecked()
            or self.cloud_flip_y.isChecked()
            or self.cloud_flip_z.isChecked()
        )
        if display_transform_active:
            # Keep grid generation in the same coordinate system the user is
            # inspecting.  The original cloud remains untouched and can still
            # be selected after returning to an untransformed display.
            display_index = self.convert_coordinates_combo.findData("display")
            self.convert_coordinates_combo.setCurrentIndex(display_index)
        self.update_height_preview()
        if show_status and hasattr(self, "map_status"):
            suffix = ""
            if (
                hasattr(self, "edit_tool_combo")
                and self.edit_tool_combo.currentData()
                in ("cloud_add", "cloud_erase")
                and (
                    self.cloud_axis_combo.currentData() != "XYZ"
                    or self.cloud_flip_x.isChecked()
                    or self.cloud_flip_y.isChecked()
                    or self.cloud_flip_z.isChecked()
                )
            ):
                self.edit_tool_combo.setCurrentIndex(0)
                suffix += "；已退出点云画笔，防止在显示变换下误编辑"
            if (
                self.cloud_color_combo.currentData() == "rgb"
                and not self.editor_map_panel.cloud_has_rgb
            ):
                suffix += "；文件没有 RGB 字段，已回退到高度着色"
            self.map_status.setText(
                "点云显示已更新（原始点云未修改）· "
                + self.editor_map_panel.cloud_display_description() + suffix)

    def update_edit_tool(self, *_args) -> None:
        tool = str(self.edit_tool_combo.currentData())
        radius = float(self.brush_radius_spin.value())
        if (
            tool in ("cloud_add", "cloud_erase")
            and self.local_cloud is not None
            and self.local_cloud.is_gaussian_splat
        ):
            self.edit_tool_combo.blockSignals(True)
            self.edit_tool_combo.setCurrentIndex(0)
            self.edit_tool_combo.blockSignals(False)
            tool = "navigate"
            self.map_status.setText(
                "该 PLY 包含扩展点属性，当前版本仅支持浏览和转栅格，"
                "不允许按普通 XYZ 点云画笔修改")
        if tool in ("cloud_add", "cloud_erase") and (
            self.cloud_axis_combo.currentData() != "XYZ"
            or self.cloud_flip_x.isChecked()
            or self.cloud_flip_y.isChecked()
            or self.cloud_flip_z.isChecked()
        ):
            widgets = (
                self.cloud_axis_combo,
                self.cloud_flip_x, self.cloud_flip_y, self.cloud_flip_z,
            )
            for widget in widgets:
                widget.blockSignals(True)
            self.cloud_axis_combo.setCurrentIndex(
                self.cloud_axis_combo.findData("XYZ"))
            self.cloud_flip_x.setChecked(False)
            self.cloud_flip_y.setChecked(False)
            self.cloud_flip_z.setChecked(False)
            for widget in widgets:
                widget.blockSignals(False)
            self.update_cloud_display_options(show_status=False)
            self.map_status.setText(
                "点云画笔按原始 XY 坐标编辑，已临时切回原始坐标显示")
        self.editor_map_panel.set_edit_tool(tool, radius)
        browsing = "旋转" if self.editor_map_panel.display_mode == "cloud" else "浏览"
        self.editor_rotate_button.setText(browsing)
        self.editor_gesture_mode.setText("单指：" + (browsing if tool == "navigate" else "平移" if tool == "pan" else "编辑"))

    def _apply_gaussian_display_preset(self) -> None:
        widgets = (
            self.cloud_axis_combo,
            self.cloud_flip_x, self.cloud_flip_y, self.cloud_flip_z,
            self.cloud_projection_combo, self.cloud_color_combo,
            self.convert_coordinates_combo,
        )
        for widget in widgets:
            widget.blockSignals(True)
        # SuperSplat uses Y-up. Convert that to this renderer's Z-up after
        # applying the object's Z rotation = 180 degrees from the reference.
        self.cloud_axis_combo.setCurrentIndex(
            self.cloud_axis_combo.findData("XZY"))
        self.cloud_flip_x.setChecked(True)
        self.cloud_flip_y.setChecked(True)
        self.cloud_flip_z.setChecked(True)
        self.cloud_projection_combo.setCurrentIndex(
            self.cloud_projection_combo.findData("perspective"))
        self.cloud_color_combo.setCurrentIndex(
            self.cloud_color_combo.findData("rgb"))
        self.convert_coordinates_combo.setCurrentIndex(
            self.convert_coordinates_combo.findData("display"))
        for widget in widgets:
            widget.blockSignals(False)
        self.update_cloud_display_options(show_status=False)

    @staticmethod
    def _set_panel_cloud(panel: MapPanel, cloud: PointCloudMap) -> None:
        panel.set_pointcloud(
            cloud.points,
            cloud.colors,
            cloud.splat_scales,
            cloud.splat_rotations,
            cloud.splat_opacities,
            cloud.color_source,
        )

    def _set_local_cloud(
        self, cloud: PointCloudMap, reset_original: bool = False,
    ) -> None:
        self.local_cloud = cloud
        if reset_original:
            self.height_preview_check.setChecked(False)
            self._cloud_original = cloud.points.copy()
            self._cloud_original_colors = (
                None if cloud.colors is None else cloud.colors.copy())
            self._undo_stack.clear()
            self.cloud_color_combo.model().item(self.cloud_color_combo.findData("intensity")).setEnabled(
                cloud.color_source == "intensity")
            self.cloud_color_combo.setCurrentIndex(self.cloud_color_combo.findData("height"))
            if cloud.is_gaussian_splat:
                self._apply_gaussian_display_preset()
        self._cloud_edit_revision += 1
        self._rendered_cloud_edit_revision = -1
        for panel in (
            self.map_panel, self.editor_map_panel, self.active_page.map_panel,
        ):
            self._set_panel_cloud(panel, cloud)
        if reset_original:
            self.editor_map_panel.reset_cloud_view()
        self.update_height_preview()

    def _set_local_grid(
        self, grid: GridMap, reset_original: bool = False,
    ) -> None:
        self.local_grid = grid
        if reset_original:
            self._grid_original = grid.occupancy.copy()
            self._undo_stack.clear()
        self._grid_edit_revision += 1
        self._rendered_grid_edit_revision = -1
        self._render_grid(grid)

    def _render_grid(self, grid: GridMap) -> None:
        for panel in (
            self.map_panel, self.editor_map_panel, self.active_page.map_panel,
        ):
            panel.set_map(
                grid.occupancy, grid.origin[:2], grid.resolution,
                float(grid.origin[2]))
        self._rendered_grid_edit_revision = self._grid_edit_revision

    def load_pointcloud_dialog(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self, "加载点云地图", str(self.map_storage_dir),
            "Point Cloud (*.pcd *.ply);;PCD (*.pcd);;PLY (*.ply);;All Files (*)")
        if not path:
            return
        try:
            cloud = load_pointcloud(path)
            self._set_local_cloud(cloud, reset_original=True)
            self.set_editor_map_mode("cloud")
            self.map_status.setText(
                f"点云已加载：{FilePath(path).name} · {len(cloud.points):,} 点 · "
                f"{self.editor_map_panel.cloud_display_description()}")
        except (OSError, ValueError) as exc:
            self.map_status.setText(f"点云加载失败：{exc}")

    def load_grid_dialog(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self, "加载栅格地图", str(self.map_storage_dir),
            "Nav2 Map (*.yaml *.yml);;Portable Graymap (*.pgm)")
        if not path:
            return
        try:
            grid = load_grid_map(path, self.grid_resolution_spin.value())
            self._set_local_grid(grid, reset_original=True)
            self.grid_resolution_spin.setValue(grid.resolution)
            self.node.grid_frame = grid.frame_id
            self.set_editor_map_mode("grid")
            h, w = grid.occupancy.shape
            self.map_status.setText(
                f"栅格已加载：{FilePath(path).name} · {w}×{h} · {grid.resolution:.3f} m")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.map_status.setText(f"栅格加载失败：{exc}")

    def update_grid_settings_label(self, *_args):
        self.grid_settings_label.setText(
            f"新输出每格 {self.grid_resolution_spin.value()*100:g} cm · "
            f"障碍外扩 {self.grid_inflation_spin.value()*100:g} cm；修改后重新转换")

    def conversion_uses_display_coordinates(self):
        return (self.convert_coordinates_combo.currentData() == "display"
                or self.cloud_axis_combo.currentData() != "XYZ"
                or self.cloud_flip_x.isChecked() or self.cloud_flip_y.isChecked()
                or self.cloud_flip_z.isChecked())

    def height_limits_changed(self, *_args):
        self.height_preview_check.setChecked(True)
        self.update_height_preview()

    def update_height_preview(self, *_args):
        panel = self.editor_map_panel
        display = self.conversion_uses_display_coordinates()
        low, high = self.z_min_spin.value(), self.z_max_spin.value()
        enabled = self.height_preview_check.isChecked() and low < high
        panel.set_height_slice((low, high) if enabled else None,
            display_coordinates=display, only_retained=self.height_only_check.isChecked())
        if low >= high:
            self.height_preview_status.setText("下界必须小于上界；当前区间无效")
        elif not self.height_preview_check.isChecked():
            self.height_preview_status.setText("高度预览已关闭；转换仍使用设定的高度区间")
        elif self.local_cloud is not None:
            mask = panel.height_slice_mask()
            coordinates = "当前显示坐标" if display else "原始 XYZ"
            self.height_preview_status.setText(
                f"{coordinates} Z={low:.2f}～{high:.2f} m · "
                f"预览保留 {np.count_nonzero(mask):,}/{len(mask):,} 点；区间外变灰。转换使用完整编辑后点云。")

    def pick_height_plane(self, target):
        if self.local_cloud is None:
            self.height_preview_status.setText("请先加载 PCD/PLY")
            return
        self.set_editor_map_mode("cloud")
        if target == "ground":
            self.height_only_check.setChecked(False)
        self.height_preview_check.setChecked(True)
        self.edit_tool_combo.setCurrentIndex(0)
        self._height_pick_target = target
        self.editor_map_panel._height_pick_enabled = True
        self.height_preview_status.setText("请在点云表面单击，设置" + {"lower": "下平面", "upper": "上平面", "ground": "地面高度"}[target])

    def height_plane_selected(self, height):
        control = {"lower": self.z_min_spin, "upper": self.z_max_spin, "ground": self.floor_z_spin}[self._height_pick_target]
        self._height_pick_target = None
        control.setValue(height)
        self.update_height_preview()

    def convert_pointcloud(self) -> None:
        cloud = self.local_cloud
        if cloud is None and getattr(self.node, "latest_pointcloud", None) is not None:
            cloud = PointCloudMap(
                self.node.latest_pointcloud.copy(),
                frame_id=self.node.pointcloud_frame)
        if cloud is None or not len(cloud.points):
            self.map_status.setText("请先加载 PCD/PLY 或配置 pointcloud_topic")
            return
        try:
            display_transform_active = (
                self.cloud_axis_combo.currentData() != "XYZ"
                or self.cloud_flip_x.isChecked()
                or self.cloud_flip_y.isChecked()
                or self.cloud_flip_z.isChecked()
            )
            if display_transform_active:
                self.convert_coordinates_combo.setCurrentIndex(
                    self.convert_coordinates_combo.findData("display"))
            use_display_coordinates = self.conversion_uses_display_coordinates()
            conversion_points = (
                self.editor_map_panel.transform_cloud_points(cloud.points)
                if use_display_coordinates else cloud.points)
            floor_z = self.floor_z_spin.value()
            tolerance = self.floor_tolerance_spin.value()
            if floor_z + tolerance >= self.z_min_spin.value():
                raise ValueError("地面层上界须低于障碍高度下界，请调整地面高度或障碍下界")
            floor = conversion_points[
                np.isfinite(conversion_points).all(axis=1)
                & (np.abs(conversion_points[:, 2] - floor_z) <= tolerance)]
            if not len(floor):
                raise ValueError("指定地面高度附近没有实测点，请点选地面或调整地面高度与厚度")
            grid = pointcloud_to_grid(
                conversion_points,
                resolution=float(self.grid_resolution_spin.value()),
                z_min=float(self.z_min_spin.value()),
                z_max=float(self.z_max_spin.value()),
                inflation_radius=float(self.grid_inflation_spin.value()),
                free_points=floor,
            )
            grid.frame_id = cloud.frame_id
            self._set_local_grid(grid, reset_original=True)
            self.node.grid_frame = grid.frame_id
            self.set_editor_map_mode("grid")
            h, w = grid.occupancy.shape
            coordinates = (
                "当前显示坐标（手动轴变换）"
                if use_display_coordinates else "原始 XYZ")
            policy = (f"黑色障碍 {np.count_nonzero(grid.occupancy == 100):,} 格 · "
                      f"白色地面 {np.count_nonzero(grid.occupancy == 0):,} 格 · "
                      f"灰色未知 {np.count_nonzero(grid.occupancy == -1):,} 格")
            self.map_status.setText(
                f"转换完成：{w}×{h} 格，{w*grid.resolution:.2f}×{h*grid.resolution:.2f} m，"
                f"每格 {grid.resolution*100:g} cm，外扩 {self.grid_inflation_spin.value()*100:g} cm"
                f"（使用{coordinates}）；{policy}，请核查后保存")
        except ValueError as exc:
            self.map_status.setText(f"点云转栅格失败：{exc}")

    def convert_terrain_grid(self) -> None:
        from .features.terrain_grid import build_terrain_grid
        from .features.resource_lease import ResourceLease
        library = self.operations_page.library
        if not library.available():
            self.map_status.setText(library.status.text())
            return
        cloud = self.local_cloud
        if cloud is None or not len(cloud.points):
            self.map_status.setText("请先加载并编辑 PCD/PLY 点云")
            return
        points = (self.editor_map_panel.transform_cloud_points(cloud.points)
                  if self.conversion_uses_display_coordinates() else cloud.points.copy())
        revisions = (self._cloud_edit_revision, self._grid_edit_revision)
        frame = cloud.frame_id
        resolution = float(self.grid_resolution_spin.value())
        def work():
            with ResourceLease('model'):
                return build_terrain_grid(points, resolution=resolution)
        def completed(value, error):
            self.convert_button.setEnabled(True)
            self.terrain_convert_button.setEnabled(True)
            if error:
                self.map_status.setText("局部高程转栅格失败："+error)
                return
            if revisions != (self._cloud_edit_revision, self._grid_edit_revision):
                self.map_status.setText("地图已修改，本次转换结果未应用，请重新转换")
                return
            grid, _layers, metadata = value
            grid.frame_id = frame
            self._set_local_grid(grid, reset_original=True)
            self.node.grid_frame = frame
            self.set_editor_map_mode("grid")
            h, w = grid.occupancy.shape
            self.map_status.setText(f"局部高程通行性转换完成：{w}×{h} 格，{grid.resolution*100:g} cm/格；"
                f"地面参考 Z={metadata['ground_reference_z_m']:.2f} m；"
                f"黑 {metadata['occupied_cells']:,} · 白 {metadata['free_cells']:,} · 灰 {metadata['unknown_cells']:,}，请核查后保存")
        self.convert_button.setEnabled(False)
        self.terrain_convert_button.setEnabled(False)
        self.map_status.setText("正在分析局部高程与通行性，请稍候…")
        library.worker.start(work, completed)

    def save_current_map(self) -> None:
        try:
            if self.editor_map_panel.display_mode == "cloud":
                if self.local_cloud is None:
                    self.map_status.setText("当前没有可保存的点云")
                    return
                suggested = str(unique_map_file(
                    self.map_storage_dir, "nav_map", ".pcd"))
                path, _selected = QFileDialog.getSaveFileName(
                    self, "保存点云地图", suggested, "Point Cloud (*.pcd)")
                if not path:
                    return
                saved = save_pcd(
                    path, self.local_cloud.points, self.local_cloud.colors)
                self.local_cloud.path = saved
                self._cloud_original = self.local_cloud.points.copy()
                self._cloud_original_colors = (
                    None if self.local_cloud.colors is None
                    else self.local_cloud.colors.copy())
                self.map_status.setText(f"点云已保存：{saved}")
            else:
                grid = self._ensure_local_grid()
                if grid is None:
                    self.map_status.setText("当前没有可保存的栅格地图")
                    return
                suggested = str(unique_map_file(
                    self.map_storage_dir, "nav_map", ".yaml"))
                path, _selected = QFileDialog.getSaveFileName(
                    self, "保存 Nav2 栅格地图", suggested, "Nav2 Map (*.yaml)")
                if not path:
                    return
                yaml_path, pgm_path = save_grid_map(path, grid)
                grid.path = yaml_path
                self._grid_original = grid.occupancy.copy()
                self.map_status.setText(
                    f"栅格已保存：{yaml_path.name} / {pgm_path.name}")
        except (OSError, ValueError) as exc:
            self.map_status.setText(f"地图保存失败：{exc}")

    def _ensure_local_grid(self) -> Optional[GridMap]:
        if self.local_grid is not None:
            return self.local_grid
        occupancy = getattr(self.node, "occupancy", None)
        resolution = float(getattr(self.node, "map_resolution", 0.0))
        if occupancy is None or resolution <= 0.0:
            return None
        origin_xy = np.asarray(self.node.map_origin, dtype=float)
        origin = np.array([
            origin_xy[0], origin_xy[1], float(self.node.map_origin_yaw)])
        grid = GridMap(
            occupancy.copy(), resolution, origin,
            frame_id=str(getattr(self.node, "grid_frame", "map")))
        self.local_grid = grid
        self._grid_original = grid.occupancy.copy()
        self._render_grid(grid)
        return grid

    def _ensure_local_cloud(self) -> Optional[PointCloudMap]:
        if self.local_cloud is not None:
            return self.local_cloud
        points = getattr(self.node, "latest_pointcloud", None)
        if points is None:
            return None
        cloud = PointCloudMap(
            points.copy(), frame_id=str(self.node.pointcloud_frame))
        self.local_cloud = cloud
        self._cloud_original = cloud.points.copy()
        self._cloud_original_colors = None
        return cloud

    def snapshot_map_edit(self) -> None:
        mode = self.editor_map_panel.display_mode
        if mode == "grid":
            grid = self._ensure_local_grid()
            if grid is not None:
                self._undo_stack.append(("grid", grid.occupancy.copy()))
        else:
            cloud = self._ensure_local_cloud()
            if cloud is not None:
                self._undo_stack.append(("cloud", (
                    cloud.points.copy(),
                    None if cloud.colors is None else cloud.colors.copy(),
                )))
        self._undo_stack = self._undo_stack[-12:]

    def edit_map_at(
        self, mode: str, tool: str, x: float, y: float, radius: float,
    ) -> None:
        if mode == "grid":
            if tool not in ("occupied", "free", "unknown"):
                self.map_status.setText("栅格模式请选择障碍物、自由区域或未知区域画笔")
                return
            grid = self._ensure_local_grid()
            if grid is None:
                self.map_status.setText("尚无栅格地图，无法编辑")
                return
            relative = np.array([x, y]) - grid.origin[:2]
            yaw = float(grid.origin[2])
            local = np.array([
                math.cos(yaw) * relative[0] + math.sin(yaw) * relative[1],
                -math.sin(yaw) * relative[0] + math.cos(yaw) * relative[1],
            ])
            center_x, center_y = np.floor(local / grid.resolution).astype(int)
            cells = max(1, int(math.ceil(radius / grid.resolution)))
            height, width = grid.occupancy.shape
            x0, x1 = max(0, center_x - cells), min(width, center_x + cells + 1)
            y0, y1 = max(0, center_y - cells), min(height, center_y + cells + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            mask = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= cells ** 2
            value = {"occupied": 100, "free": 0, "unknown": -1}[tool]
            patch = grid.occupancy[y0:y1, x0:x1]
            changed = mask & (patch != value)
            if not np.any(changed):
                return
            patch[changed] = value
            self._grid_edit_revision += 1
            for panel in (self.map_panel, self.editor_map_panel, self.active_page.map_panel):
                panel.update_grid_patch(patch, x0, y0)
            self._rendered_grid_edit_revision = self._grid_edit_revision
            self.map_status.setText("栅格地图已修改（尚未保存）")
            return

        if tool not in ("cloud_add", "cloud_erase"):
            self.map_status.setText("点云模式请选择添加点或擦除点工具")
            return
        cloud = self._ensure_local_cloud()
        if cloud is None:
            self.map_status.setText("尚无点云地图，无法编辑")
            return
        if tool == "cloud_erase":
            distance_sq = (
                (cloud.points[:, 0] - x) ** 2 + (cloud.points[:, 1] - y) ** 2)
            keep = distance_sq > radius ** 2
            cloud.points = np.ascontiguousarray(cloud.points[keep])
            if cloud.colors is not None:
                cloud.colors = np.ascontiguousarray(cloud.colors[keep])
        else:
            angles = np.linspace(0.0, 2.0 * math.pi, 18, endpoint=False)
            radial = np.array([0.0, radius * 0.5, radius])
            z_low, z_high = sorted((self.z_min_spin.value(), self.z_max_spin.value()))
            heights = np.linspace(z_low, z_high, 6)
            additions = []
            for z_value in heights:
                for ring in radial:
                    additions.extend([
                        [x + ring * math.cos(angle), y + ring * math.sin(angle), z_value]
                        for angle in angles
                    ])
            additions = np.asarray(additions, dtype=np.float32)
            cloud.points = np.vstack([cloud.points, additions])
            if cloud.colors is not None:
                added_colors = np.tile(
                    np.array([[0, 210, 255]], dtype=np.uint8),
                    (len(additions), 1))
                cloud.colors = np.vstack([cloud.colors, added_colors])
        self._cloud_edit_revision += 1
        for panel in (
            self.map_panel, self.editor_map_panel, self.active_page.map_panel,
        ):
            self._set_panel_cloud(panel, cloud)
        self.map_status.setText(
            f"点云地图已修改 · 当前 {len(cloud.points):,} 点（尚未保存）")

    def undo_map_edit(self) -> None:
        if not self._undo_stack:
            self.map_status.setText("没有可撤销的地图编辑")
            return
        mode, data = self._undo_stack.pop()
        if mode == "grid" and self.local_grid is not None:
            self.local_grid.occupancy = data
            self._grid_edit_revision += 1
            self._render_grid(self.local_grid)
            self.set_editor_map_mode("grid")
        elif mode == "cloud" and self.local_cloud is not None:
            points, colors = data
            self.local_cloud.points = points
            self.local_cloud.colors = colors
            self._cloud_edit_revision += 1
            for panel in (
                self.map_panel, self.editor_map_panel,
                self.active_page.map_panel,
            ):
                self._set_panel_cloud(panel, self.local_cloud)
            self.set_editor_map_mode("cloud")
        self.map_status.setText("已撤销上一笔地图编辑")

    def reset_map_edits(self) -> None:
        if self.editor_map_panel.display_mode == "grid":
            if self.local_grid is None or self._grid_original is None:
                self.map_status.setText("当前栅格地图没有可恢复的本地版本")
                return
            self.local_grid.occupancy = self._grid_original.copy()
            self._grid_edit_revision += 1
            self._render_grid(self.local_grid)
        else:
            if self.local_cloud is None or self._cloud_original is None:
                self.map_status.setText("当前点云地图没有可恢复的本地版本")
                return
            self.local_cloud.points = self._cloud_original.copy()
            self.local_cloud.colors = (
                None if self._cloud_original_colors is None
                else self._cloud_original_colors.copy())
            self._cloud_edit_revision += 1
            for panel in (
                self.map_panel, self.editor_map_panel,
                self.active_page.map_panel,
            ):
                self._set_panel_cloud(panel, self.local_cloud)
        self._undo_stack.clear()
        self.map_status.setText("已恢复到最近一次加载/保存的地图")

    def on_map_point(self, x: float, y: float) -> None:
        point = np.array([x, y], dtype=np.float32)
        edit_index = self.waypoint_edit_index
        if edit_index is not None and 0 <= edit_index < len(self.waypoints):
            self.waypoints[edit_index] = point
            self.waypoint_edit_index = None
            self.waypoint_list.setEnabled(True)
            self.node.navigation_status = f"已更新途径点 {edit_index + 1}"
            self.nav_status.setText(self.node.navigation_status)
            self._sync_selection_ui(edit_index)
            return
        self.waypoints.append(point)
        self.node.navigation_status = f"已添加途径点 {len(self.waypoints)}"
        self.nav_status.setText(self.node.navigation_status)
        self._sync_selection_ui(len(self.waypoints) - 1)

    def clear_selection(self) -> None:
        self.waypoints = []
        self.waypoint_edit_index = None
        self.waypoint_list.setEnabled(True)
        self._sync_selection_ui()

    def _on_waypoint_selection_changed(self, row: int) -> None:
        valid = 0 <= row < len(self.waypoints)
        editing = self.waypoint_edit_index is not None
        self.edit_waypoint_button.setEnabled(valid or editing)
        self.edit_waypoint_button.setText(
            "取消重新选点" if editing else "重新选点")
        self.delete_waypoint_button.setEnabled(valid and not editing)
        self.start_button.setEnabled(bool(self.waypoints) and not editing)
        highlighted = self.waypoint_edit_index if editing else (row if valid else None)
        self.map_panel.set_waypoint_selection(highlighted)

    def begin_waypoint_reselection(self, *_args) -> None:
        if self.waypoint_edit_index is not None:
            previous = self.waypoint_edit_index
            self.waypoint_edit_index = None
            self.waypoint_list.setEnabled(True)
            self.node.navigation_status = "已取消重新选点"
            self.nav_status.setText(self.node.navigation_status)
            self._sync_selection_ui(previous)
            return
        row = self.waypoint_list.currentRow()
        if not 0 <= row < len(self.waypoints):
            self.node.navigation_status = "请先在列表中选择一个途径点"
            self.nav_status.setText(self.node.navigation_status)
            return
        self.waypoint_edit_index = row
        self.waypoint_list.setEnabled(False)
        self.node.navigation_status = (
            f"正在修改途径点 {row + 1}：请在地图上短按新的位置")
        self.nav_status.setText(self.node.navigation_status)
        self._on_waypoint_selection_changed(row)

    def delete_selected_waypoint(self) -> None:
        row = self.waypoint_list.currentRow()
        if not 0 <= row < len(self.waypoints):
            self.node.navigation_status = "请先在列表中选择要删除的途径点"
            self.nav_status.setText(self.node.navigation_status)
            return
        self.waypoints.pop(row)
        self.waypoint_edit_index = None
        self.waypoint_list.setEnabled(True)
        self.node.navigation_status = f"已删除途径点 {row + 1}"
        self.nav_status.setText(self.node.navigation_status)
        next_row = min(row, len(self.waypoints) - 1)
        self._sync_selection_ui(next_row)

    def _sync_selection_ui(self, preferred_index: Optional[int] = None) -> None:
        current_row = (
            self.waypoint_list.currentRow()
            if preferred_index is None else preferred_index)
        self.waypoint_list.blockSignals(True)
        self.waypoint_list.clear()
        for index, point in enumerate(self.waypoints):
            self.waypoint_list.addItem(
                f"{index + 1}.   X {point[0]:.2f} m    Y {point[1]:.2f} m")
        if self.waypoints:
            current_row = int(np.clip(current_row, 0, len(self.waypoints) - 1))
            self.waypoint_list.setCurrentRow(current_row)
        else:
            current_row = -1
        self.waypoint_list.blockSignals(False)
        self.waypoint_title.setText(
            f"途径点（{len(self.waypoints)}）"
            if self.waypoints else "途径点（尚未添加）")
        self.map_panel.set_waypoints(self.waypoints)
        self._on_waypoint_selection_changed(current_row)

    def start_navigation(self) -> None:
        if self.waypoint_edit_index is not None:
            self.node.navigation_status = "请先在地图上完成途径点重新选择"
            self.nav_status.setText(self.node.navigation_status)
            return
        self.pending_navigation_result_status = None
        self.active_page.clear_navigation_result()
        self.last_navigation_result_revision = int(
            getattr(self.node, "navigation_result_revision", 0))
        if self.node.send_navigation_waypoints(self.waypoints):
            self._clear_navigation_overlays()
            self._navigation_overlay_enabled = True
            self.active_page.map_panel.set_waypoints([])
            self.active_page.map_panel.set_motion_trace(None)
            self.active_page.follow_button.setChecked(True)
            self.pages.setCurrentWidget(self.active_page)

    def begin_navigation_guidance(self) -> None:
        if not self.node.begin_guidance():
            self.nav_status.setText(self.node.navigation_status)
            return
        self.active_page.set_preview_available(False)
        self.pages.setCurrentWidget(self.active_page)

    def _clear_navigation_overlays(self) -> None:
        self._navigation_overlay_enabled = False
        self._motion_trace = []
        self._cached_route = None
        self._ar_frame_key = None
        self._cached_guidance = None
        self._cached_remaining_distance = 0.0
        self._last_route_refresh = 0.0
        self._last_map_navigation_refresh = 0.0
        for panel in (self.map_panel, self.active_page.map_panel):
            panel.set_navigation_state(None, panel._robot_xy, panel._robot_yaw)
            panel.set_motion_trace(None)
        for panel in (self.camera_panel, self.active_page.camera_panel):
            panel.set_frame(None, None, 0.0)

    def exit_navigation(self) -> None:
        if self.pending_navigation_result_status is not None:
            self._finish_navigation_return()
            return
        self.node.cancel_navigation()
        self._clear_navigation_overlays()
        self.active_page.clear_navigation_result()
        self.clear_selection()
        self.pages.setCurrentWidget(self.setup_page)

    def _handle_navigation_terminal_state(self) -> None:
        revision = int(getattr(
            self.node, "navigation_result_revision",
            self.last_navigation_result_revision))
        if revision == self.last_navigation_result_revision:
            return
        self.last_navigation_result_revision = revision
        self._clear_navigation_overlays()
        if self.pages.currentWidget() is not self.active_page:
            return
        status = getattr(self.node, "navigation_result_status", None)
        self.pending_navigation_result_status = status
        self.active_page.show_navigation_result(
            status, str(self.node.navigation_status))

    def _finish_navigation_return(self) -> None:
        status = self.pending_navigation_result_status
        self.pending_navigation_result_status = None
        self.clear_selection()
        self._clear_navigation_overlays()
        self.active_page.clear_navigation_result()
        self.pages.setCurrentWidget(self.setup_page)
        if status == 4:
            self.node.navigation_status = "导航成功，已返回地图"

    def refresh(self) -> None:
        self._handle_navigation_terminal_state()
        self._refresh_sensor_previews()
        current_page = self.pages.currentWidget()
        if current_page is self.relocalization_page:
            self._refresh_relocalization()
        self.mapping_page.refresh()
        if current_page is self.system_home_page:
            self._refresh_system_home()
        image = self.node.latest_image
        now = time.monotonic()
        image_revision = int(getattr(
            self.node, "image_revision", id(image) if image is not None else 0))
        if current_page is self.active_page:
            walking = getattr(self.node, "walking", None)
            self.active_page.set_preview_available(
                walking is not None and walking.phase == "preview")
            frame_key = (image_revision, id(getattr(self.node, "path", None)),
                         getattr(self.node, "navigation_active", None))
            if frame_key != self._ar_frame_key or now - self._last_route_refresh >= 1.0 / 15.0:
                self._ar_frame_key = frame_key
                self._cached_route = (
                    self.node.projected_route(image.shape)
                    if image is not None and self._navigation_overlay_enabled else None)
                guidance = getattr(self.node, "guidance_state", None)
                self._cached_guidance = (guidance() if self._navigation_overlay_enabled
                                         and guidance is not None else None)
                self._cached_remaining_distance = ((
                    self._cached_guidance['remaining_m'] or 0. if self._cached_guidance is not None
                    else self.node.remaining_distance()) if self._navigation_overlay_enabled else 0.0)
                self._last_route_refresh = now
            self.active_page.camera_panel.set_frame(
                image,
                self._cached_route,
                self._cached_remaining_distance,
                image_revision,
                guidance=self._cached_guidance,
                projection_status=(str(self.node.navigation_status)
                    if walking is not None and walking.phase in ("planning", "preview", "error")
                    else getattr(self.node, "ar_status", {}).get("message", "")),
            )
            if hasattr(self.node, "_image_received_at"):
                stamp = self.node.image_stamp
                valid_stamp = stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0)
                source_age = ((self.node.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
                              if valid_stamp else float('inf'))
                self.active_page.camera_panel.set_frame_age(now-self.node._image_received_at, source_age)
        elif current_page is self.setup_page:
            self.camera_panel.set_frame(
                image, None, 0.0, image_revision)
        if self.local_grid is not None:
            if self._rendered_grid_edit_revision != self._grid_edit_revision:
                self._render_grid(self.local_grid)
                self._rendered_grid_edit_revision = self._grid_edit_revision
        elif self.node.map_revision != self.last_map_revision:
            for panel in (
                self.map_panel, self.editor_map_panel,
                self.active_page.map_panel,
            ):
                panel.set_map(
                    self.node.occupancy,
                    self.node.map_origin,
                    self.node.map_resolution,
                    self.node.map_origin_yaw,
                )
            self.last_map_revision = self.node.map_revision
        if self.local_cloud is not None:
            if self._rendered_cloud_edit_revision != self._cloud_edit_revision:
                for panel in (
                    self.map_panel, self.editor_map_panel,
                    self.active_page.map_panel,
                ):
                    self._set_panel_cloud(panel, self.local_cloud)
                self._rendered_cloud_edit_revision = self._cloud_edit_revision
        elif self.node.latest_pointcloud is not None:
            for panel in (self.map_panel, self.editor_map_panel, self.active_page.map_panel):
                if (panel.isVisible() and getattr(panel, "_live_cloud_revision", -1)
                        != self.node.pointcloud_revision):
                    panel.set_pointcloud(self.node.latest_pointcloud)
                    panel._live_cloud_revision = self.node.pointcloud_revision
        if (
            current_page in (self.setup_page, self.active_page)
            and now - self._last_map_navigation_refresh >= 1.0 / 15.0
        ):
            path, robot_xy, robot_yaw = self.node.map_navigation_state()
            preview = (getattr(getattr(self.node, "walking", None), "phase", "idle") == "preview"
                       and bool(self.waypoints))
            display_navigation = self._navigation_overlay_enabled and (
                getattr(self.node, "navigation_active", False) or preview)
            if not display_navigation:
                path = None
                self._motion_trace = []
            if display_navigation and robot_xy is not None and (
                not self._motion_trace
                or np.linalg.norm(robot_xy - self._motion_trace[-1]) >= .05
            ):
                self._motion_trace.append(np.asarray(robot_xy).copy())
                if len(self._motion_trace) > 4000:
                    self._motion_trace = self._motion_trace[-4000:]
            trace = (
                np.asarray(self._motion_trace, dtype=np.float32)
                if len(self._motion_trace) >= 2 else None)
            self.map_panel.set_navigation_state(path, robot_xy, robot_yaw)
            self.active_page.map_panel.set_navigation_state(
                path, robot_xy, robot_yaw)
            self.map_panel.set_motion_trace(trace)
            self.active_page.map_panel.set_motion_trace(trace)
            self._last_map_navigation_refresh = now
        if self.navigation_stack_controller.overall_state in ("starting", "stopping", "error"):
            self.nav_status.setText(self.navigation_stack_page.action_status.text())
        else:
            self.nav_status.setText(self.node.navigation_status)

    def _record_close_state(self, kind, state):
        if self._closing_after_record and kind == 'record' and state in ('stopped', 'error'):
            QTimer.singleShot(0, self.close)

    def closeEvent(self, event) -> None:
        """Wait for recording receipts before stopping child processes."""
        operations = self.operations_page
        if operations.recordings.deleting or operations.diagnostics_page.exporting:
            if operations.recordings.deleting:
                self.show_recordings()
                operations.bags.tabs.setCurrentWidget(operations.recordings)
                operations.recordings.status.setText('录包删除尚未完成，请等待后退出')
            else:
                self.pages.setCurrentWidget(self.diagnostics_workspace_page)
                operations.diagnostics_page.status.setText('诊断导出尚未完成，请等待后退出')
            event.ignore()
            return
        if operations.library.worker.busy:
            operations.library.status.setText('地图包处理尚未完成，请等待完成后退出')
            self.map_pages.setCurrentWidget(operations.library)
            self.pages.setCurrentWidget(self.map_workspace_page)
            event.ignore()
            return
        if operations.controller.recording:
            self._closing_after_record = True
            operations.controller.stop_recording()
            operations.bags.set_status('正在保存录包，完成后退出；请保持供电')
            self.show_recordings()
            event.ignore()
            return
        self.manual_drive_page.shutdown()
        operations.shutdown()
        self.refresh_timer.stop()
        self.navigation_stack_controller.shutdown()
        self.localization_tools_controller.shutdown()
        self.sensor_driver_controller.shutdown()
        self.mapping_controller.shutdown()
        self.sensor_preview_adapter.stop()
        super().closeEvent(event)


def main(args=None) -> None:
    from .features.log_store import install_exception_logging, log_store
    from .ui.web_map import prepare_web_engine
    install_exception_logging()
    prepare_web_engine()
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName("NAV")
    node = QtNavRosNode()
    window = NavigationWindow(node)
    screen = app.primaryScreen()
    available = screen.availableGeometry() if screen is not None else None
    small_screen = available is not None and min(
        available.width(), available.height()) < 700
    if node.fullscreen or (small_screen and node.fullscreen_on_small_screen):
        window.showFullScreen()
    else:
        window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())

    ros_timer = QTimer()
    ros_timer.setTimerType(Qt.PreciseTimer)
    from .ros_event_pump import process_ros_events, GuiRosExecutor
    ros_executor = GuiRosExecutor(context=node.context)
    ros_executor.add_node(node)
    ros_timer.timeout.connect(lambda: process_ros_events(
        node, max_callbacks=4, budget_s=.0015, executor=ros_executor))
    ros_timer.start(5)
    try:
        app.exec_()
    finally:
        ros_timer.stop()
        ros_executor.remove_node(node)
        ros_executor.shutdown()
        node.destroy_node()
        log_store().finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Touch-first two-point map relocalization page."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..features.localization_tools import initial_pose_from_points
from ..qt_nav_node import MapPanel


def _label(text: str = "", role: str = "body") -> QLabel:
    widget = QLabel(text)
    widget.setTextFormat(Qt.PlainText)
    widget.setObjectName(role)
    widget.setWordWrap(True)
    return widget


def _button(text: str, primary: bool = False) -> QPushButton:
    widget = QPushButton(text)
    widget.setObjectName(
        "relocalizePrimary" if primary else "relocalizeButton")
    widget.setMinimumHeight(46)
    widget.setMaximumHeight(52)
    widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return widget


class RelocalizationMap(MapPanel):
    """A pannable 2-D map with a distinct, unconfirmed pose overlay."""

    DEFAULT_TOUCH_ZOOM = 1.35
    MAX_TOUCH_FIT_ZOOM = 8.0
    MIN_TOUCH_MAP_SIDE = 156.0
    TOUCH_SLOP = 18.0

    def __init__(self, parent=None) -> None:
        super().__init__(cloud_3d=False)
        self.setParent(parent)
        self.setMinimumSize(120, 90)
        self.zoom = 1.0
        self.pan = QPointF()
        self._down: Optional[QPointF] = None
        self._last: Optional[QPointF] = None
        self._dragged = False
        self._touch_fit_active = True
        self._selection_enabled = True
        self.candidate_position: Optional[np.ndarray] = None
        self.candidate_direction: Optional[np.ndarray] = None

    def set_map(
        self,
        occupancy: Optional[np.ndarray],
        origin: np.ndarray,
        resolution: float,
        origin_yaw: float,
    ) -> None:
        """Drop unknown-only borders from this view without changing map coordinates."""
        if occupancy is None:
            super().set_map(None, origin, resolution, origin_yaw)
            return
        displayed = np.asarray(occupancy)
        adjusted_origin = np.asarray(origin, dtype=float)[:2].copy()
        known_y, known_x = np.nonzero(displayed >= 0)
        if len(known_x) and resolution > 0.0:
            padding = max(2, min(24, int(math.ceil(0.50 / resolution))))
            x0 = max(0, int(known_x.min()) - padding)
            x1 = min(displayed.shape[1], int(known_x.max()) + padding + 1)
            y0 = max(0, int(known_y.min()) - padding)
            y1 = min(displayed.shape[0], int(known_y.max()) + padding + 1)
            if x0 or y0 or x1 < displayed.shape[1] or y1 < displayed.shape[0]:
                displayed = displayed[y0:y1, x0:x1]
                local_offset = np.array(
                    [x0 * resolution, y0 * resolution], dtype=float)
                cos_yaw = math.cos(origin_yaw)
                sin_yaw = math.sin(origin_yaw)
                adjusted_origin += np.array([
                    cos_yaw * local_offset[0] - sin_yaw * local_offset[1],
                    sin_yaw * local_offset[0] + cos_yaw * local_offset[1],
                ])
        super().set_map(displayed, adjusted_origin, resolution, origin_yaw)

    def _map_rect(self) -> QRectF:
        rect = super()._map_rect()
        if rect.isEmpty():
            return rect
        center = rect.center() + self.pan
        return QRectF(
            center.x() - rect.width() * self.zoom * 0.5,
            center.y() - rect.height() * self.zoom * 0.5,
            rect.width() * self.zoom,
            rect.height() * self.zoom,
        )

    def set_candidate(self, position, direction) -> None:
        self.candidate_position = (
            None if position is None
            else np.asarray(position, dtype=float)[:2].copy())
        self.candidate_direction = (
            None if direction is None
            else np.asarray(direction, dtype=float)[:2].copy())
        self.update()

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = bool(enabled)
        self.setCursor(
            Qt.PointingHandCursor if self._selection_enabled
            else Qt.ArrowCursor)

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)
        self._down = event.localPos()
        self._last = event.localPos()
        self._dragged = False
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if self._down is None or self._last is None:
            return super().mouseMoveEvent(event)
        if (event.localPos() - self._down).manhattanLength() > 12:
            self._dragged = True
        if self._dragged:
            self._touch_fit_active = False
            self.pan += event.localPos() - self._last
            self.update()
        self._last = event.localPos()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if self._down is None:
            return super().mouseReleaseEvent(event)
        if (
            event.button() == Qt.LeftButton
            and not self._dragged
            and self._selection_enabled
        ):
            point = self._touch_to_world(event.localPos())
            if point is not None:
                self.point_selected.emit(float(point[0]), float(point[1]))
        self._down = None
        self._last = None
        event.accept()

    def change_zoom(self, factor: float) -> None:
        self._touch_fit_active = False
        self.zoom = min(
            self.MAX_TOUCH_FIT_ZOOM,
            max(0.7, self.zoom * float(factor)),
        )
        self.update()

    def fit(self) -> None:
        self._touch_fit_active = True
        self.pan = QPointF()
        self._apply_touch_fit()
        self.update()

    def fit_overview(self) -> None:
        """Show the complete map, even when a narrow map becomes small."""
        self._touch_fit_active = False
        self.zoom = 1.0
        self.pan = QPointF()
        self.update()

    def _apply_touch_fit(self) -> None:
        base = super()._map_rect()
        if base.isEmpty():
            self.zoom = self.DEFAULT_TOUCH_ZOOM
            return
        short_side = min(base.width(), base.height())
        size_zoom = (
            self.MIN_TOUCH_MAP_SIDE / short_side
            if short_side > 0.0 else self.MAX_TOUCH_FIT_ZOOM)
        self.zoom = min(
            self.MAX_TOUCH_FIT_ZOOM,
            max(self.DEFAULT_TOUCH_ZOOM, size_zoom),
        )

    def _touch_to_world(self, point: QPointF) -> Optional[np.ndarray]:
        world = self.widget_to_world(point)
        if world is not None:
            return world
        target = self._map_rect()
        if target.isEmpty() or not target.adjusted(
            -self.TOUCH_SLOP,
            -self.TOUCH_SLOP,
            self.TOUCH_SLOP,
            self.TOUCH_SLOP,
        ).contains(point):
            return None
        inset = 0.5
        clamped = QPointF(
            min(target.right() - inset, max(target.left() + inset, point.x())),
            min(target.bottom() - inset, max(target.top() + inset, point.y())),
        )
        return self.widget_to_world(clamped)

    def resizeEvent(self, event) -> None:
        if self._touch_fit_active:
            self._apply_touch_fit()
        super().resizeEvent(event)

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        if self.candidate_position is None:
            return
        target = self._map_rect()
        if target.isEmpty():
            return
        start = self.world_to_widget(self.candidate_position)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setClipRect(target)
        painter.setPen(QPen(QColor("#ffe082"), 3.2))
        painter.setBrush(QColor("#ffb300"))
        painter.drawEllipse(start, 11.0, 11.0)
        painter.setFont(QFont("Sans Serif", 11, QFont.DemiBold))
        painter.setPen(QColor("#ffffff"))
        painter.drawText(start + QPointF(15.0, -11.0), "1 位置")
        if self.candidate_direction is None:
            return
        end = self.world_to_widget(self.candidate_direction)
        vector = np.array([end.x() - start.x(), end.y() - start.y()])
        length = float(np.linalg.norm(vector))
        if length <= 1e-6:
            return
        unit = vector / length
        side = np.array([-unit[1], unit[0]])
        painter.setPen(QPen(
            QColor("#ffe082"), 4.0, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(start, end)
        arrow = QPolygonF([
            end,
            QPointF(*(np.array([end.x(), end.y()]) - unit * 18.0 + side * 9.0)),
            QPointF(*(np.array([end.x(), end.y()]) - unit * 18.0 - side * 9.0)),
        ])
        painter.setBrush(QColor("#ffe082"))
        painter.drawPolygon(arrow)
        painter.setPen(QPen(QColor("#ffffff"), 1.5))
        painter.setBrush(QColor("#4ed8ff"))
        painter.drawEllipse(end, 9.0, 9.0)
        painter.setPen(QColor("#ffffff"))
        painter.drawText(end + QPointF(13.0, -10.0), "2 方向")


class ManualRelocalizationPage(QWidget):
    """Select map position first, then a point along the device heading."""

    submit_requested = pyqtSignal()
    navigation_requested = pyqtSignal()
    automatic_requested = pyqtSignal()
    selection_changed = pyqtSignal()
    timeout_changed = pyqtSignal(int)
    export_requested = pyqtSignal()
    back_requested = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.scene_key = None
        self.position_point: Optional[np.ndarray] = None
        self.direction_point: Optional[np.ndarray] = None
        self.minimum_direction_distance = 0.30
        self._cloud_ready = False
        self._cloud_error = ""
        self._operation = "idle"
        self._continue_navigation = False
        self._landscape_layout = None
        self._narrow_layout = None

        layout = QVBoxLayout(self)
        # Leave one physical pixel on the right: QGridLayout can otherwise
        # round a nested half-width row one pixel beyond a 320 px viewport.
        layout.setContentsMargins(0, 0, 1, 0)
        layout.setSpacing(5)
        self.setStyleSheet("""
            QPushButton#relocalizeButton, QPushButton#relocalizePrimary {
                min-height: 46px; max-height: 52px; padding: 0 8px;
                border-radius: 9px; border: 1px solid #425967;
                background: #1b2c38; color: #f1f6f8;
                font-size: 18px; font-weight: 600;
            }
            QPushButton#relocalizePrimary {
                background: #78e8cf; color: #08221e;
                border-color: #78e8cf;
            }
            QPushButton#relocalizeButton:disabled,
            QPushButton#relocalizePrimary:disabled {
                background: #25323b; color: #8a9ca6;
                border-color: #334550;
            }
            QLabel#relocalizeHint {
                color: #d4e0e6; font-size: 18px; font-weight: 600;
            }
            QLabel#relocalizeStatus, QLabel#relocalizeMeta {
                color: #b6c8d3; font-size: 16px;
            }
            QComboBox { min-height: 40px; font-size: 16px; }
        """)
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(6)
        self.back_button = _button("‹ 返回")
        self.back_button.setFixedWidth(102)
        self.back_button.clicked.connect(self.back_requested.emit)
        header.addWidget(self.back_button)
        self.hint = _label(
            "第 1 步：轻点设备当前的大概位置", "relocalizeHint")
        header.addWidget(self.hint, 1)
        layout.addLayout(header)
        body = QWidget(self)
        self.content_layout = QGridLayout(body)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(5)
        layout.addWidget(body, 1)
        self.map = RelocalizationMap(self)
        self.map.point_selected.connect(self.select_world_point)
        self.cloud_status = _label("", "relocalizeMeta")
        self.cloud_status.hide()
        self.status = _label(
            "白色箭头：已确认；黄色标记：待确认先验",
            "relocalizeStatus",
        )
        self.status.setMinimumHeight(40)
        self.message_panel = QWidget(self)
        message_layout = QVBoxLayout(self.message_panel)
        message_layout.setContentsMargins(0, 0, 0, 0)
        message_layout.setSpacing(2)
        message_layout.addWidget(self.cloud_status)
        message_layout.addWidget(self.status)
        self.map_mode = _button("点云地图")
        self.map_mode.clicked.connect(self.toggle_map_mode)
        self.zoom_out = _button("−")
        self.zoom_out.clicked.connect(lambda: self.map.change_zoom(0.8))
        self.zoom_in = _button("＋")
        self.zoom_in.clicked.connect(lambda: self.map.change_zoom(1.25))
        self.fit_map = _button("查看全图")
        self.fit_map.clicked.connect(self.map.fit_overview)
        self.reselect = _button("清除重选")
        self.reselect.clicked.connect(self.clear_selection)
        self.timeout_controls = QWidget(self)
        timeout_layout = QHBoxLayout(self.timeout_controls)
        timeout_layout.setContentsMargins(0, 0, 0, 0)
        timeout_layout.setSpacing(5)
        self.timeout_label = _label("测试时限", "relocalizeMeta")
        self.timeout_label.setWordWrap(False)
        timeout_layout.addWidget(self.timeout_label)
        self.timeout_select = QComboBox(self.timeout_controls)
        for seconds in (60, 120, 300, 600):
            self.timeout_select.addItem(f"{seconds} 秒", seconds)
        self.timeout_select.setCurrentIndex(1)
        self.timeout_select.setMinimumHeight(40)
        self.timeout_select.setToolTip("仅本次软件运行有效，重启后恢复 120 秒")
        self.timeout_select.currentIndexChanged.connect(
            lambda _index: self.timeout_changed.emit(self.timeout_seconds))
        timeout_layout.addWidget(self.timeout_select, 1)
        self.export_diagnostic = _button("导出诊断")
        self.export_diagnostic.clicked.connect(self.export_requested.emit)
        timeout_layout.addWidget(self.export_diagnostic, 1)
        self.automatic = _button("自动全局匹配")
        self.automatic.clicked.connect(self.automatic_requested.emit)
        self.submit = _button("使用两点重定位", True)
        self.submit.clicked.connect(self._primary_action)
        self.submit.setEnabled(False)
        self._apply_responsive_layout(False)
        self.set_cloud_loading(False)

    def _apply_responsive_layout(self, landscape: bool) -> None:
        landscape = bool(landscape)
        narrow = self.width() < 420
        if (
            self._landscape_layout == landscape
            and self._narrow_layout == narrow
        ):
            return
        self._landscape_layout = landscape
        self._narrow_layout = narrow
        self.back_button.setFixedWidth(84 if narrow else 102)
        self.export_diagnostic.setText("诊断" if narrow else "导出诊断")
        self.export_diagnostic.setMaximumWidth(96 if narrow else 16777215)
        self.timeout_label.setVisible(not narrow)
        widgets = (
            self.map, self.message_panel, self.map_mode,
            self.zoom_out, self.zoom_in, self.fit_map, self.reselect,
            self.timeout_controls, self.automatic, self.submit,
        )
        for widget in widgets:
            self.content_layout.removeWidget(widget)
        for index in range(5):
            self.content_layout.setColumnStretch(index, 0)
            self.content_layout.setColumnMinimumWidth(index, 0)
        for index in range(8):
            self.content_layout.setRowStretch(index, 0)
        if landscape:
            self.timeout_label.setText("时限")
            self.content_layout.addWidget(self.map, 0, 0, 1, 5)
            self.content_layout.addWidget(self.message_panel, 1, 0, 1, 5)
            self.content_layout.addWidget(self.map_mode, 2, 0)
            self.content_layout.addWidget(self.zoom_out, 2, 1)
            self.content_layout.addWidget(self.zoom_in, 2, 2)
            self.content_layout.addWidget(self.fit_map, 2, 3)
            self.content_layout.addWidget(self.reselect, 2, 4)
            self.content_layout.addWidget(self.timeout_controls, 3, 0, 1, 2)
            self.content_layout.addWidget(self.automatic, 3, 2)
            self.content_layout.addWidget(self.submit, 3, 3, 1, 2)
            for index in range(5):
                self.content_layout.setColumnStretch(index, 1)
            self.content_layout.setRowStretch(0, 1)
        else:
            self.timeout_label.setText("测试时限")
            self.content_layout.addWidget(self.map, 0, 0, 1, 5)
            self.content_layout.addWidget(self.message_panel, 1, 0, 1, 5)
            self.content_layout.addWidget(self.timeout_controls, 2, 0, 1, 5)
            self.content_layout.addWidget(self.map_mode, 3, 0)
            self.content_layout.addWidget(self.zoom_out, 3, 1)
            self.content_layout.addWidget(self.zoom_in, 3, 2)
            self.content_layout.addWidget(self.fit_map, 3, 3)
            self.content_layout.addWidget(self.reselect, 3, 4)
            self.content_layout.addWidget(self.automatic, 4, 0, 1, 2)
            self.content_layout.addWidget(self.submit, 4, 2, 1, 3)
            for index in range(5):
                self.content_layout.setColumnStretch(index, 1)
            self.content_layout.setRowStretch(0, 1)

    def resizeEvent(self, event) -> None:
        self._apply_responsive_layout(
            event.size().width() >= 640
            and event.size().width() > event.size().height() * 1.2)
        super().resizeEvent(event)

    @property
    def has_complete_selection(self) -> bool:
        return self.position_point is not None and self.direction_point is not None

    @property
    def cloud_ready(self) -> bool:
        return self._cloud_ready

    @property
    def timeout_seconds(self) -> int:
        return int(self.timeout_select.currentData())

    @property
    def candidate_yaw(self) -> Optional[float]:
        if not self.has_complete_selection:
            return None
        return initial_pose_from_points(
            self.position_point,
            self.direction_point,
            self.minimum_direction_distance,
        )[2]

    def bind_scene(self, key, title: str, grid) -> None:
        changed = key != self.scene_key
        self.scene_key = key
        self.minimum_direction_distance = max(
            0.30, 2.0 * float(grid.resolution))
        self.map.set_map(
            grid.occupancy,
            grid.origin[:2],
            grid.resolution,
            float(grid.origin[2]),
        )
        if changed:
            self.clear_selection(force=True)
            self.map.set_pointcloud(None)
            self._cloud_ready = False
            self._cloud_error = ""
            self.cloud_status.clear()
            self.cloud_status.hide()
            self.map.set_display_mode("grid")
            self.map.fit()
            self.status.setText(
                f"{title} · 白色箭头为已确认位置，黄色标记为人工先验")
        self._refresh_mode_button()

    def clear_scene(self) -> None:
        self.scene_key = None
        self.clear_selection(force=True)
        self.map.set_map(None, np.zeros(2), 0.0, 0.0)
        self.map.set_pointcloud(None)
        self.map.set_navigation_state(None, None, 0.0)
        self._cloud_ready = False
        self._cloud_error = ""
        self.cloud_status.clear()
        self.cloud_status.hide()
        self.map.set_display_mode("grid")
        self._refresh_mode_button()

    def set_pointcloud(self, cloud) -> None:
        self.map.set_pointcloud(
            cloud.points,
            cloud.colors,
            color_source=cloud.color_source,
        )
        self._cloud_ready = True
        self._cloud_error = ""
        self.cloud_status.clear()
        self.cloud_status.hide()
        self._refresh_mode_button()

    def set_cloud_loading(self, loading: bool, error: str = "") -> None:
        if error:
            self._cloud_ready = False
            self._cloud_error = str(error)
            if self.map.display_mode == "cloud":
                self.map.set_display_mode("grid")
            self.cloud_status.setText(
                f"点云地图读取失败：{error}；可继续使用栅格选点")
            self.cloud_status.show()
        elif loading:
            self._cloud_error = ""
            self.cloud_status.setText("正在读取点云地图…")
            self.cloud_status.show()
        else:
            self.cloud_status.clear()
            self.cloud_status.hide()
        self.map_mode.setEnabled(self._cloud_ready and not loading)
        if loading:
            self.map_mode.setText("点云加载中…")
        else:
            self._refresh_mode_button()

    def _refresh_mode_button(self) -> None:
        if not hasattr(self, "map_mode"):
            return
        self.map_mode.setText(
            "栅格地图" if self.map.display_mode == "cloud" else "点云地图")
        self.map_mode.setEnabled(self._cloud_ready)

    def toggle_map_mode(self) -> None:
        if self.map.display_mode == "grid":
            if not self._cloud_ready:
                return
            self.map.set_display_mode("cloud")
        else:
            self.map.set_display_mode("grid")
        self.map.fit()
        self._refresh_mode_button()

    def select_world_point(self, x: float, y: float) -> None:
        if self._operation == "pending":
            return
        point = np.array([float(x), float(y)], dtype=float)
        if not np.all(np.isfinite(point)):
            self.status.setText("选点无效，请在地图范围内重试")
            return
        if self.position_point is None:
            self.position_point = point
            self.direction_point = None
            self.hint.setText("第 2 步：轻点设备正前方，用连线表示朝向")
        elif self.direction_point is None:
            try:
                initial_pose_from_points(
                    self.position_point,
                    point,
                    self.minimum_direction_distance,
                )
            except ValueError as exc:
                self.status.setText(str(exc))
                return
            self.direction_point = point
            yaw = self.candidate_yaw
            self.hint.setText(
                f"已选大概位置和方向（{math.degrees(yaw):.0f}°），确认后开始点云匹配")
        else:
            self.status.setText("如需修改，请先点“清除重选”")
            return
        self.map.set_candidate(self.position_point, self.direction_point)
        self.selection_changed.emit()

    def clear_selection(self, force: bool = False) -> None:
        if self._operation == "pending" and not force:
            return
        self.position_point = None
        self.direction_point = None
        self._operation = "idle"
        self.map.set_candidate(None, None)
        self.map.set_selection_enabled(True)
        self.hint.setText("第 1 步：轻点设备当前的大概位置")
        if hasattr(self, "submit"):
            self.submit.setEnabled(False)
        self.selection_changed.emit()

    def reset_operation_state(self) -> None:
        """Forget the result label when a new stack/request generation starts."""
        if self._operation != "pending":
            self._operation = "idle"

    def set_current_pose(self, xy, yaw: float) -> None:
        self.map.set_navigation_state(None, xy, yaw)

    def _primary_action(self):
        if self._continue_navigation:
            self.navigation_requested.emit()
        else:
            self.submit_requested.emit()

    def set_runtime(
        self,
        submit_ready: bool,
        readiness_reason: str,
        global_pending: bool,
        manual_pending: bool,
        automatic_ready: bool,
        current_pose_available: bool,
        manual_state: str,
        progress_text: str = "",
        progress_metrics: str = "",
        elapsed_s: float = 0.0,
        timeout_s: int = 120,
        navigation_ready: bool = False,
    ) -> None:
        pending = bool(global_pending or manual_pending)
        elapsed_display = min(max(0.0, elapsed_s), float(timeout_s))
        timing = f"{elapsed_display:.0f}/{timeout_s} 秒"
        if elapsed_s > timeout_s:
            timing += "（等待终态，最多 10 秒）"
        if manual_pending:
            self._operation = "pending"
            detail = f"{progress_text or '正在用所选位置和方向匹配点云'} · {timing}"
            if progress_metrics:
                detail += f"\n{progress_metrics}"
            self.status.setText(detail)
        elif global_pending:
            detail = f"{progress_text or '正在自动搜索整张地图'} · {timing}"
            if progress_metrics:
                detail += f"\n{progress_metrics}"
            self.status.setText(detail)
        elif (manual_state == "matched" or self._operation == "matched") and current_pose_available:
            self._operation = "matched"
            self.status.setText("重定位成功；白色箭头为当前地图位置")
        elif manual_state == "matched" or self._operation == "matched":
            self._operation = "matched"
            self.status.setText("点云匹配通过，正在连续复核当前位置")
        elif manual_state == "error":
            if self._operation != "error":
                self.status.setText("上次两点重定位未完成，请重新选择并提交")
            self._operation = "error"
        elif self._operation == "error":
            pass
        elif current_pose_available:
            self.status.setText("白色箭头为当前已确认的地图位置，可重新选择先验")
        elif not self.has_complete_selection:
            self.status.setText("当前地图位置尚未确认；请依次选择位置和方向")
        else:
            self.status.setText(readiness_reason)
        self.map.set_selection_enabled(not pending)
        self.timeout_select.setEnabled(not pending)
        self.reselect.setEnabled(not pending and self.position_point is not None)
        self.automatic.setEnabled(bool(automatic_ready) and not pending)
        self._continue_navigation = bool(navigation_ready and current_pose_available
                                         and self.position_point is None and not pending)
        self.submit.setText("选择目的地" if self._continue_navigation else "使用两点重定位")
        self.submit.setEnabled(self._continue_navigation or (
            self.has_complete_selection and submit_ready and not pending))
        if self.position_point is None:
            self.hint.setText("定位已确认，可选择目的地" if self._continue_navigation
                              else "第 1 步：轻点设备当前的大概位置")

    def set_operation_error(self, message: str) -> None:
        self._operation = "error"
        self.status.setText(str(message))

    def set_operation_matched(self) -> None:
        self._operation = "matched"
        self.status.setText("点云匹配通过，正在连续复核当前位置")

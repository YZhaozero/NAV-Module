"""Mapping settings and the shared mapping workflow.

This module only renders state and forwards user actions to
``MappingController``. ROS messages, launch commands and service calls stay in
the feature layer.
"""

from __future__ import annotations

import re
import time
from pathlib import Path

import numpy as np
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QBoxLayout,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .page_header import PageHeader
from ..features.mapping import MAPPING_BACKENDS, MappingController
from ..map_storage import default_map_directory


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class MappingPage(QWidget):
    """Mapping parameters; runtime widgets are hosted by MappingWorkspace."""

    return_requested = pyqtSignal()
    navigation_requested = pyqtSignal(str, str, str)
    progress_requested = pyqtSignal()

    def __init__(
        self, controller: MappingController, map_panel_factory,
        map_storage_dir: Path | None = None,
    ) -> None:
        super().__init__()
        self.controller = controller
        self.map_storage_dir = default_map_directory(str(
            map_storage_dir or ""))
        self.process_state = "stopped"
        self.last_cloud_revision = controller.ros.cloud_revision
        self.last_save_revision = controller.ros.save_revision
        self._map_panel_factory = map_panel_factory
        self._finishing_grid_method = "observed"
        self._finishing_attempt = None
        self._navigation_source = None
        self._last_cloud_refresh = 0.
        self.navigation_job_id = None
        self._build_ui()
        self.controller.log_received.connect(self._append_log)
        self.controller.state_changed.connect(self._handle_state)
        self._backend_changed()
        self._handle_state("stopped")

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        header = PageHeader("NAV · 建图参数", self.return_requested.emit,
            destination="返回建图", subtitle="算法、传感器输入与地图保存设置")
        self.header = header
        refresh_button = QPushButton("刷新输入话题")
        refresh_button.setObjectName("secondaryButton")
        refresh_button.clicked.connect(self.refresh_topics)
        header.add_action(refresh_button)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        self.splitter = splitter
        map_card = QFrame()
        self.map_card = map_card
        map_card.setObjectName("sidePanel")
        map_layout = QVBoxLayout(map_card)
        map_layout.setContentsMargins(14, 14, 14, 14)
        reset_view = QPushButton("重置视角")
        reset_view.setObjectName("secondaryButton")
        self.reset_view = reset_view
        appearance = QHBoxLayout()
        appearance.addWidget(QLabel("点大小"))
        self.point_size_spin = QDoubleSpinBox()
        self.point_size_spin.setRange(1, 5)
        self.point_size_spin.setDecimals(0)
        self.point_size_spin.setValue(2)
        self.point_size_spin.setSuffix(" px")
        appearance.addWidget(self.point_size_spin)
        self.color_combo = QComboBox()
        self.color_combo.addItem("高度伪彩色", "height")
        self.color_combo.addItem("强度伪彩色", "intensity")
        self.color_combo.addItem("原始 RGB / 强度", "rgb")
        appearance.addWidget(self.color_combo)
        appearance.addStretch(1)
        self.map_panel = self._map_panel_factory(cloud_3d=True)
        self.map_panel.setObjectName("mappingMapPanel")
        self.map_panel.set_display_mode("cloud")
        self.map_panel.set_cloud_display_options(
            "original", "XYZ", (False, False, False), "orthographic", "height")
        reset_view.clicked.connect(self.map_panel.reset_cloud_view)
        self.point_size_spin.valueChanged.connect(self.map_panel.set_cloud_point_size)
        self.color_combo.currentIndexChanged.connect(lambda: self.map_panel.set_cloud_display_options(
            "original", "XYZ", (False, False, False), "orthographic", self.color_combo.currentData()))
        map_layout.addWidget(self.map_panel, 1)
        self.cloud_status = QLabel(
            "尚未开始建图 · 等待 PointCloud2 地图输出")
        self.cloud_status.setObjectName("statusBar")
        map_layout.addWidget(self.cloud_status)
        controls = QFrame()
        controls.setObjectName("sidePanel")
        self.controls = controls
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(16, 16, 16, 16)
        controls_layout.setSpacing(10)
        self.topic_status = QLabel()
        self.topic_status.setObjectName("hint")
        self.topic_status.setWordWrap(True)
        controls_layout.addWidget(self.topic_status)
        backend_title = QLabel("建图算法")
        backend_title.setObjectName("sectionTitle")
        controls_layout.addWidget(backend_title)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.algorithm_combo = QComboBox()
        for key, backend in MAPPING_BACKENDS.items():
            self.algorithm_combo.addItem(backend.label, key)
        self.algorithm_combo.currentIndexChanged.connect(
            self._backend_changed)
        form.addRow("算法", self.algorithm_combo)
        self.pointcloud_combo = self._editable_combo(
            "选择或输入 PointCloud2 话题")
        self.imu_combo = self._editable_combo("选择或输入 IMU 话题")
        self.map_topic = QLineEdit()
        form.addRow("雷达话题", self.pointcloud_combo)
        form.addRow("IMU", self.imu_combo)
        form.addRow("地图输出", self.map_topic)
        self.image_topic = QLineEdit("/camera/camera/color/image_raw")
        self.params_file = QLineEdit()
        self.camera_params_file = QLineEdit()
        self.params_file.setPlaceholderText("留空使用 MID-360 默认配置")
        self.camera_params_file.setPlaceholderText("留空使用包内相机配置；实机需核对标定")
        self.vision_enabled = QCheckBox("启用视觉融合")
        self.vision_enabled.setToolTip("启用前核对本机相机内外参、分辨率和时间补偿")
        form.addRow("视觉", self.vision_enabled)
        form.addRow("相机话题", self.image_topic)
        form.addRow("LIVO 配置", self.params_file)
        form.addRow("相机标定", self.camera_params_file)
        controls_layout.addLayout(form)
        self.backend_description = QLabel()
        self.backend_description.setObjectName("hint")
        self.backend_description.setWordWrap(True)
        controls_layout.addWidget(self.backend_description)

        options_row = QHBoxLayout()
        self.use_sim_time = QCheckBox("使用仿真时钟")
        self.open_rviz = QCheckBox("同时启动 RViz")
        options_row.addWidget(self.use_sim_time)
        options_row.addWidget(self.open_rviz)
        options_row.addStretch(1)
        controls_layout.addLayout(options_row)

        action_row = QHBoxLayout()
        self.runtime_actions = action_row
        self.state_label = QLabel("● 未启动")
        self.state_label.setObjectName("sensorStatus")
        action_row.addWidget(self.state_label)
        action_row.addStretch(1)
        self.stop_button = QPushButton("停止（不处理）")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.clicked.connect(self.controller.stop)
        self.start_button = QPushButton("开始建图")
        self.start_button.setObjectName("primaryButton")
        self.start_button.clicked.connect(self.start_mapping)
        action_row.addWidget(self.stop_button)
        action_row.addWidget(self.start_button)

        save_title = QLabel("地图保存设置")
        save_title.setObjectName("sectionTitle")
        controls_layout.addWidget(save_title)
        save_form = QFormLayout()
        save_form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.map_name = QLineEdit("nav_map")
        self.map_name.setPlaceholderText("例如：factory_floor_1")
        save_form.addRow("地图名称", self.map_name)
        save_row = QWidget()
        save_row_layout = QHBoxLayout(save_row)
        save_row_layout.setContentsMargins(0, 0, 0, 0)
        self.save_path = QLineEdit(str(self.map_storage_dir))
        choose_save_path = QPushButton("选择")
        choose_save_path.setObjectName("secondaryButton")
        choose_save_path.clicked.connect(self._choose_save_path)
        save_row_layout.addWidget(self.save_path, 1)
        save_row_layout.addWidget(choose_save_path)
        save_form.addRow("保存目录", save_row)
        self.leaf_size = QDoubleSpinBox()
        self.leaf_size.setRange(0.01, 2.0)
        self.leaf_size.setDecimals(2)
        self.leaf_size.setSingleStep(0.05)
        self.leaf_size.setValue(0.05)
        self.leaf_size.setSuffix(" m")
        save_form.addRow("降采样体素", self.leaf_size)
        self.grid_method_combo = QComboBox()
        self.grid_method_combo.addItem("原有地面投影", "observed")
        self.grid_method_combo.addItem("局部高程通行性（grid_map_pcl）", "terrain")
        self.grid_method_combo.setToolTip("保存并结束后，用所选方法生成栅格；SC 索引仍按原流程生成")
        save_form.addRow("栅格方法", self.grid_method_combo)
        controls_layout.addLayout(save_form)
        controls_layout.addLayout(appearance)
        self.save_button = QPushButton("保存当前 PCD")
        self.save_button.setObjectName("primaryButton")
        self.save_button.clicked.connect(self.save_mapping_map)
        self.finish_button = QPushButton("保存并结束")
        self.finish_button.setObjectName("primaryButton")
        self.finish_button.clicked.connect(self.finish_mapping)
        self.finish_button.setToolTip("保存 PCD、结束建图，自动生成栅格地图和 SC 重定位索引")
        self.navigation_status = QLabel("结束后自动识别地面，生成栅格地图和 SC 重定位索引")
        self.navigation_status.setWordWrap(True)
        self.navigation_status.setObjectName("hint")
        result_row = QHBoxLayout()
        result_row.addWidget(self.navigation_status, 1)
        progress_button = QPushButton("处理进度 / 地图包")
        progress_button.setObjectName("secondaryButton")
        progress_button.clicked.connect(self.progress_requested.emit)
        result_row.addWidget(progress_button)
        map_layout.addLayout(result_row)
        self.save_status = QLabel()
        self.save_status.setObjectName("hint")
        self.save_status.setWordWrap(True)
        self.recover_button = QPushButton("恢复中断建图")
        self.recover_button.setObjectName("secondaryButton")
        self.recover_button.clicked.connect(self.recover_mapping_map)
        controls_layout.addWidget(self.recover_button)

        log_header = QHBoxLayout()
        log_title = QLabel("建图日志")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        clear_log = QPushButton("清空")
        clear_log.setObjectName("secondaryButton")
        clear_log.clicked.connect(self.log_view_clear)
        log_header.addWidget(clear_log)
        controls_layout.addLayout(log_header)
        self.log_view = QPlainTextEdit()
        self.log_view.setObjectName("sensorLog")
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.log_view.document().setMaximumBlockCount(3000)
        controls_layout.addWidget(self.log_view, 1)
        controls_scroll = QScrollArea()
        controls_scroll.setObjectName("transparentScroll")
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setWidget(controls)
        splitter.addWidget(controls_scroll)
        layout.addWidget(splitter, 1)

    def set_compact_mode(
        self, compact: bool, portrait: bool | None = None,
    ) -> None:
        """Use a vertically scrollable layout on narrow portrait screens."""
        compact = bool(compact)
        self.layout().setContentsMargins(
            *(8, 6, 8, 8) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(7 if compact else 14)
        self.header.setDirection(QBoxLayout.LeftToRight)
        for label in self.findChildren(QLabel):
            if label.objectName() == "subtitle":
                label.setVisible(not compact)

    @staticmethod
    def _editable_combo(placeholder: str) -> QComboBox:
        combo = QComboBox()
        combo.setEditable(True)
        combo.setInsertPolicy(QComboBox.NoInsert)
        combo.lineEdit().setPlaceholderText(placeholder)
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return combo

    def activate(self) -> None:
        self.refresh_topics()

    def _backend_changed(self, *_args) -> None:
        key = str(self.algorithm_combo.currentData() or "dlio")
        try:
            backend = self.controller.select_backend(key)
        except RuntimeError as exc:
            self.cloud_status.setText(str(exc))
            return
        self.backend_description.setText(
            backend.description)
        fast_livo = backend.save_adapter == "fast_livo_save_map"
        for widget in (self.image_topic, self.params_file, self.camera_params_file, self.vision_enabled):
            widget.setEnabled(fast_livo)
        if key in ("woosh_fast_livo", "woosh_native5"):
            self.image_topic.setText("/foo/camera_1/color/image_raw")
            self.vision_enabled.setChecked(True)
        self.leaf_size.setMinimum(0.05 if fast_livo else 0.01)
        self.pointcloud_combo.setCurrentText(
            backend.default_pointcloud_topic)
        self.imu_combo.setCurrentText(backend.default_imu_topic)
        self.map_topic.setText(backend.default_map_topic)
        self.save_status.setText(
            f"地图保存服务：{backend.save_service or '未配置'}；"
            "文件名会自动附加时间，不会覆盖旧地图")

    def refresh_topics(self) -> None:
        try:
            grouped = self.controller.available_topics()
        except (RuntimeError, TypeError, ValueError) as exc:
            self.topic_status.setText(f"读取建图输入话题失败：{exc}")
            return
        backend = self.controller.backend
        selectors = (
            (self.pointcloud_combo, grouped.get("custom" if backend.pointcloud_type == "livox_ros_driver2/msg/CustomMsg" else "pointcloud", []),
             backend.default_pointcloud_topic),
            (self.imu_combo, grouped.get("imu", []),
             backend.default_imu_topic),
        )
        for combo, discovered_topics, default in selectors:
            current = combo.currentText().strip()
            candidates = list(discovered_topics)
            for candidate in (current, default):
                if candidate and candidate not in candidates:
                    candidates.append(candidate)
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(sorted(candidates))
            combo.setCurrentText(current or default)
            combo.blockSignals(False)
        self.topic_status.setText(
            f"发现 {len(grouped.get('pointcloud', []))} 个 PointCloud2、"
            f"{len(grouped.get('imu', []))} 个 IMU 话题")

    def launch_arguments(self):
        return self.controller.build_launch_arguments(
            self.pointcloud_combo.currentText(),
            self.imu_combo.currentText(),
            self.open_rviz.isChecked(),
            self.use_sim_time.isChecked(),
            self.image_topic.text(), self.params_file.text(), self.camera_params_file.text(),
            vision_enabled=self.vision_enabled.isChecked(),
        )

    def start_mapping(self) -> None:
        self.map_panel.set_pointcloud(np.empty((0, 3), dtype=np.float32))
        self.map_panel.set_display_mode("cloud")
        success, status = self.controller.start(
            self.pointcloud_combo.currentText(),
            self.imu_combo.currentText(),
            self.map_topic.text(),
            self.open_rviz.isChecked(),
            self.use_sim_time.isChecked(),
            self.image_topic.text(), self.params_file.text(), self.camera_params_file.text(),
            vision_enabled=self.vision_enabled.isChecked(),
        )
        self.cloud_status.setText(status)
        if not success:
            self._append_log(status + "\n")

        return success, status

    def recover_mapping_map(self) -> None:
        checkpoint = QFileDialog.getExistingDirectory(
            self, "选择中断建图的 capture-* 检查点目录",
            str(Path.home() / ".local/state/nav/map-spool"))
        if checkpoint:
            success, status = self.controller.recover_map(checkpoint, self.save_path.text())
            self.save_status.setText(status)
            if not success:
                self._append_log(status + "\n")

    def save_mapping_map(self) -> None:
        success, status = self.controller.save_map(
            self.save_path.text().strip(),
            self.map_name.text().strip(),
            self.leaf_size.value(),
        )
        self.save_status.setText(status)
        if success:
            self.save_button.setEnabled(False)
        else:
            self._append_log(status + "\n")

    def finish_mapping(self) -> None:
        success, status = self.controller.save_map(
            self.save_path.text().strip(), self.map_name.text().strip(), self.leaf_size.value())
        self.save_status.setText(status)
        if success:
            self._finishing_grid_method = self.grid_method_combo.currentData()
            self._finishing_attempt = self.controller.ros.save_attempt_id
            self.navigation_status.setText("正在保存 PCD；保存成功后结束建图并生成导航地图")
            self.finish_button.setEnabled(False)
            self.save_button.setEnabled(False)
            self.stop_button.setEnabled(False)
        else:
            self._append_log(status + "\n")

    def refresh(self) -> None:
        ros = self.controller.ros
        if ros.cloud_error:
            self.cloud_status.setText(ros.cloud_error)
        if (self.map_panel.isVisible() and ros.cloud_revision != self.last_cloud_revision
                and time.monotonic() - self._last_cloud_refresh >= .2):
            self._last_cloud_refresh = time.monotonic()
            self.last_cloud_revision = ros.cloud_revision
            if ros.cloud is not None:
                self.map_panel.set_pointcloud(
                    ros.cloud, colors=ros.cloud_colors, color_source=ros.cloud_color_source)
                self.color_combo.model().item(self.color_combo.findData("intensity")).setEnabled(
                    ros.cloud_color_source == "intensity")
                self.map_panel.set_display_mode("cloud")
                self.cloud_status.setText(
                    f"实时地图 {ros.cloud_topic} · {len(ros.cloud):,} 点 · "
                    f"frame={ros.cloud_frame or '(empty)'}")
        if ros.save_revision != self.last_save_revision:
            self.last_save_revision = ros.save_revision
            if ros.save_status:
                self.save_status.setText(ros.save_status)
                self._append_log(ros.save_status + "\n")
            if self._finishing_attempt is not None:
                result = ros.last_save_result
                if result['attempt_id'] == self._finishing_attempt:
                    if result['success']:
                        self._navigation_source = (result['path'], self.map_name.text().strip(), self._finishing_grid_method)
                        self.navigation_status.setText("PCD 已保存，正在结束建图…")
                        self.controller.stop()
                    else:
                        self._finishing_attempt = None
                        self.navigation_status.setText("PCD 保存失败；请查看保存提示后重试")
            ready = self.process_state == "running" and self._finishing_attempt is None
            self.save_button.setEnabled(ready)
            self.finish_button.setEnabled(ready)
            self.stop_button.setEnabled(ready)
        if self._navigation_source is not None and not self.controller.process.active:
            source, title, method = self._navigation_source
            self._navigation_source = None
            self._finishing_attempt = None
            self.navigation_requested.emit(source, title, method)

    def _handle_state(self, state: str) -> None:
        self.recover_button.setEnabled(state in ("stopped", "error"))
        self.process_state = state
        labels = {
            "starting": ("启动中…", "#ffd166"),
            "running": ("建图中", "#66e09a"),
            "stopping": ("停止中…", "#ffd166"),
            "stopped": ("未启动", "#91a2ad"),
            "error": ("启动失败", "#ff7f88"),
        }
        text, color = labels.get(state, (state, "#91a2ad"))
        self.state_label.setText(f"● {text}")
        self.state_label.setStyleSheet(
            f"color: {color}; font-weight: 700;")
        active = state in ("starting", "running", "stopping")
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(state in ("starting", "running") and self._finishing_attempt is None)
        self.save_button.setEnabled(
            state == "running" and not self.controller.ros.save_in_progress)
        self.finish_button.setEnabled(
            state == "running" and not self.controller.ros.save_in_progress
            and self._finishing_attempt is None)
        self.algorithm_combo.setEnabled(not active)
        if state in ("stopped", "error") and self.controller.ros.cloud is not None:
            self.cloud_status.setText(
                "建图已停止 · 保留最后一帧地图，可继续查看")

    def _append_log(self, text: str) -> None:
        clean = ANSI_ESCAPE_RE.sub("", text).replace("\r", "").rstrip("\n")
        if clean:
            self.log_view.appendPlainText(clean)

    def log_view_clear(self) -> None:
        self.log_view.clear()

    def _choose_save_path(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self, "选择地图保存目录", self.save_path.text())
        if selected:
            self.save_path.setText(selected)

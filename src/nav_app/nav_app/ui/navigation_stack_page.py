"""Qt page for configuring and launching the complete navigation stack."""

from __future__ import annotations

import re
from collections import deque
from pathlib import Path
from typing import Callable

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QBoxLayout,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .page_header import PageHeader
from ..features.navigation_stack import (
    COMPONENT_LABELS,
    LOCALIZER_TUNING_DEFAULTS,
    START_ORDER,
    NavigationStackConfig,
    NavigationStackController,
)
from ..features.navigation_backends import (
    LOCALIZATION_BACKENDS,
    NAVIGATION_BACKENDS,
)
from ..features.localization_tools import (
    DEFAULT_GLOBAL_RELOCALIZE_SERVICE,
    LocalizationToolsController,
)


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def latest_map(directory: Path, suffixes) -> str:
    candidates = []
    for suffix in suffixes:
        candidates.extend(directory.glob(f"*{suffix}"))
    files = [path for path in candidates if path.is_file()]
    return str(max(files, key=lambda path: path.stat().st_mtime)) if files else ""


class NavigationStackPage(QWidget):
    """Parameter editor, ordered launcher, process status and combined logs."""

    return_requested = pyqtSignal()
    access_requested = pyqtSignal()
    localization_map_selected = pyqtSignal(str)

    def __init__(
        self,
        controller: NavigationStackController,
        map_storage_dir: Path,
        lidar_arguments: Callable[[], list],
        camera_arguments: Callable[[], list],
        localization_tools: LocalizationToolsController,
        robot_profile: str = "",
        operating_mode: str = "robot",
        woosh_control_mode: int = 0,
    ) -> None:
        super().__init__()
        self.controller = controller
        self.robot_profile = robot_profile
        self.operating_mode = operating_mode
        self.woosh_control_mode = woosh_control_mode if operating_mode == "robot" else 0
        self.map_storage_dir = Path(map_storage_dir)
        self.lidar_arguments = lidar_arguments
        self.camera_arguments = camera_arguments
        self.localization_tools = localization_tools
        self.component_status = {}
        self.access_settings = {}
        self.meshnav_scene = None
        self._build_ui()
        if operating_mode == "walking":
            self.navigation_backend.setCurrentIndex(self.navigation_backend.findData("walking"))
        if robot_profile == "woosh":
            self.dlio_pointcloud_topic.setText("/foo/livox/lidar_192_168_128_3")
            self.dlio_imu_topic.setText("/foo/livox/imu_192_168_128_3")
            self.odometry_backend.setCurrentIndex(self.odometry_backend.findData("fast_livo"))
            self.fast_livo_lidar_topic.setText("/foo/livox/lidar_192_168_128_3")
            self.fast_livo_image_topic.setText("/foo/camera_1/color/image_raw")
            self.localizer_cloud_topic.setText("/fast_livo/cloud_body")
            self.localizer_odom_topic.setText("/aft_mapped_to_init")
            self.scan_topic.setText("/foo/scan")
            self.laser_target_frame.setText("base_footprint")
            self.navigation_map.clear()
            self.localization_map.clear()
            self.action_status.setText("WOOSH：选择地图包后进入导航即启动 FAST-LIVO2、定位和 Nav2；底盘执行未启用")

        self._pending_logs = {key: deque(maxlen=3000) for key in self.log_views}
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(150)
        self._log_timer.timeout.connect(self._flush_logs)
        self._log_timer.start()
        self.log_tabs.currentChanged.connect(lambda _: self._flush_logs(force=True))
        self.controller.log_received.connect(self._append_log)
        self.controller.input_waiting.connect(self.action_status.setText)
        self.controller.component_state_changed.connect(
            self._handle_component_state)
        self.controller.overall_state_changed.connect(
            self._handle_overall_state)
        self.localization_tools.log_received.connect(
            lambda text: self.controller.record_external_log(
                "localizer", text))
        self.localization_tools.operation_state_changed.connect(
            self._handle_localization_operation_state)
        self.localization_tools.status_changed.connect(
            self._handle_localization_tool_status)
        self._handle_overall_state("stopped")
        for key in START_ORDER:
            self._handle_component_state(key, "stopped")

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        header = PageHeader("NAV · 导航系统", self.return_requested.emit,
            destination="返回系统首页", subtitle="传感器 · 里程计 · 定位后端 · 导航后端顺序启动")
        self.header = header
        self.access_button = QPushButton("导航接入设置")
        self.access_button.clicked.connect(self.access_requested.emit)
        header.add_action(self.access_button)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        self.splitter = splitter
        controls = QFrame()
        controls.setObjectName("sidePanel")
        controls.setMinimumWidth(560)
        self.controls = controls
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(16, 16, 16, 16)
        controls_layout.setSpacing(10)
        parameters_title = QLabel("启动参数")
        parameters_title.setObjectName("sectionTitle")
        controls_layout.addWidget(parameters_title)
        self.parameter_tabs = QTabWidget()
        self.parameter_tabs.addTab(self._build_localization_tab(), "定位")
        self.parameter_tabs.addTab(self._build_navigation_tab(), "导航")
        self.parameter_tabs.addTab(self._scrollable(self._build_dlio_tab()), "里程计")
        self.parameter_tabs.addTab(self._build_laserscan_tab(), "LaserScan")
        self.parameter_tabs.addTab(self._build_general_tab(), "通用")
        controls_layout.addWidget(self.parameter_tabs, 1)

        sensor_hint = QLabel(
            "只按接入设置启动需要的模块；已有话题检查实际数据与 TF，停止时不接管外部驱动。")
        sensor_hint.setObjectName("hint")
        sensor_hint.setWordWrap(True)
        controls_layout.addWidget(sensor_hint)
        action_row = QHBoxLayout()
        self.overall_status = QLabel("● 未启动")
        self.overall_status.setObjectName("sensorStatus")
        action_row.addWidget(self.overall_status)
        action_row.addStretch(1)
        self.stop_button = QPushButton("停止整套系统")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.clicked.connect(self.controller.stop)
        self.start_button = QPushButton("启动 / 重试")
        self.start_button.setObjectName("primaryButton")
        self.start_button.clicked.connect(self.start_stack)
        action_row.addWidget(self.stop_button)
        action_row.addWidget(self.start_button)
        controls_layout.addLayout(action_row)
        self.action_status = QLabel("请选择地图、定位后端和导航后端")
        self.action_status.setObjectName("statusBar")
        self.action_status.setWordWrap(True)
        controls_layout.addWidget(self.action_status)
        splitter.addWidget(controls)

        monitor = QFrame()
        monitor.setObjectName("sidePanel")
        monitor_layout = QVBoxLayout(monitor)
        monitor_layout.setContentsMargins(16, 16, 16, 16)
        status_title = QLabel("启动顺序与状态")
        status_title.setObjectName("sectionTitle")
        monitor_layout.addWidget(status_title)
        self.component_names = {}
        for index, key in enumerate(START_ORDER, start=1):
            row = QHBoxLayout()
            name = QLabel(f"{index}. {COMPONENT_LABELS[key]}")
            self.component_names[key] = name
            row.addWidget(name)
            row.addStretch(1)
            status = QLabel("● 未启动")
            status.setObjectName("sensorStatus")
            self.component_status[key] = status
            row.addWidget(status)
            monitor_layout.addLayout(row)
        log_header = QHBoxLayout()
        log_title = QLabel("启动日志")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        clear = QPushButton("清空")
        clear.setObjectName("secondaryButton")
        clear.clicked.connect(self._clear_logs)
        log_header.addWidget(clear)
        monitor_layout.addLayout(log_header)
        self.log_views = {}
        self.log_tabs = QTabWidget()
        for key, label in (
            ("all", "全部"),
            ("system", "系统"),
            *((component, COMPONENT_LABELS[component])
              for component in START_ORDER),
        ):
            view = QPlainTextEdit()
            view.setObjectName("sensorLog")
            view.setReadOnly(True)
            view.setLineWrapMode(QPlainTextEdit.NoWrap)
            view.document().setMaximumBlockCount(3000)
            self.log_views[key] = view
            self.log_tabs.addTab(view, label)
        monitor_layout.addWidget(self.log_tabs, 1)
        splitter.addWidget(monitor)
        splitter.setSizes([620, 780])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

    def set_compact_mode(
        self, compact: bool, portrait: bool | None = None,
    ) -> None:
        """Stack controls and logs when the screen cannot fit two columns."""
        compact = bool(compact)
        portrait = (
            compact and self.width() < self.height()
            if portrait is None else compact and bool(portrait))
        landscape = compact and not portrait
        self.layout().setContentsMargins(
            *(8, 6, 8, 8) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(7 if compact else 14)
        self.header.setDirection(QBoxLayout.LeftToRight)
        self.splitter.setOrientation(Qt.Vertical if portrait else Qt.Horizontal)
        self.controls.setMinimumWidth(0 if compact else 560)
        if portrait:
            self.splitter.setSizes([400, 250])
        elif landscape:
            self.splitter.setSizes([350, 430])
        else:
            self.splitter.setSizes([620, 780])
        for label in self.findChildren(QLabel):
            if label.objectName() == "subtitle":
                label.setVisible(not compact)

    def _build_localization_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        selector = QFormLayout()
        selector.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.localization_map = QLineEdit(latest_map(
            self.map_storage_dir, (".pcd",)))
        self.localization_map.setPlaceholderText("选择用于定位和生成 SC 的 PCD 地图")
        self.localization_map.editingFinished.connect(
            self._notify_localization_map_selected)
        self.localization_map_row = self._path_row(self.localization_map, self._choose_localization_map)
        self.localization_selector = selector
        selector.addRow("定位 PCD 地图", self.localization_map_row)
        self.localization_backend = QComboBox()
        for key, backend in LOCALIZATION_BACKENDS.items():
            self.localization_backend.addItem(backend.label, key)
        selector.addRow("定位后端", self.localization_backend)
        layout.addLayout(selector)
        self.localization_backend_description = QLabel()
        self.localization_backend_description.setObjectName("hint")
        self.localization_backend_description.setWordWrap(True)
        layout.addWidget(self.localization_backend_description)
        self.localization_backend_stack = QStackedWidget()
        self.localization_backend_stack.addWidget(
            self._scrollable(self._build_pointcloud_localizer_panel()))
        self.localization_backend_stack.addWidget(
            self._build_disabled_localization_panel())
        self.localization_backend_stack.addWidget(
            self._build_custom_localization_panel())
        for key in ("external", "amcl"):
            description = QLabel("使用当前导航栅格和 LaserScan。启动后可在“重新定位”用两点设定初始位姿。" if key == "amcl" else
                                 "不启动 NAV 定位算法。需要提供与当前地图一致的 map→odom TF。")
            description.setWordWrap(True)
            description.setAlignment(Qt.AlignTop)
            self.localization_backend_stack.addWidget(description)
        layout.addWidget(self.localization_backend_stack, 1)
        self.localization_backend.currentIndexChanged.connect(
            self._update_localization_backend)
        self._update_localization_backend(0)
        return tab

    def _build_pointcloud_localizer_panel(self) -> QWidget:
        panel = QWidget()
        form = QFormLayout(panel)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.scan_context_map = QLineEdit()
        self.scan_context_map.setPlaceholderText("可选：与 PCD 匹配的 .sc.v2")
        form.addRow("Scan Context 地图", self.scan_context_map)
        self.localizer_config = QLineEdit()
        self.localizer_config.setPlaceholderText("留空使用 localizer 包内配置")
        form.addRow("Localizer 配置", self._path_row(
            self.localizer_config, self._choose_localizer_config))
        self.localizer_tuning = QGroupBox("高级匹配参数（下次启动生效）")
        self.localizer_tuning.setObjectName("localizerAdvancedTuning")
        self.localizer_tuning.setCheckable(True)
        self.localizer_tuning.setChecked(False)
        tuning = QGridLayout(self.localizer_tuning)
        tuning.setContentsMargins(8, 8, 8, 8)
        tuning.setHorizontalSpacing(8)
        tuning.setVerticalSpacing(6)
        tuning.addWidget(QLabel(""), 0, 0)
        tuning.addWidget(QLabel("粗匹配"), 0, 1)
        tuning.addWidget(QLabel("精匹配"), 0, 2)

        def score_box(name):
            box = QDoubleSpinBox()
            box.setObjectName(name)
            box.setDecimals(3)
            box.setRange(0.001, 10.0)
            box.setSingleStep(0.01)
            return box

        def resolution_box(name):
            box = QDoubleSpinBox()
            box.setObjectName(name)
            box.setDecimals(3)
            box.setRange(0.01, 2.0)
            box.setSingleStep(0.01)
            box.setSuffix(" m")
            return box

        def iteration_box(name):
            box = QSpinBox()
            box.setObjectName(name)
            box.setRange(1, 2000)
            box.setSingleStep(10)
            return box

        self.rough_score_thresh = score_box("roughScoreThreshold")
        self.refine_score_thresh = score_box("refineScoreThreshold")
        self.rough_scan_resolution = resolution_box("roughScanResolution")
        self.refine_scan_resolution = resolution_box("refineScanResolution")
        self.rough_map_resolution = resolution_box("roughMapResolution")
        self.refine_map_resolution = resolution_box("refineMapResolution")
        self.rough_max_iteration = iteration_box("roughMaxIterations")
        self.refine_max_iteration = iteration_box("refineMaxIterations")
        for row, (label, rough, refine) in enumerate((
            ("评分阈值", self.rough_score_thresh, self.refine_score_thresh),
            ("点云分辨率", self.rough_scan_resolution, self.refine_scan_resolution),
            ("地图分辨率", self.rough_map_resolution, self.refine_map_resolution),
            ("最大迭代次数", self.rough_max_iteration, self.refine_max_iteration),
        ), 1):
            tuning.addWidget(QLabel(label), row, 0)
            tuning.addWidget(rough, row, 1)
            tuning.addWidget(refine, row, 2)
        reset = QPushButton("恢复默认值")
        reset.setObjectName("secondaryButton")
        reset.clicked.connect(self.reset_localizer_tuning)
        tuning.addWidget(reset, 5, 0, 1, 3)
        self.localizer_tuning_summary = QLabel()
        self.localizer_tuning_summary.setObjectName("hint")
        self.localizer_tuning_summary.setWordWrap(True)
        tuning.addWidget(self.localizer_tuning_summary, 6, 0, 1, 3)
        self.localizer_tuning.toggled.connect(
            self._refresh_localizer_tuning_summary)
        for widget in (
            self.rough_score_thresh, self.refine_score_thresh,
            self.rough_scan_resolution, self.refine_scan_resolution,
            self.rough_map_resolution, self.refine_map_resolution,
            self.rough_max_iteration, self.refine_max_iteration,
        ):
            widget.valueChanged.connect(
                self._refresh_localizer_tuning_summary)
        self.reset_localizer_tuning()
        form.addRow(self.localizer_tuning)
        self.localizer_cloud_topic = QLineEdit(
            "/dlio/odom_node/pointcloud/deskewed")
        self.localizer_odom_topic = QLineEdit("/dlio/odom_node/odom")
        form.addRow("定位点云话题", self.localizer_cloud_topic)
        form.addRow("定位里程计话题", self.localizer_odom_topic)

        sc_hint = QLabel(
            "使用本页上方选择的定位 PCD，生成同目录、同文件名的 "
            "<地图>.pcd.sc.v2 数据库。")
        sc_hint.setObjectName("hint")
        sc_hint.setWordWrap(True)
        form.addRow("SC 地图", sc_hint)
        self.generate_sc_button = QPushButton("生成 Scan Context 地图")
        self.generate_sc_button.setObjectName("primaryButton")
        self.generate_sc_button.clicked.connect(self.generate_scan_context)
        form.addRow("离线生成", self.generate_sc_button)

        self.global_relocalize_service = QLineEdit(
            DEFAULT_GLOBAL_RELOCALIZE_SERVICE)
        form.addRow("重定位服务", self.global_relocalize_service)
        self.global_relocalize_button = QPushButton("触发全局重定位")
        self.global_relocalize_button.setObjectName("primaryButton")
        self.global_relocalize_button.clicked.connect(
            self.trigger_global_relocalization)
        form.addRow("在线重定位", self.global_relocalize_button)
        self.localization_tool_status = QLabel(self.localization_tools.status)
        self.localization_tool_status.setObjectName("statusBar")
        self.localization_tool_status.setWordWrap(True)
        form.addRow("", self.localization_tool_status)
        return panel

    def reset_localizer_tuning(self) -> None:
        for name, value in LOCALIZER_TUNING_DEFAULTS.items():
            getattr(self, name).setValue(value)
        self._refresh_localizer_tuning_summary()

    def _refresh_localizer_tuning_summary(self, *_args) -> None:
        if not self.localizer_tuning.isChecked():
            self.localizer_tuning_summary.setText(
                "未启用：使用所选 Localizer YAML 中的匹配参数。")
            return
        self.localizer_tuning_summary.setText(
            "页面覆盖已启用："
            f"粗匹配 {self.rough_score_thresh.value():.3f}，"
            f"精匹配 {self.refine_score_thresh.value():.3f}。"
            "数值越小越严格；停止并重新启动定位后生效。")

    @staticmethod
    def _build_disabled_localization_panel() -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        message = QLabel(
            "定位阶段将从启动顺序中移除。适合直接使用 LIO 里程计，或定位由外部系统"
            "提供的导航方案。")
        message.setObjectName("hint")
        message.setWordWrap(True)
        layout.addWidget(message)
        layout.addStretch(1)
        return panel

    def _build_custom_localization_panel(self) -> QWidget:
        panel = QWidget()
        form = QFormLayout(panel)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.custom_localization_package = QLineEdit()
        self.custom_localization_launch = QLineEdit()
        self.custom_localization_arguments = QLineEdit()
        self.custom_localization_arguments.setPlaceholderText(
            "map:={map} config:={config} use_sim_time:={use_sim_time}")
        form.addRow("ROS 包名", self.custom_localization_package)
        form.addRow("Launch 文件", self.custom_localization_launch)
        form.addRow("附加参数", self.custom_localization_arguments)
        hint = QLabel("支持占位符：{map}、{config}、{use_sim_time}")
        hint.setObjectName("hint")
        form.addRow("", hint)
        return panel

    def _build_navigation_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        selector = QFormLayout()
        selector.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.navigation_map = QLineEdit(latest_map(
            self.map_storage_dir, (".yaml", ".yml")))
        self.navigation_map.setPlaceholderText("选择 Nav2 使用的 YAML 栅格地图")
        selector.addRow("导航 YAML 地图", self._path_row(
            self.navigation_map, self._choose_navigation_map))
        self.navigation_backend = QComboBox()
        for key, backend in NAVIGATION_BACKENDS.items():
            self.navigation_backend.addItem(backend.label, key)
        selector.addRow("导航后端", self.navigation_backend)
        layout.addLayout(selector)
        self.navigation_backend_description = QLabel()
        self.navigation_backend_description.setObjectName("hint")
        self.navigation_backend_description.setWordWrap(True)
        layout.addWidget(self.navigation_backend_description)
        self.navigation_backend_stack = QStackedWidget()
        self.navigation_backend_stack.addWidget(self._build_nav2_panel())
        self.navigation_backend_stack.addWidget(
            self._scrollable(self._build_scan_planner_panel()))
        self.navigation_backend_stack.addWidget(
            self._build_custom_navigation_panel())
        self.navigation_backend_stack.addWidget(QLabel(
            "在「导航接入设置」选择 PLY 网格。该网格需与定位地图使用同一坐标系。"))
        layout.addWidget(self.navigation_backend_stack, 1)
        self.navigation_backend.currentIndexChanged.connect(
            self._update_navigation_backend)
        self._update_navigation_backend(0)
        return tab

    def _build_nav2_panel(self) -> QWidget:
        panel = QWidget()
        form = QFormLayout(panel)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.nav2_params = QLineEdit()
        self.nav2_params.setPlaceholderText("留空使用 nav2_bringup 默认参数")
        form.addRow("Nav2 参数文件", self._path_row(
            self.nav2_params, self._choose_nav2_params))
        self.autostart = QCheckBox("Nav2 自动进入 Active")
        self.autostart.setChecked(True)
        self.nav2_rviz = QCheckBox("同时启动 Nav2 RViz")
        form.addRow("生命周期", self.autostart)
        form.addRow("可视化", self.nav2_rviz)
        return panel

    def _build_scan_planner_panel(self) -> QWidget:
        panel = QWidget()
        form = QFormLayout(panel)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.scan_navi_mode = QComboBox()
        self.scan_navi_mode.addItem("1 · 单目标", 1)
        self.scan_navi_mode.addItem("2 · 参数途径点", 2)
        self.scan_navi_mode.addItem("3 · 外部参考路径", 3)
        self.scan_sensor_type = QComboBox()
        self.scan_sensor_type.addItem("激光点云", "lidar")
        self.scan_sensor_type.addItem("深度图", "depth")
        self.scan_body_pose_topic = QLineEdit("/dlio/odom_node/odom")
        self.scan_sensor_pose_topic = QLineEdit("/dlio/odom_node/odom")
        self.scan_cloud_topic = QLineEdit("/livox/lidar/pointcloud")
        self.scan_depth_topic = QLineEdit(
            "/camera/aligned_depth_to_color/image_raw")
        self.scan_goal_topic = QLineEdit("/move_base_simple/goal")
        self.scan_initial_path_topic = QLineEdit("/initial_path")
        self.scan_cmd_vel_topic = QLineEdit("/cmd_vel")
        self.scan_start_controller = QCheckBox("启动闭环速度控制器")
        self.scan_start_controller.setChecked(True)
        self.scan_cloud_is_world = QCheckBox("输入点云已经位于世界坐标系")
        self.scan_need_extrinsic = QCheckBox("应用 SCAN 内置传感器外参")
        self.scan_planner_params = QLineEdit()
        self.scan_controller_params = QLineEdit()
        self.scan_keypoints_file = QLineEdit()
        self.scan_reference_path_file = QLineEdit()
        form.addRow("导航模式", self.scan_navi_mode)
        form.addRow("感知类型", self.scan_sensor_type)
        form.addRow("机身里程计", self.scan_body_pose_topic)
        form.addRow("传感器位姿", self.scan_sensor_pose_topic)
        form.addRow("点云话题", self.scan_cloud_topic)
        form.addRow("深度图话题", self.scan_depth_topic)
        form.addRow("目标话题", self.scan_goal_topic)
        form.addRow("参考路径话题", self.scan_initial_path_topic)
        form.addRow("速度输出", self.scan_cmd_vel_topic)
        form.addRow("控制器", self.scan_start_controller)
        form.addRow("点云坐标", self.scan_cloud_is_world)
        form.addRow("传感器外参", self.scan_need_extrinsic)
        form.addRow("规划器参数", self._path_row(
            self.scan_planner_params,
            lambda: self._choose_yaml(
                self.scan_planner_params, "选择 SCAN-Planner 参数")))
        form.addRow("控制器参数", self._path_row(
            self.scan_controller_params,
            lambda: self._choose_yaml(
                self.scan_controller_params, "选择 SCAN 控制器参数")))
        form.addRow("模式 2 途径点", self._path_row(
            self.scan_keypoints_file,
            lambda: self._choose_yaml(
                self.scan_keypoints_file, "选择 SCAN 途径点")))
        form.addRow("模式 3 参考路径", self._path_row(
            self.scan_reference_path_file,
            lambda: self._choose_yaml(
                self.scan_reference_path_file, "选择 SCAN 参考路径")))
        return panel

    def _build_custom_navigation_panel(self) -> QWidget:
        panel = QWidget()
        form = QFormLayout(panel)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.custom_navigation_package = QLineEdit()
        self.custom_navigation_launch = QLineEdit()
        self.custom_navigation_arguments = QLineEdit()
        self.custom_navigation_arguments.setPlaceholderText(
            "map:={map} params_file:={params} use_sim_time:={use_sim_time}")
        form.addRow("ROS 包名", self.custom_navigation_package)
        form.addRow("Launch 文件", self.custom_navigation_launch)
        form.addRow("附加参数", self.custom_navigation_arguments)
        hint = QLabel("支持占位符：{map}、{params}、{use_sim_time}")
        hint.setObjectName("hint")
        form.addRow("", hint)
        return panel

    def _build_dlio_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.odometry_backend = QComboBox()
        self.odometry_backend.addItem("DLIO", "dlio")
        self.odometry_backend.addItem("FAST-LIVO2", "fast_livo")
        self.fast_livo_lidar_topic = QLineEdit("/livox/lidar")
        self.fast_livo_image_topic = QLineEdit("/camera/camera/color/image_raw")
        self.fast_livo_params = QLineEdit()
        self.fast_livo_camera_params = QLineEdit()
        self.fast_livo_vision = QCheckBox("启用视觉融合")
        self.fast_livo_vision.setToolTip("启用前核对本机相机内外参、分辨率和时间补偿")
        form.addRow("里程计后端", self.odometry_backend)
        form.addRow("LIVO 点云话题", self.fast_livo_lidar_topic)
        form.addRow("LIVO 相机话题", self.fast_livo_image_topic)
        form.addRow("LIVO 配置", self._path_row(self.fast_livo_params,
            lambda: self._choose_yaml(self.fast_livo_params, "选择 LIVO 配置")))
        form.addRow("LIVO 相机标定", self._path_row(self.fast_livo_camera_params,
            lambda: self._choose_yaml(self.fast_livo_camera_params, "选择相机标定")))
        form.addRow("视觉", self.fast_livo_vision)
        self.dlio_pointcloud_topic = QLineEdit("/livox/lidar/pointcloud")
        self.dlio_imu_topic = QLineEdit("/livox/imu")
        self.dlio_rviz = QCheckBox("启动里程计 RViz")
        form.addRow("点云话题", self.dlio_pointcloud_topic)
        form.addRow("IMU 话题", self.dlio_imu_topic)
        form.addRow("RViz", self.dlio_rviz)
        self.odometry_backend.addItem("使用已有里程计", "external")
        self.odometry_backend.currentIndexChanged.connect(self._update_odometry_backend)
        self._update_odometry_backend()
        return tab

    def _update_odometry_backend(self, *_args) -> None:
        fast_livo = self.odometry_backend.currentData() == "fast_livo"
        for widget in (self.fast_livo_lidar_topic, self.fast_livo_image_topic,
                       self.fast_livo_params, self.fast_livo_camera_params, self.fast_livo_vision):
            widget.setEnabled(fast_livo)
        self.dlio_pointcloud_topic.setEnabled(not fast_livo)

    def _build_laserscan_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.laser_cloud_topic = QLineEdit("/livox/lidar/pointcloud")
        self.scan_topic = QLineEdit("/scan")
        self.laser_target_frame = QLineEdit("livox_frame")
        form.addRow("输入点云", self.laser_cloud_topic)
        form.addRow("输出扫描", self.scan_topic)
        form.addRow("目标坐标系", self.laser_target_frame)
        self.min_height = self._number(-20.0, 20.0, -1.0)
        self.max_height = self._number(-20.0, 20.0, 0.1)
        self.range_min = self._number(0.0, 1000.0, 0.45)
        self.range_max = self._number(0.01, 1000.0, 10.0)
        form.addRow("最低高度", self.min_height)
        form.addRow("最高高度", self.max_height)
        form.addRow("最小量程", self.range_min)
        form.addRow("最大量程", self.range_max)
        return tab

    def _build_general_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.use_sim_time = QCheckBox("使用 /clock 仿真时间")
        self.startup_interval = self._number(0.0, 10.0, 1.0)
        self.startup_interval.setSuffix(" s")
        self.sensor_timeout = self._number(1.0, 120.0, 12.0)
        self.sensor_timeout.setSuffix(" s")
        form.addRow("时间源", self.use_sim_time)
        form.addRow("组件启动间隔", self.startup_interval)
        form.addRow("传感器数据超时", self.sensor_timeout)
        return tab

    @staticmethod
    def _number(minimum: float, maximum: float, value: float):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setDecimals(2)
        box.setSingleStep(0.05)
        box.setValue(value)
        box.setSuffix(" m")
        return box

    @staticmethod
    def _path_row(field: QLineEdit, callback) -> QWidget:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        choose = QPushButton("选择")
        choose.setObjectName("secondaryButton")
        choose.clicked.connect(callback)
        layout.addWidget(field, 1)
        layout.addWidget(choose)
        return row

    @staticmethod
    def _scrollable(widget: QWidget) -> QScrollArea:
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.NoFrame)
        area.setWidget(widget)
        return area

    def _update_localization_backend(self, index: int) -> None:
        self.localization_backend_stack.setCurrentIndex(index)
        key = self.localization_backend.itemData(index)
        self.localization_map_row.setVisible(key not in ("amcl", "external"))
        self.localization_selector.labelForField(self.localization_map_row).setVisible(key not in ("amcl", "external"))
        backend = LOCALIZATION_BACKENDS.get(key)
        self.localization_backend_description.setText(
            backend.description if backend is not None else "")

    def _update_navigation_backend(self, index: int) -> None:
        key = self.navigation_backend.itemData(index)
        self.nav2_params.setPlaceholderText("留空使用仅规划参数" if key == "walking" else "留空使用 nav2_bringup 默认参数")
        self.nav2_rviz.setEnabled(key != "walking")
        self.navigation_backend_stack.setCurrentIndex({"nav2": 0, "walking": 0, "scan_planner": 1, "custom": 2, "meshnav": 3}[key])
        backend = NAVIGATION_BACKENDS.get(key)
        self.navigation_backend_description.setText(
            backend.description if backend is not None else "")

    def _choose_navigation_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择导航地图", str(self.map_storage_dir),
            "Nav2 Map (*.yaml *.yml);;All Files (*)")
        if path:
            self.navigation_map.setText(path)

    def _choose_localization_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择定位地图", str(self.map_storage_dir),
            "Point Cloud Map (*.pcd);;All Files (*)")
        if path:
            self.localization_map.setText(path)
            self.localization_map_selected.emit(path)

    def _notify_localization_map_selected(self) -> None:
        path = self.localization_map.text().strip()
        if path:
            self.localization_map_selected.emit(path)

    def _choose_nav2_params(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择 Nav2 参数文件", self.nav2_params.text(),
            "YAML (*.yaml *.yml)")
        if path:
            self.nav2_params.setText(path)

    def _choose_localizer_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择 Localizer 配置", self.localizer_config.text(),
            "YAML (*.yaml *.yml)")
        if path:
            self.localizer_config.setText(path)

    def _choose_yaml(self, field: QLineEdit, title: str) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, title, field.text() or str(self.map_storage_dir),
            "YAML (*.yaml *.yml);;All Files (*)")
        if path:
            field.setText(path)

    def configuration(self) -> NavigationStackConfig:
        scene = self.meshnav_scene
        scene_matches = (scene is not None and scene.cloud == self.localization_map.text().strip()
                         and scene.grid == self.navigation_map.text().strip())
        return NavigationStackConfig(
            robot_profile=self.robot_profile,
            woosh_control_mode=self.woosh_control_mode,
            meshnav_map=(scene.meshnav if scene_matches and scene.meshnav
                         else self.access_settings.get("meshnav_map", "")),
            meshnav_map_error=scene.meshnav_error if scene_matches else "",
            lidar_driver_mode=self.access_settings.get("lidar_driver_mode", "profile"),
            camera_driver_mode=self.access_settings.get("camera_driver_mode", "profile"),
            scan_source=self.access_settings.get("scan_source", "profile"),
            external_odom_topic=self.access_settings.get("external_odom_topic", "/odom"),
            woosh_fast_livo_profile=self.access_settings.get("woosh_fast_livo_profile", "livox3"),
            map_gravity_aligned=(self.robot_profile == "woosh" and self.odometry_backend.currentData() == "fast_livo"),
            navigation_map=self.navigation_map.text().strip(),
            localization_map=self.localization_map.text().strip(),
            localization_backend=self.localization_backend.currentData(),
            navigation_backend=self.navigation_backend.currentData(),
            lidar_arguments=self.lidar_arguments(),
            camera_arguments=self.camera_arguments(),
            use_sim_time=self.use_sim_time.isChecked(),
            autostart=self.autostart.isChecked(),
            odometry_backend=self.odometry_backend.currentData(),
            fast_livo_lidar_topic=self.fast_livo_lidar_topic.text().strip(),
            fast_livo_image_topic=self.fast_livo_image_topic.text().strip(),
            fast_livo_params=self.fast_livo_params.text().strip(),
            fast_livo_camera_params=self.fast_livo_camera_params.text().strip(),
            fast_livo_vision=self.fast_livo_vision.isChecked(),
            dlio_rviz=self.dlio_rviz.isChecked(),
            dlio_pointcloud_topic=self.dlio_pointcloud_topic.text().strip(),
            dlio_imu_topic=self.dlio_imu_topic.text().strip(),
            laser_cloud_topic=self.laser_cloud_topic.text().strip(),
            scan_topic=self.scan_topic.text().strip(),
            laser_target_frame=self.laser_target_frame.text().strip(),
            min_height=self.min_height.value(),
            max_height=self.max_height.value(),
            range_min=self.range_min.value(),
            range_max=self.range_max.value(),
            localizer_cloud_topic=self.localizer_cloud_topic.text().strip(),
            localizer_odom_topic=self.localizer_odom_topic.text().strip(),
            localizer_config=self.localizer_config.text().strip(),
            localizer_tuning_enabled=self.localizer_tuning.isChecked(),
            rough_score_thresh=self.rough_score_thresh.value(),
            refine_score_thresh=self.refine_score_thresh.value(),
            rough_scan_resolution=self.rough_scan_resolution.value(),
            refine_scan_resolution=self.refine_scan_resolution.value(),
            rough_map_resolution=self.rough_map_resolution.value(),
            refine_map_resolution=self.refine_map_resolution.value(),
            rough_max_iteration=self.rough_max_iteration.value(),
            refine_max_iteration=self.refine_max_iteration.value(),
            scan_context_map=self.scan_context_map.text().strip(),
            nav2_params=self.nav2_params.text().strip(),
            nav2_rviz=self.nav2_rviz.isChecked(),
            custom_localization_package=(
                self.custom_localization_package.text().strip()),
            custom_localization_launch=(
                self.custom_localization_launch.text().strip()),
            custom_localization_arguments=(
                self.custom_localization_arguments.text().strip()),
            custom_navigation_package=(
                self.custom_navigation_package.text().strip()),
            custom_navigation_launch=(
                self.custom_navigation_launch.text().strip()),
            custom_navigation_arguments=(
                self.custom_navigation_arguments.text().strip()),
            scan_navi_mode=self.scan_navi_mode.currentData(),
            scan_sensor_type=self.scan_sensor_type.currentData(),
            scan_body_pose_topic=self.scan_body_pose_topic.text().strip(),
            scan_sensor_pose_topic=self.scan_sensor_pose_topic.text().strip(),
            scan_cloud_topic=self.scan_cloud_topic.text().strip(),
            scan_depth_topic=self.scan_depth_topic.text().strip(),
            scan_goal_topic=self.scan_goal_topic.text().strip(),
            scan_initial_path_topic=(
                self.scan_initial_path_topic.text().strip()),
            scan_cmd_vel_topic=self.scan_cmd_vel_topic.text().strip(),
            scan_start_controller=self.scan_start_controller.isChecked(),
            scan_cloud_is_world=self.scan_cloud_is_world.isChecked(),
            scan_need_extrinsic=self.scan_need_extrinsic.isChecked(),
            scan_planner_params=self.scan_planner_params.text().strip(),
            scan_controller_params=self.scan_controller_params.text().strip(),
            scan_keypoints_file=self.scan_keypoints_file.text().strip(),
            scan_reference_path_file=(
                self.scan_reference_path_file.text().strip()),
            startup_interval_ms=int(self.startup_interval.value() * 1000),
            sensor_timeout_s=self.sensor_timeout.value(),
        )

    def apply_access_settings(self, values):
        previous = self.access_settings
        mount_changed = values.get("woosh_fast_livo_profile", "livox3") != previous.get("woosh_fast_livo_profile", "livox3")
        native5 = values.get("woosh_fast_livo_profile") == "native5"
        if native5 and values["scan_source"] == "convert" and (mount_changed or previous.get("scan_source") != "convert"):
            self.min_height.setValue(.05)
            self.max_height.setValue(1.5)
        if mount_changed:
            self.fast_livo_params.clear()  # A different mount must not reuse the previous sensor's override.
        self.access_settings = dict(values)
        for key in ("odometry_backend", "localization_backend"):
            combo = getattr(self, key)
            combo.setCurrentIndex(combo.findData(values[key]))
            combo.setEnabled(False)
        for key, field in (("lidar_topic", self.dlio_pointcloud_topic),
                           ("fast_livo_lidar_topic", self.fast_livo_lidar_topic),
                           ("lidar_topic", self.laser_cloud_topic),
                           ("imu_topic", self.dlio_imu_topic),
                           ("image_topic", self.fast_livo_image_topic),
                           ("scan_topic", self.scan_topic),
                           ("laser_target_frame", self.laser_target_frame)):
            field.setText(values[key])
        if native5 and values["odometry_backend"] == "fast_livo":
            self.laser_cloud_topic.setText("/nav/livox5/points_filtered")
        self.fast_livo_vision.setChecked(values["fast_livo_vision"])
        names = dict(lidar="雷达驱动 · " + ("NAV 启动" if values["lidar_driver_mode"] == "nav" else "已有话题"),
            camera="相机驱动 · " + {"nav": "NAV 启动", "external": "已有话题", "off": "不使用"}[values["camera_driver_mode"]],
            dlio="里程计 · " + {"external": "已有 Odometry", "fast_livo": "FAST-LIVO2", "dlio": "DLIO"}[values["odometry_backend"]],
            laserscan="避障 scan · " + ("已有 LaserScan" if values["scan_source"] == "external" else "点云转换"),
            localizer="地图定位 · " + {"amcl": "AMCL", "external": "已有 TF", "pointcloud_localizer": "点云 Localizer"}[values["localization_backend"]])
        for index, key in enumerate(START_ORDER[:-1], start=1):
            self.component_names[key].setText(f"{index}. {names[key]}")
        for key, state in self.controller.states.items():
            self._handle_component_state(key, state)
        self.set_operating_mode(self.operating_mode, self.woosh_control_mode)

    def set_operating_mode(self, mode, control_mode):
        self.operating_mode = mode
        self.woosh_control_mode = control_mode
        walking = mode == "walking"
        mesh = self.access_settings.get("navigation_engine") == "meshnav"
        self.navigation_backend.setCurrentIndex(self.navigation_backend.findData(
            "meshnav" if mesh else "walking" if walking else "nav2"))
        self.navigation_backend.setEnabled(False)
        label = "路线规划 · MeshNav" if mesh else "路线规划 · Nav2" if walking else "统一导航 · Nav2"
        self.component_names["nav2"].setText(f"{len(START_ORDER)}. {label}")
        if mesh and self.woosh_control_mode:
            status = "MeshNav 自动行驶；确认三维路线后点击“开始自动行驶”。"
            self.component_names["laserscan"].setText("4. 自动行驶避障 · 实时三维点云")
        elif mesh:
            status = "MeshNav 三维规划；使用手柄控制行走。"
            self.component_names["laserscan"].setText("4. 避障 LaserScan · MeshNav 无需此输入")
        elif walking:
            status = "手柄规划；使用手柄控制行走。"
        elif self.woosh_control_mode:
            status = "自动导航；路线预览后点击“开始导航”，需定位和底盘反馈就绪。"
        else:
            status = "接入设置已应用；导航后端为 Nav2。"
        self.action_status.setText(status)

    def start_stack(self) -> tuple[bool, str]:
        config = self.configuration()
        success, status = self.controller.start(config)
        self.action_status.setText(status)
        if success and config.localization_map:
            self.localization_map_selected.emit(config.localization_map)
        elif not success:
            self._append_log("system", status + "\n")
        return success, status

    def generate_scan_context(self) -> None:
        _success, status = self.localization_tools.generate_scan_context(
            self.localization_map.text().strip())
        self.localization_tool_status.setText(status)

    def trigger_global_relocalization(self) -> None:
        _success, status = (
            self.localization_tools.trigger_global_relocalization(
                self.global_relocalize_service.text().strip()))
        self.localization_tool_status.setText(status)

    def _handle_localization_operation_state(
        self, operation: str, state: str,
    ) -> None:
        busy = state in ("starting", "running")
        if operation == "scan_context":
            self.generate_sc_button.setEnabled(not busy)
            self.generate_sc_button.setText(
                "正在生成 SC 地图…" if busy else "生成 Scan Context 地图")
        elif operation == "global_relocalize":
            self.global_relocalize_button.setEnabled(not busy)
            self.global_relocalize_button.setText(
                "正在请求重定位…" if busy else "触发全局重定位")

    def _handle_localization_tool_status(self, status: str) -> None:
        self.localization_tool_status.setText(status)

    def activate(self) -> None:
        """Show the bounded pending tail without replaying controller history."""
        self._flush_logs(force=True)
        for key, state in self.controller.states.items():
            self._handle_component_state(key, state)
        self._handle_overall_state(self.controller.overall_state)

    def _append_log(self, key: str, text: str) -> None:
        clean = ANSI_ESCAPE_RE.sub("", text).replace("\r", "").rstrip("\n")
        if not clean:
            return
        lines = clean.splitlines()
        prefix = "系统" if key == "system" else COMPONENT_LABELS.get(key, key)
        if key in self._pending_logs:
            self._pending_logs[key].extend(lines)
        self._pending_logs["all"].extend(f"[{prefix}] {line}" for line in lines)

    def _flush_logs(self, force=False) -> None:
        if not force and not self.isVisible():
            return
        current = self.log_tabs.currentWidget()
        for key, view in self.log_views.items():
            pending = self._pending_logs[key]
            if view is current and pending:
                view.appendPlainText("\n".join(pending))
                pending.clear()

    def _clear_logs(self) -> None:
        for key, view in self.log_views.items():
            self._pending_logs[key].clear()
            view.clear()

    def _handle_component_state(self, key: str, state: str) -> None:
        if state == "stopped" and self.access_settings:
            values = self.access_settings
            if (key in ("lidar", "camera") and values[key + "_driver_mode"] == "external"
                    or key == "laserscan" and values["scan_source"] == "external"
                    or key == "dlio" and values["odometry_backend"] == "external"
                    or key == "localizer" and values["localization_backend"] == "external"):
                state = "external"
            elif key == "camera" and values["camera_driver_mode"] == "off":
                state = "disabled"
            elif key == "localizer" and values["localization_backend"] == "amcl":
                state = "nav2_managed"
        labels = {
            "starting": ("启动中", "#ffd166"),
            "waiting": ("等待数据", "#ffd166"),
            "running": ("运行中", "#66e09a"),
            "stopping": ("停止中", "#ffd166"),
            "stopped": ("未启动", "#91a2ad"),
            "disabled": ("不使用", "#71828e"),
            "external": ("外部输入", "#70cddd"),
            "nav2_managed": ("由 Nav2 启动", "#70cddd"),
            "error": ("失败", "#ff7f88"),
        }
        text, color = labels.get(state, (state, "#91a2ad"))
        status = self.component_status.get(key)
        if status is not None:
            status.setText(f"● {text}")
            status.setStyleSheet(f"color: {color}; font-weight: 700;")

    def _handle_overall_state(self, state: str) -> None:
        labels = {
            "starting": ("系统启动中", "#ffd166"),
            "running": ("系统运行中", "#66e09a"),
            "stopping": ("系统停止中", "#ffd166"),
            "stopped": ("系统未启动", "#91a2ad"),
            "error": ("启动失败", "#ff7f88"),
        }
        text, color = labels.get(state, (state, "#91a2ad"))
        self.overall_status.setText(f"● {text}")
        self.overall_status.setStyleSheet(
            f"color: {color}; font-weight: 700;")
        busy = state in ("starting", "stopping")
        self.localizer_tuning.setEnabled(state in ("stopped", "error"))
        self.start_button.setEnabled(not busy and state != "running")
        self.stop_button.setEnabled(state in ("starting", "running", "error"))
        if state == "running":
            self.action_status.setText("导航输入与 Nav2 已就绪，可以选择目的地")
        elif state == "error":
            self.action_status.setText(self.controller._waiting_input_message.replace("等待输入：", "输入超时：", 1) or "导航系统启动失败，请查看右侧日志")

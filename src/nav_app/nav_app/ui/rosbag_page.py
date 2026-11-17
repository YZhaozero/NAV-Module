"""Qt workspace for selective rosbag recording and replay workflows."""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..features.rosbag import (
    BASE_SENSOR_TOPICS,
    BagMetadata,
    BagTopic,
    PlaybackRequest,
    RosbagController,
    default_record_selected,
    read_bag_metadata,
    previous_capture_warning,
    safe_algorithm_topic,
)
from .touch_scroll import enable_touch_scroll
from .page_header import PageHeader


class RosbagPage(QWidget):
    """Record selected topics and replay a bag into existing runtime stacks."""

    return_requested = pyqtSignal()
    mapping_requested = pyqtSignal(object)
    navigation_requested = pyqtSignal(object)
    mapping_view_requested = pyqtSignal()

    def __init__(self, controller: RosbagController, parent=None, *, embedded=False) -> None:
        super().__init__(parent)
        self.controller = controller
        self.embedded = embedded
        self.metadata: BagMetadata | None = None
        self._build_ui()
        self.controller.log_received.connect(self._append_log)
        self.controller.state_changed.connect(self._handle_state)
        self.controller.record_finished.connect(self._record_finished)
        warning = previous_capture_warning()
        if warning:
            self.record_status.setText(warning)

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(14)

        if not self.embedded:
            header = PageHeader("数据录制与回放", self.return_requested.emit,
                destination="返回导航主页", subtitle="选择话题录制 · 回放建图 · 导航算法回放")
            layout.addLayout(header)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_record_tab(), "录制 rosbag")
        self.tabs.addTab(self._build_play_tab(), "加载与回放")
        self.mapping_view = QPushButton("建图视图  →")
        self.mapping_view.setObjectName("workspaceButton")
        self.mapping_view.setToolTip("打开实时建图与点云页面")
        self.mapping_view.setCursor(Qt.PointingHandCursor)
        self.mapping_view.clicked.connect(self.mapping_view_requested.emit)
        self.mapping_corner = QWidget()
        mapping_corner_layout = QHBoxLayout(self.mapping_corner)
        mapping_corner_layout.setContentsMargins(0, 0, 10, 0)
        mapping_corner_layout.setSpacing(0)
        mapping_corner_layout.addWidget(self.mapping_view)
        self.tabs.setCornerWidget(self.mapping_corner, Qt.TopRightCorner)
        self.tabs.currentChanged.connect(
            lambda index: self.mapping_corner.setVisible(index == 1))
        self.mapping_corner.setVisible(self.tabs.currentIndex() == 1)
        self.tabs.currentChanged.connect(
            lambda index: self.activate() if index == 0 else None)
        layout.addWidget(self.tabs, 1)

    @staticmethod
    def _topic_tree(headers) -> QTreeWidget:
        tree = QTreeWidget()
        tree.setColumnCount(len(headers))
        tree.setHeaderLabels(headers)
        tree.setRootIsDecorated(False)
        tree.setAlternatingRowColors(True)
        tree.setSelectionMode(QAbstractItemView.ExtendedSelection)
        tree.header().setStretchLastSection(True)
        enable_touch_scroll(tree)
        return tree

    def _build_record_tab(self) -> QWidget:
        tab = QWidget()
        splitter = QSplitter(Qt.Horizontal)
        self.record_splitter = splitter
        root = QVBoxLayout(tab)
        root.setContentsMargins(0, 8, 0, 0)
        root.addWidget(splitter, 1)

        topics_card = QFrame()
        topics_card.setObjectName("sidePanel")
        topics_layout = QVBoxLayout(topics_card)
        heading = QHBoxLayout()
        heading.addWidget(QLabel("当前 ROS 2 话题"))
        heading.addStretch(1)
        clear = QPushButton("清空")
        clear.clicked.connect(lambda: self._set_tree_checked(
            self.record_topics, False))
        refresh = QPushButton("刷新并选择传感器基础话题")
        refresh.setObjectName("secondaryButton")
        refresh.clicked.connect(self.refresh_record_topics)
        heading.addWidget(clear)
        heading.addWidget(refresh)
        topics_layout.addLayout(heading)
        self.record_topics = self._topic_tree(
            ("录制", "话题", "消息类型", "状态"))
        self.record_topics.setColumnWidth(0, 58)
        self.record_topics.setColumnWidth(1, 300)
        topics_layout.addWidget(self.record_topics, 1)
        splitter.addWidget(topics_card)

        controls = QFrame()
        controls.setObjectName("sidePanel")
        controls.setMinimumWidth(380)
        controls.setMaximumWidth(520)
        self.record_controls = controls
        form_layout = QVBoxLayout(controls)
        self.record_controls_layout = form_layout
        form = QFormLayout()
        self.record_form = form
        default_root = Path(os.environ.get(
            "NAV_BAG_DIR", str(Path.home() / "rosbags"))).expanduser()
        self.record_directory = QLineEdit(str(default_root))
        choose_row = QWidget()
        choose_layout = QHBoxLayout(choose_row)
        choose_layout.setContentsMargins(0, 0, 0, 0)
        choose_layout.addWidget(self.record_directory, 1)
        choose = QPushButton("选择")
        choose.clicked.connect(self._choose_record_directory)
        choose_layout.addWidget(choose)
        form.addRow("保存目录", choose_row)
        self.record_name = QLineEdit(
            datetime.now().strftime("nav_bag_%Y%m%d_%H%M%S"))
        form.addRow("Bag 名称", self.record_name)
        self.storage_id = QComboBox()
        self.storage_id.addItems(("sqlite3", "mcap"))
        form.addRow("存储格式", self.storage_id)
        self.image_storage = QComboBox()
        self.image_storage.addItem("原始 RGB（无损）", "raw")
        self.image_storage.addItem("JPEG 95（有损，节省空间）", "jpeg95")
        form.addRow("图像保存", self.image_storage)
        self.compression = QComboBox()
        # 默认逐消息: 高码率(深度+彩色约 160 MB/s)下文件级压缩会积压待压缩分片,
        # 停机要等很久甚至被误杀; 逐消息在写入时压缩, 没有积压, 压缩比几乎一样。
        self.compression.addItem(
            "zstd（逐消息压缩，长录制推荐）", userData=("message", "zstd"))
        self.compression.addItem(
            "zstd（文件级压缩，停机较慢）", userData=("file", "zstd"))
        self.compression.addItem("不压缩（体积大）", userData=("none", "none"))
        form.addRow("压缩格式", self.compression)
        self.max_cache_mb = QLineEdit("1000")
        self.max_cache_mb.setToolTip(
            "ros2 bag record --max-cache-size（MB）；高码率下太小会丢包")
        form.addRow("缓存大小 (MB)", self.max_cache_mb)
        self.max_bag_mb = QLineEdit("1000")
        self.max_bag_mb.setToolTip("单个 db3 分片上限（MB）；0 = 不分片")
        form.addRow("分片大小 (MB)", self.max_bag_mb)
        self.unique_if_exists = QCheckBox("同名目录自动加时间戳（不勾＝拒绝启动）")
        self.unique_if_exists.setChecked(False)
        form.addRow("同名目录", self.unique_if_exists)
        form_layout.addLayout(form)

        hint = QLabel(
            "录制前检查传感器、内存和磁盘。停止后等待落盘；随后选择录包执行离线校验。"
            "默认使用 zstd 消息压缩、1 GB 写缓存和 1 GB 分片；已有目录不会被覆盖。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        self.record_hint = hint
        form_layout.addWidget(hint)
        self.record_status = QLabel("尚未开始录制")
        self.record_status.setWordWrap(True)
        self.record_status.setStyleSheet("font-size: 15px; font-weight: 600; padding: 4px;")
        root.insertWidget(0, self.record_status)
        from ..features.log_store import LogBuffer
        self.record_log = LogBuffer("recording", "record-events")

        # 小屏(800x480)上 QSplitter 会把右侧表单压到互相重叠(下拉框挤在一起)。
        # 与"建图"页一致: 控件放进可滚动区域, 高度不够就出现滚动条, 不再压缩行高。
        controls_scroll = QScrollArea()
        controls_scroll.setObjectName("transparentScroll")
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setWidget(controls)
        enable_touch_scroll(controls_scroll)

        # 但"开始/停止录制"固定在面板底部, 不跟着滚 —— 小屏上不必翻页才能开始录。
        actions = QHBoxLayout()
        self.record_stop = QPushButton("停止录制")
        self.record_stop.setObjectName("dangerButton")
        self.record_stop.clicked.connect(self.controller.stop_recording)
        self.record_start = QPushButton("开始录制")
        self.record_start.setObjectName("primaryButton")
        self.record_start.clicked.connect(self._start_recording)
        actions.addWidget(self.record_stop)
        actions.addWidget(self.record_start)
        actions_frame = QFrame()
        actions_frame.setObjectName("sidePanel")
        actions_layout = QVBoxLayout(actions_frame)
        actions_layout.setContentsMargins(10, 8, 10, 8)
        actions_layout.addLayout(actions)

        self.record_controls_scroll = controls_scroll
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        right_layout.addWidget(controls_scroll, 1)
        right_layout.addWidget(actions_frame)
        splitter.addWidget(right_panel)
        splitter.setSizes([850, 450])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        self._handle_state("record", "stopped")
        return tab

    def _build_play_tab(self) -> QWidget:
        tab = QWidget()
        splitter = QSplitter(Qt.Horizontal)
        self.play_splitter = splitter
        root = QHBoxLayout(tab)
        root.setContentsMargins(0, 8, 0, 0)
        root.addWidget(splitter)

        topics_card = QFrame()
        topics_card.setObjectName("sidePanel")
        topics_layout = QVBoxLayout(topics_card)
        bag_row = QHBoxLayout()
        self.bag_path = QLineEdit()
        self.bag_path.setPlaceholderText("选择包含 metadata.yaml 的 rosbag 目录")
        load = QPushButton("加载 rosbag")
        load.setObjectName("secondaryButton")
        load.clicked.connect(self._choose_bag)
        bag_row.addWidget(self.bag_path, 1)
        bag_row.addWidget(load)
        topics_layout.addLayout(bag_row)
        self.bag_summary = QLabel("尚未加载 rosbag")
        self.bag_summary.setObjectName("hint")
        self.bag_summary.setWordWrap(True)
        topics_layout.addWidget(self.bag_summary)
        self.play_topics = self._topic_tree(
            ("回放", "Bag 话题", "消息类型", "消息数", "目标话题/Remap"))
        self.play_topics.setColumnWidth(0, 58)
        self.play_topics.setColumnWidth(1, 240)
        self.play_topics.setColumnWidth(2, 230)
        self.play_topics.setColumnWidth(3, 90)
        topics_layout.addWidget(self.play_topics, 1)
        splitter.addWidget(topics_card)

        controls = QFrame()
        controls.setObjectName("sidePanel")
        controls.setMinimumWidth(390)
        controls.setMaximumWidth(520)
        self.play_controls = controls
        controls_layout = QVBoxLayout(controls)
        self.play_controls_layout = controls_layout
        form = QFormLayout()
        self.play_form = form
        self.play_rate = QDoubleSpinBox()
        self.play_rate.setRange(0.05, 10.0)
        self.play_rate.setDecimals(2)
        self.play_rate.setValue(1.0)
        self.clock_hz = QDoubleSpinBox()
        self.clock_hz.setRange(1.0, 1000.0)
        self.clock_hz.setValue(50.0)
        self.clock_hz.setSuffix(" Hz")
        self.play_loop = QCheckBox("仅普通预览使用；算法模式会自动关闭")
        self.play_loop.setToolTip(
            "循环播放仅用于普通预览，建图/导航算法回放会自动关闭循环")
        form.addRow("播放倍率", self.play_rate)
        form.addRow("/clock 频率", self.clock_hz)
        form.addRow("循环播放", self.play_loop)
        controls_layout.addLayout(form)
        warning = QLabel(
            "建图和导航回放只发布原始传感器输入，自动排除录制的里程计、"
            "地图、路径、cmd_vel 与动态 TF，避免和重新运行的节点冲突。")
        warning.setObjectName("hint")
        warning.setWordWrap(True)
        self.play_warning = warning
        controls_layout.addWidget(warning)

        self.play_status = QLabel("尚未开始回放")
        self.play_status.setWordWrap(True)
        controls_layout.addWidget(self.play_status)
        playback_actions = QHBoxLayout()
        self.play_stop = QPushButton("停止")
        self.play_stop.setObjectName("dangerButton")
        self.play_stop.clicked.connect(self.controller.stop_playback)
        pause = QPushButton("暂停")
        pause.clicked.connect(self._pause)
        resume = QPushButton("继续")
        resume.clicked.connect(self._resume)
        self.play_start = QPushButton("普通回放")
        self.play_start.setObjectName("primaryButton")
        self.play_start.clicked.connect(self._start_plain_playback)
        for button in (self.play_stop, pause, resume, self.play_start):
            playback_actions.addWidget(button)
        controls_layout.addLayout(playback_actions)

        workflow_title = QLabel("使用 rosbag 运行算法")
        workflow_title.setObjectName("sectionTitle")
        controls_layout.addWidget(workflow_title)
        mapping = QPushButton("使用当前建图配置回放建图")
        mapping.setObjectName("primaryButton")
        mapping.clicked.connect(self._request_mapping)
        navigation = QPushButton("使用当前导航配置做算法回放")
        navigation.setObjectName("primaryButton")
        navigation.clicked.connect(self._request_navigation)
        controls_layout.addWidget(mapping)
        controls_layout.addWidget(navigation)
        replay_boundary = QLabel(
            "导航算法回放用于检查定位、路径和控制输出，不代表实车闭环导航。")
        replay_boundary.setWordWrap(True)
        self.replay_boundary = replay_boundary
        controls_layout.addWidget(replay_boundary)
        from ..features.log_store import LogBuffer
        self.play_log = LogBuffer("recording", "playback-events")
        # 同录制页: 小屏上可滚动, 不压缩行高
        controls_scroll = QScrollArea()
        controls_scroll.setObjectName("transparentScroll")
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setWidget(controls)
        self.play_controls_scroll = controls_scroll
        enable_touch_scroll(controls_scroll)
        splitter.addWidget(controls_scroll)
        splitter.setSizes([850, 450])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        self._handle_state("play", "stopped")
        return tab

    @staticmethod
    def _scrollable(widget: QWidget) -> QScrollArea:
        area = QScrollArea()
        area.setObjectName("transparentScroll")
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.NoFrame)
        area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        area.setWidget(widget)
        return area

    def set_compact_mode(
        self, compact: bool, portrait: bool | None = None,
    ) -> None:
        """Keep record/replay actions readable on the 800×480 display."""
        compact = bool(compact)
        portrait = (
            compact and self.width() < self.height()
            if portrait is None else compact and bool(portrait))
        self.layout().setContentsMargins(
            *(8, 6, 8, 8) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(7 if compact else 14)
        orientation = Qt.Vertical if portrait else Qt.Horizontal
        self.record_splitter.setOrientation(orientation)
        self.play_splitter.setOrientation(orientation)
        for controls, scroll, controls_layout in (
            (self.record_controls, self.record_controls_scroll,
             self.record_controls_layout),
            (self.play_controls, self.play_controls_scroll,
             self.play_controls_layout),
        ):
            controls.setMinimumWidth(0 if compact else 380)
            controls.setMaximumWidth(16777215 if compact else 520)
            scroll.setMinimumWidth(0 if compact else 380)
            scroll.setMaximumWidth(16777215 if compact else 520)
            controls_layout.setContentsMargins(
                *(8, 7, 8, 7) if compact else (9, 9, 9, 9))
            controls_layout.setSpacing(6 if compact else 6)
        for form in (self.record_form, self.play_form):
            form.setVerticalSpacing(6 if compact else 8)
            form.setHorizontalSpacing(7 if compact else 10)
        self.play_loop.setText(
            "普通回放可循环" if compact else
            "仅普通预览使用；算法模式会自动关闭")
        # Keep operational logs reachable on the device: only compact their
        # height.  Explanatory copy is available through the controls/tooltips.
        for widget in (
            self.record_hint, self.play_warning, self.replay_boundary,
        ):
            widget.setVisible(not compact)
        if portrait:
            self.record_splitter.setSizes([360, 280])
            self.play_splitter.setSizes([360, 280])
        elif compact:
            self.record_splitter.setSizes([400, 380])
            self.play_splitter.setSizes([400, 380])
        else:
            self.record_splitter.setSizes([850, 450])
            self.play_splitter.setSizes([850, 450])

    def refresh_record_topics(self, *, preserve_selection=False) -> None:
        try:
            topics = self.controller.available_topics()
        except (RuntimeError, TypeError, ValueError) as exc:
            self.record_status.setText(f"读取 ROS 2 话题失败：{exc}")
            return
        selected = {(self.record_topics.topLevelItem(index).text(1),
                     self.record_topics.topLevelItem(index).text(2)):
                    self.record_topics.topLevelItem(index).checkState(0)
                    for index in range(self.record_topics.topLevelItemCount())} if preserve_selection else {}
        discovered = {(topic.name, topic.message_type): topic for topic in topics}
        catalog = dict(discovered)
        for topic in BASE_SENSOR_TOPICS:
            catalog.setdefault((topic.name, topic.message_type), topic)
        self.record_topics.clear()
        for key, topic in sorted(catalog.items()):
            online = key in discovered
            item = QTreeWidgetItem((
                "", topic.name, topic.message_type,
                "已发现" if online else "尚未发现"))
            item.setCheckState(
                0, selected.get(key, Qt.Checked
                if online and default_record_selected(topic) else Qt.Unchecked))
            self.record_topics.addTopLevelItem(item)
        self.record_status.setText(
            f"已发现 {len(topics)} 个话题；开始录制前会检查所选数据是否在线。")

    @staticmethod
    def _set_tree_checked(tree: QTreeWidget, checked: bool) -> None:
        state = Qt.Checked if checked else Qt.Unchecked
        for index in range(tree.topLevelItemCount()):
            tree.topLevelItem(index).setCheckState(0, state)

    @staticmethod
    def _checked_topic_names(tree: QTreeWidget) -> list[str]:
        names = []
        for index in range(tree.topLevelItemCount()):
            item = tree.topLevelItem(index)
            if item.checkState(0) == Qt.Checked:
                names.append(item.text(1).strip())
        return list(dict.fromkeys(name for name in names if name))

    def _choose_record_directory(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self, "选择 rosbag 保存目录", self.record_directory.text())
        if selected:
            self.record_directory.setText(selected)

    @staticmethod
    def _size_bytes(text: str, default_mb: int) -> int:
        """把界面上的 MB 输入换算成字节；非法输入回退到默认值。"""
        try:
            value = int(float(str(text).strip()))
        except (TypeError, ValueError):
            value = default_mb
        return max(0, value) * 1000 * 1000

    def _start_recording(self) -> None:
        mode, compression_format = (
            self.compression.currentData() or ("none", "none"))
        success, status = self.controller.start_recording(
            self.record_directory.text(), self.record_name.text(),
            self._checked_topic_names(self.record_topics),
            self.storage_id.currentText(),
            compression_mode=mode,
            compression_format=compression_format,
            max_cache_size=self._size_bytes(self.max_cache_mb.text(), 1000),
            max_bag_size=self._size_bytes(self.max_bag_mb.text(), 1000),
            unique_if_exists=self.unique_if_exists.isChecked(),
            image_storage=self.image_storage.currentData())
        self.record_status.setText("正在检查数据和存储空间…" if success else status)
        self.record_status.setToolTip(status)
        if success:
            self.record_name.setText(
                datetime.now().strftime("nav_bag_%Y%m%d_%H%M%S"))

    def _choose_bag(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self, "选择 rosbag 目录", self.bag_path.text())
        if selected:
            self.load_bag(selected)

    def load_bag(self, path: str | Path) -> bool:
        try:
            metadata = read_bag_metadata(path)
        except ValueError as exc:
            self.play_status.setText(str(exc))
            return False
        self.metadata = metadata
        self.bag_path.setText(str(metadata.path))
        self.play_topics.clear()
        for topic in metadata.topics:
            item = QTreeWidgetItem((
                "", topic.name, topic.message_type,
                f"{topic.message_count:,}", topic.name,
            ))
            item.setCheckState(
                0, Qt.Checked if safe_algorithm_topic(topic) else Qt.Unchecked)
            item.setFlags(item.flags() | Qt.ItemIsEditable)
            item.setData(0, Qt.UserRole, topic)
            self.play_topics.addTopLevelItem(item)
        self.bag_summary.setText(
            f"{metadata.storage_id or '未知格式'} · "
            f"{metadata.duration_seconds:.2f} s · "
            f"{metadata.message_count:,} 条消息 · "
            f"{len(metadata.topics)} 个话题")
        self.play_status.setText('已加载旧格式稀疏采集；可普通回放，不支持重跑建图。'
                                 if metadata.recording_profile == 'keyframes' else
                                 "已加载；默认选择安全的原始传感器输入")
        return True

    def playback_request(
        self, algorithm_only: bool = False,
        start_paused: bool = False,
    ) -> PlaybackRequest:
        if self.metadata is None:
            raise ValueError("请先加载 rosbag")
        if algorithm_only and self.metadata.recording_profile == 'keyframes':
            raise ValueError('旧格式稀疏关键帧不含完整原始输入，无法重跑建图。')
        selected = []
        remaps = []
        for index in range(self.play_topics.topLevelItemCount()):
            item = self.play_topics.topLevelItem(index)
            if item.checkState(0) != Qt.Checked:
                continue
            topic = item.data(0, Qt.UserRole)
            if not isinstance(topic, BagTopic):
                continue
            if algorithm_only and not safe_algorithm_topic(topic):
                continue
            target = item.text(4).strip() or topic.name
            selected.append(topic.name)
            if target != topic.name:
                remaps.append((topic.name, target))
        if not selected:
            raise ValueError("至少选择一个回放话题")
        return PlaybackRequest(
            bag_path=self.metadata.path,
            topics=tuple(selected),
            remaps=tuple(remaps),
            rate=self.play_rate.value(),
            clock_hz=self.clock_hz.value(),
            loop=self.play_loop.isChecked() and not algorithm_only,
            start_paused=start_paused,
        )

    def _start_plain_playback(self) -> None:
        try:
            request = self.playback_request()
        except ValueError as exc:
            self.play_status.setText(str(exc))
            return
        success, status = self.controller.start_playback(request)
        self.play_status.setText(status)

    def _request_mapping(self) -> None:
        try:
            request = self.playback_request(
                algorithm_only=True, start_paused=True)
        except ValueError as exc:
            self.play_status.setText(str(exc))
            return
        self.mapping_requested.emit(request)

    def _request_navigation(self) -> None:
        try:
            request = self.playback_request(
                algorithm_only=True, start_paused=True)
        except ValueError as exc:
            self.play_status.setText(str(exc))
            return
        self.navigation_requested.emit(request)

    def _pause(self) -> None:
        _success, status = self.controller.pause_playback()
        self.play_status.setText(status)

    def _resume(self) -> None:
        _success, status = self.controller.resume_playback()
        self.play_status.setText(status)

    def set_status(self, text: str) -> None:
        self.play_status.setText(text)

    def _append_log(self, kind: str, text: str) -> None:
        view = self.record_log if kind == "record" else self.play_log
        view.appendPlainText(text)

    def _handle_state(self, kind: str, state: str) -> None:
        if kind == "record":
            active = state in ("starting", "running", "stopping", "checking", "flushing", "auditing")
            self.record_start.setEnabled(not active)
            self.record_stop.setEnabled(active and state not in ("flushing", "auditing", "stopping"))
            messages = {"checking": "正在检查传感器数据、内存和磁盘…", "running": "正在录制，持续检查数据与存储空间",
                        "flushing": "正在收尾，请保持供电…", "auditing": "正在逐片校验数据，请保持供电…",
                        "stored": "采集数据已保存，等待后台校验；可以继续采集",
                        "incomplete": "数据不完整，请查看录制日志", "cancelled": "已取消录制准备"}
            if state in messages:
                self.record_status.setText(messages[state])
            if state == "stopping":
                self.record_status.setText(
                    "正在收尾并校验，请保持供电，等待完成…")
        else:
            active = state in ("starting", "running", "stopping")
            self.play_start.setEnabled(not active)
            self.play_stop.setEnabled(active)
            if state == "stopped" and self.play_status.text().startswith("正在回放"):
                self.play_status.setText("rosbag 回放已结束")
            elif state == "error":
                self.play_status.setText("rosbag 回放异常结束，请查看日志")

    def _record_finished(self, metadata) -> None:
        if isinstance(metadata, BagMetadata):
            self.record_status.setText(
                ("已保存，等待校验：" if self.controller.last_record_state == 'stored' else "录制完成：") +
                f"{metadata.duration_seconds:.2f} s，"
                f"{metadata.message_count:,} 条消息" +
                ("；质量提示：" + "；".join(self.controller.last_record_warnings) if self.controller.last_record_warnings else ""))
            self.record_status.setToolTip(str(metadata.path))
        else:
            if self.controller.last_record_state == "cancelled":
                self.record_status.setText("已取消录制准备")
            else:
                reason = self.controller.last_record_error.split("；", 1)[0][:180]
                self.record_status.setText("数据不完整：" + (reason or "请查看录制日志和输出报告"))

    def showEvent(self, event):
        super().showEvent(event)
        self.activate()

    def activate(self) -> None:
        if not self.controller.recording:
            self.refresh_record_topics(preserve_selection=True)

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit, QLabel, QPushButton, QCheckBox, QTabWidget
from ..ros_launch_process import RosLaunchProcess


class RtkPanel(QWidget):
    def __init__(self, adapter=None):
        super().__init__()
        self.adapter = adapter
        self.process = RosLaunchProcess("GNSS 监测", self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        data = QWidget()
        data_layout = QVBoxLayout(data)
        data_layout.setContentsMargins(5, 4, 5, 4)
        data_layout.setSpacing(4)
        self.tabs.addTab(data, "监测数据")
        settings = QWidget()
        form = QFormLayout(settings)
        form.setContentsMargins(5, 4, 5, 4)
        self.topic = QLineEdit("/rtk/fix")
        form.addRow("GNSS 定位话题", self.topic)
        self.sim_time = QCheckBox("使用回放时钟")
        form.addRow(self.sim_time)
        self.tabs.addTab(settings, "监测设置")
        actions = QHBoxLayout()
        start = QPushButton("开始监测")
        self.start_button = start
        start.setObjectName("primaryButton")
        start.clicked.connect(self.start)
        stop = QPushButton("停止监测")
        self.stop_button = stop
        stop.setObjectName("dangerButton")
        stop.clicked.connect(self.process.stop_launch)
        actions.addWidget(start)
        actions.addWidget(stop)
        layout.addLayout(actions)
        self.status = QLabel("等待 GNSS 数据")
        self.status.setWordWrap(True)
        self.position = QLabel("ENU 坐标：等待有效定位")
        self.position.setWordWrap(True)
        self.counts = QLabel()
        self.counts.setWordWrap(True)
        for widget in (self.status, self.position, self.counts):
            widget.setMinimumWidth(0)
            data_layout.addWidget(widget)
        data_layout.addStretch(1)
        self.position.setToolTip("以首个有效定位为 ENU 原点；本页仅监测 GNSS 观测，不代表地图定位成功。")
        from ..features.log_store import LogBuffer
        self.log = LogBuffer(persist=False)
        self.process.log_received.connect(self.log.appendPlainText)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def start(self):
        if not self.topic.text().strip():
            self.status.setText("请填写 GNSS 定位话题")
            return
        self.process.start_launch(["launch", "nav_rtk", "monitor.launch.py",
            f"fix_topic:={self.topic.text().strip()}",
            f"use_sim_time:={'true' if self.sim_time.isChecked() else 'false'}"])

    def refresh(self):
        self.start_button.setEnabled(not self.process.active)
        self.stop_button.setEnabled(self.process.active)
        if self.adapter is None:
            return
        self.status.setText(self.adapter.status)
        self.counts.setText(self.adapter.counts)
        if self.adapter.position is not None:
            e, n, u = self.adapter.position
            self.position.setText(f"最近有效位置：东 {e:.2f} m · 北 {n:.2f} m · 上 {u:.2f} m")

    def shutdown(self):
        self.timer.stop()
        self.process.shutdown()

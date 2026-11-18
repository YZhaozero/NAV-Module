"""Read-only controller setup; motion is enabled separately by the host page."""
import os
import time
from pathlib import Path
from PyQt5.QtCore import QProcess, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QScrollArea, QFrame)
from ..features.gamepad import Calibration, parse_devices, load_profiles, save_profile
from ..ros_launch_process import RosLaunchProcess
from .touch_scroll import enable_touch_scroll


class GamepadPanel(QWidget):
    configuration_changed = pyqtSignal()

    def __init__(self, node, context):
        super().__init__()
        self.node, self.context = node, context
        self.path = Path(os.environ.get("NAV_GAMEPAD_PROFILES", str(Path.home()/".config/nav/gamepads.json")))
        self.selected = self.profile = self.calibration = self.sample = self.subscription = None
        self.last_pressed = []
        self.driver = RosLaunchProcess("外接手柄", self)
        self.driver.log_received.connect(self.driver_log)
        self.discovery = QProcess(self)
        self.discovery.setProcessChannelMode(QProcess.MergedChannels)
        self.discovery.finished.connect(self.devices_found)
        self.discovery.errorOccurred.connect(self.scan_error)
        outer = QVBoxLayout(self)
        row = QHBoxLayout()
        self.devices = QComboBox();self.devices.addItem("正在搜索手柄", None)
        self.devices.currentIndexChanged.connect(self.select_device)
        row.addWidget(self.devices, 1)
        self.refresh_button = QPushButton("重新搜索");self.refresh_button.clicked.connect(self.scan)
        row.addWidget(self.refresh_button);outer.addLayout(row)
        scroll = QScrollArea();scroll.setWidgetResizable(True);scroll.setFrameShape(QFrame.NoFrame)
        enable_touch_scroll(scroll)
        content = QWidget();body = QVBoxLayout(content);body.setSpacing(6)
        self.status = QLabel("连接 USB / 蓝牙手柄后自动搜索；首次使用请标定")
        self.status.setWordWrap(True);self.status.setTextFormat(Qt.PlainText);body.addWidget(self.status)
        self.instruction = QLabel("标定只读取手柄，不向底盘发送移动指令。行驶时必须按住选定保持键。")
        self.instruction.setWordWrap(True);body.addWidget(self.instruction)
        actions = QHBoxLayout()
        self.calibrate_button = QPushButton("开始标定");self.calibrate_button.clicked.connect(self.begin)
        self.center_button = QPushButton("记录中位");self.center_button.clicked.connect(self.record_center)
        self.cancel_button = QPushButton("取消");self.cancel_button.clicked.connect(self.cancel)
        self.save_button = QPushButton("保存标定");self.save_button.clicked.connect(self.save)
        for button in (self.calibrate_button, self.center_button, self.cancel_button, self.save_button):actions.addWidget(button)
        body.addLayout(actions)
        self.summary = QLabel("尚无标定参数");self.summary.setWordWrap(True);self.summary.setTextFormat(Qt.PlainText)
        body.addWidget(self.summary)
        self.live = QLabel("尚未收到手柄输入");self.live.setWordWrap(True);self.live.setTextFormat(Qt.PlainText)
        body.addWidget(self.live)
        self.feedback = QLabel();self.feedback.setWordWrap(True);body.addWidget(self.feedback)
        body.addStretch(1)
        scroll.setWidget(content);outer.addWidget(scroll, 1)
        self.scan_timer = QTimer(self);self.scan_timer.setInterval(2000);self.scan_timer.timeout.connect(self.scan)
        self.update_buttons()

    @property
    def ready(self):
        return self.selected is not None and self.profile is not None and self.calibration is None

    def scan(self):
        if self.isVisible() and self.discovery.state() == QProcess.NotRunning:
            self.discovery.start("ros2", ["run", "joy", "joy_enumerate_devices"])

    def scan_error(self, error):
        if error == QProcess.FailedToStart:
            self.set_devices([])
            self.status.setText("无法启动 ROS 手柄枚举工具："+self.discovery.errorString())

    def devices_found(self, code, status):
        output = bytes(self.discovery.readAllStandardOutput()).decode(errors="replace")
        if code != 0 or status != QProcess.NormalExit:
            self.set_devices([])
            self.status.setText("手柄搜索失败："+output[-300:])
            return
        try:
            self.set_devices(parse_devices(output))
        except ValueError as error:
            self.set_devices([]);self.status.setText(str(error))

    def set_devices(self, devices):
        current = self.selected
        self.devices.blockSignals(True);self.devices.clear()
        for device in devices:self.devices.addItem(f"{device['name']} · 设备 {device['id']}", device)
        if not devices:self.devices.addItem("未发现手柄，请检查 USB / 蓝牙及系统驱动", None)
        selected = next((i for i, device in enumerate(devices) if current is not None and device['guid'] == current['guid'] and device['id'] == current['id']), 0)
        self.devices.setCurrentIndex(selected);self.devices.blockSignals(False)
        if self.devices.currentData() != current or (current is not None and not self.driver.active):self.select_device()

    def disconnect_device(self):
        self.calibration = self.profile = None
        self.configuration_changed.emit()
        self.sample = None;self.last_pressed = []
        self.show_summary()
        self.driver.shutdown()
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription);self.subscription = None

    def select_device(self, *_args):
        self.disconnect_device()
        self.selected, self.profile = self.devices.currentData(), None
        if self.selected is None:
            self.status.setText("未发现手柄；USB 已连接但无输入时请检查系统驱动")
            self.update_buttons();return
        try:
            profiles = load_profiles(self.path)
            self.profile = profiles.get(self.selected["guid"])
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.status.setText("读取手柄标定失败："+str(error));self.update_buttons();return
        from sensor_msgs.msg import Joy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.subscription = self.node.create_subscription(Joy, "/nav/manual_gamepad/joy", self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.driver.start_launch(["run", "joy", "joy_node", "--ros-args", "-r", "__node:=nav_manual_joy",
            "-r", "joy:=/nav/manual_gamepad/joy", "-p", "device_id:="+str(self.selected["id"]),
            "-p", "autorepeat_rate:=20.0", "-p", "deadzone:=0.0"])
        self.status.setText("已加载此手柄标定；回中并松开保持键后，勾选允许遥控" if self.profile else "新手柄已连接，请点击“开始标定”")
        self.configuration_changed.emit();self.show_summary();self.update_buttons()

    def receive(self, message):
        now = time.monotonic()
        self.sample = (tuple(message.axes), tuple(message.buttons), now)
        pressed = [i for i, value in enumerate(message.buttons) if value]
        if pressed:
            self.last_pressed = pressed
        # Process each input event so a short press is not overwritten by release.
        if self.calibration is not None and self.calibration.stage not in ("center", "done"):
            try:
                self.calibration.update(self.sample, now)
                self.instruction.setText(self.calibration.message)
            except ValueError as error:
                self.calibration.candidate = None
                self.instruction.setText(str(error))
            self.update_buttons()

    def begin(self):
        context = self.context()
        if context["chassis"] is None:
            self.status.setText("请先在上方选择实际车型");return
        self.calibration = Calibration(context["chassis"])
        self.status.setText("标定中：仅识别手柄输入，尚未允许底盘移动")
        self.configuration_changed.emit()
        self.instruction.setText(self.calibration.message);self.update_buttons()

    def record_center(self):
        try:
            self.calibration.center(self.sample, time.monotonic())
            self.instruction.setText(self.calibration.message)
        except ValueError as error:self.instruction.setText(str(error))
        self.update_buttons()

    def cancel(self):
        self.calibration = None;self.configuration_changed.emit()
        self.instruction.setText("标定已取消，原保存参数保留；需要重新允许遥控")
        self.update_buttons()

    def save(self):
        try:
            profile = self.calibration.profile(self.context())
            if profile["chassis"] != self.calibration.chassis:
                raise ValueError("车型在标定期间已变化，请重新标定")
            profile["name"] = self.selected["name"]
            save_profile(self.path, self.selected["guid"], profile)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.status.setText("保存标定失败："+str(error));return
        self.profile, self.calibration = profile, None
        self.configuration_changed.emit();self.show_summary();self.update_buttons()
        self.status.setText("标定已保存。下次识别到同一型号会自动加载，仍需勾选允许遥控")
        self.instruction.setText("按住所选保持键移动，松开停止；无需记设备或按钮序号。")

    def show_summary(self):
        if self.profile is None:
            self.summary.setText("尚无标定参数");return
        p = self.profile
        directions = [f"{title}：轴 {p['axes'][i]}（{'正向' if p['signs'][i] > 0 else '反向'}）" for i, title in ((0,"前后"),(2,"转向"))]
        if p["chassis"] == "omni":directions.append(f"侧移：轴 {p['axes'][1]}")
        self.summary.setText(f"保持键：{p['deadman']} · "+" · ".join(directions))

    def tick(self, now):
        if self.sample is not None:
            axes, buttons, stamp = self.sample
            values = " · ".join(f"{i}:{v:+.2f}" for i, v in enumerate(axes[:8]))
            self.live.setText(f"实时轴 {values}\n按下的按钮 {[i for i,v in enumerate(buttons) if v]} · 最近按下 {self.last_pressed} · 数据年龄 {now-stamp:.2f} s")
        if self.calibration is not None and self.calibration.stage not in ("center", "done"):
            if self.sample is None or now-self.sample[2] > .3:
                self.calibration.candidate = None
                self.instruction.setText("手柄数据中断；恢复后可继续，或取消重新标定")
        self.update_buttons()

    def update_buttons(self):
        active = self.calibration is not None
        self.calibrate_button.setEnabled(self.selected is not None and not active)
        self.center_button.setEnabled(active and self.calibration.stage == "center")
        self.cancel_button.setEnabled(active)
        self.save_button.setEnabled(active and self.calibration.stage == "done")

    def driver_log(self, text):
        if "ERROR" in text or "failed" in text.lower():self.status.setText(text.strip()[-300:])

    def showEvent(self, event):
        super().showEvent(event);self.scan_timer.start();self.scan()

    def hideEvent(self, event):
        self.scan_timer.stop()
        self.disconnect_device();self.selected = self.profile = None
        super().hideEvent(event)

    def shutdown(self):
        self.scan_timer.stop()
        if self.discovery.state() != QProcess.NotRunning:
            self.discovery.kill();self.discovery.waitForFinished(1000)
        self.disconnect_device()

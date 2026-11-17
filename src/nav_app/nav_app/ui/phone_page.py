from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .phone_remote import PhoneRemote


class PhoneRemotePage(QWidget):
    def __init__(self, window):
        super().__init__()
        self.window = window
        self.remote = PhoneRemote(window)
        layout = QVBoxLayout(self)
        title = QLabel("手机遥控")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)
        self.status = QLabel("服务未开启")
        self.status.setObjectName("statusBar")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.code = QLabel("配对码：------")
        self.code.setObjectName("sectionTitle")
        layout.addWidget(self.code)
        self.addresses = QLabel("开启后显示手机访问地址")
        self.addresses.setWordWrap(True)
        layout.addWidget(self.addresses)
        actions = QHBoxLayout()
        self.toggle = QPushButton("开启手机遥控")
        self.toggle.setObjectName("primaryButton")
        self.toggle.clicked.connect(self.toggle_server)
        actions.addWidget(self.toggle)
        self.disconnect = QPushButton("断开当前手机")
        self.disconnect.setObjectName("dangerButton")
        self.disconnect.clicked.connect(self.remote.revoke)
        actions.addWidget(self.disconnect)
        layout.addLayout(actions)
        layout.addStretch(1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        if getattr(window.node, "phone_remote_enabled", False):
            self.start()
        self.refresh()

    def start(self):
        self.remote.start(
            self.window.node.phone_remote_bind,
            self.window.node.phone_remote_port)

    def toggle_server(self):
        if self.remote.server:
            self.remote.close()
        else:
            self.start()
        self.refresh()

    def refresh(self):
        state = self.remote.state
        running = self.remote.server is not None
        connected = bool(state and state.controller())
        self.toggle.setText("停止手机遥控" if running else "开启手机遥控")
        self.disconnect.setEnabled(connected)
        if not running:
            self.status.setText(self.remote.error or "服务未开启")
            self.code.setText("配对码：------")
            self.addresses.setText("开启后显示手机访问地址")
            return
        self.status.setText("手机已连接" if connected else "等待手机配对")
        self.code.setText(f"配对码：{state.code}")
        self.addresses.setText("\n".join(self.remote.urls))

    def shutdown(self):
        self.timer.stop()
        self.remote.close()

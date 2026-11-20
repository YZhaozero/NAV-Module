"""Task-oriented system entry and device settings for the NAV desktop."""
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QLabel, QPushButton, QScrollArea, QSizePolicy)

from .page_header import PageHeader
from .touch_scroll import enable_touch_scroll


class SystemHomePage(QWidget):
    mapping_requested = pyqtSignal()
    navigation_requested = pyqtSignal()
    navigation_mode_requested = pyqtSignal()
    maps_requested = pyqtSignal()
    settings_requested = pyqtSignal()
    phone_requested = pyqtSignal()
    drive_requested = pyqtSignal()
    mapping_stop_requested = pyqtSignal()
    navigation_stop_requested = pyqtSignal()
    map_tasks_requested = pyqtSignal()
    mapping_settings_requested = pyqtSignal()

    def __init__(self, *, woosh=False):
        super().__init__()
        self.setObjectName("systemHome")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(10)
        header = QHBoxLayout()
        self.title = QLabel("建图与导航")
        self.title.setObjectName("title")
        header.addWidget(self.title, 1)
        actions = QVBoxLayout()
        self.state = QLabel("机器人 · 待命")
        self.state.setObjectName("homeState")
        self.state.setAlignment(Qt.AlignRight)
        actions.addWidget(self.state)
        self.battery = QLabel("底盘电量 -- · 等待底盘电量")
        self.battery.setObjectName("homeBattery")
        self.battery.setAlignment(Qt.AlignRight)
        self.battery.setWordWrap(True)
        self.battery.setVisible(woosh)
        actions.addWidget(self.battery)
        quick = QHBoxLayout()
        self.phone_button = QPushButton("手机遥控")
        self.phone_button.setObjectName("workspaceButton")
        self.phone_button.clicked.connect(self.phone_requested.emit)
        self.device_button = QPushButton("设备")
        self.device_button.setObjectName("workspaceButton")
        self.device_button.clicked.connect(self.settings_requested.emit)
        self.drive_button = QPushButton("底盘遥控")
        self.drive_button.setObjectName("workspaceButton")
        self.drive_button.clicked.connect(self.drive_requested.emit)
        quick.addWidget(self.phone_button)
        quick.addWidget(self.drive_button)
        quick.addWidget(self.device_button)
        actions.addLayout(quick)
        header.addLayout(actions)
        layout.addLayout(header)
        self.status = QLabel("准备设备后，采集新地图或选择已有地图")
        self.status.setObjectName("homeStatus")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.map_name = QLabel("尚未选择地图")
        self.map_name.setObjectName("homeMapName")
        self.map_name.setTextFormat(Qt.PlainText)
        self.map_name.setWordWrap(True)
        layout.addWidget(self.map_name)
        services = QGridLayout()
        services.setVerticalSpacing(3)
        self.mapping_state = QLabel("建图服务：未启动")
        self.navigation_state = QLabel("导航服务：未启动")
        self.task_state = QLabel("地图后台：正在读取状态")
        self.mapping_stop_button = QPushButton("停止建图")
        self.navigation_stop_button = QPushButton("停止导航")
        self.task_button = QPushButton("查看/暂停")
        self.mapping_settings_button = QPushButton("建图参数")
        self.mapping_settings_button.setObjectName("homeServiceAction")
        self.mapping_settings_button.clicked.connect(self.mapping_settings_requested.emit)
        services.addWidget(self.mapping_settings_button, 0, 1)
        for row, (label, button, signal) in enumerate((
                (self.mapping_state, self.mapping_stop_button, self.mapping_stop_requested),
                (self.navigation_state, self.navigation_stop_button, self.navigation_stop_requested),
                (self.task_state, self.task_button, self.map_tasks_requested))):
            label.setObjectName("homeService")
            label.setTextFormat(Qt.PlainText)
            button.setObjectName("homeServiceAction")
            button.clicked.connect(signal.emit)
            services.addWidget(label, row, 0, 1, 1 if row == 0 else 2)
            services.addWidget(button, row, 2)
        services.setColumnStretch(0, 1)
        layout.addLayout(services)
        self.prompt = QLabel("采集新地图，或使用已有地图导航")
        self.prompt.setObjectName("homePrompt")
        self.prompt.setWordWrap(True)
        layout.addWidget(self.prompt, 1)
        self.mapping_button = QPushButton("开始建图")
        self.mapping_button.setObjectName("homePrimary")
        self.mapping_button.clicked.connect(self.mapping_requested.emit)
        layout.addWidget(self.mapping_button)
        self.navigation_button = QPushButton("进入导航")
        self.navigation_button.setObjectName("homeNavigation")
        self.navigation_button.clicked.connect(self.navigation_requested.emit)
        navigation_row = QHBoxLayout()
        navigation_row.addWidget(self.navigation_button, 2)
        self.navigation_mode_button = QPushButton("导航模式")
        self.navigation_mode_button.setObjectName("homeNavigation")
        self.navigation_mode_button.setVisible(woosh)
        self.navigation_mode_button.clicked.connect(self.navigation_mode_requested.emit)
        navigation_row.addWidget(self.navigation_mode_button, 1)
        layout.addLayout(navigation_row)
        bottom = QHBoxLayout()
        self.maps_button = QPushButton("地图管理")
        self.maps_button.setObjectName("homeTile")
        self.maps_button.clicked.connect(self.maps_requested.emit)
        self.settings_button = QPushButton("设备与设置")
        self.settings_button.setObjectName("homeTile")
        self.settings_button.clicked.connect(self.settings_requested.emit)
        bottom.addWidget(self.maps_button)
        bottom.addWidget(self.settings_button)
        layout.addLayout(bottom)
        self.set_compact_mode(False)

    def set_compact_mode(self, compact):
        self.layout().setContentsMargins(*(12, 8, 12, 10) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(5 if compact else 10)
        self.prompt.setVisible(not compact)
        self.mapping_button.setMinimumHeight(60 if compact else 82)
        for button in (self.navigation_button, self.navigation_mode_button):
            button.setMinimumHeight(44 if compact else 56)
        for button in (self.maps_button, self.settings_button):
            button.setMinimumHeight(48 if compact else 62)
        self.setStyleSheet("""
            QWidget#systemHome { background: #0c1921; }
            QLabel { background: transparent; }
            QLabel#title { font-size: 24px; }
            QLabel#homeState { color: #ffd166; font-weight: 650; font-size: 18px; }
            QLabel#homeMapName { color: #9fb9c7; font-size: 14px; }
            QLabel#homeService { font-size: 14px; color: #b9d8e8; }
            QPushButton#homeServiceAction { min-height: 25px; font-size: 13px; padding: 0 10px; }
            QLabel#homePrompt { font-size: 20px; font-weight: 650; }
            QLabel#homeStatus { background: #243543; color: #eaf0f4; font-size: 16px;
                border: 1px solid #526978; border-radius: 9px; padding: 10px 12px; }
            QLabel#homeStatus[blocked="true"] { color: #ffd166; border-color: #b9832e; }
            QPushButton#workspaceButton { font-size: 14px; min-height: 34px; padding: 0 12px; }
            QPushButton#homePrimary { background: #00a9df; color: #041018;
                min-height: %dpx; border: 1px solid #00a9df; font-weight: 700; font-size: 21px; }
            QPushButton#homePrimary:hover { background: #25c2f2; }
            QPushButton#homePrimary:pressed { background: #0084b1; }
            QPushButton#homeNavigation { font-size: 18px; min-height: %dpx; }
            QPushButton#homeTile { font-size: 18px; min-height: %dpx; }
        """ % (60 if compact else 82, 44 if compact else 56, 52 if compact else 62))



class SystemSettingsPage(QWidget):
    def __init__(self, window):
        super().__init__()
        self.setObjectName("systemSettings")
        layout = QVBoxLayout(self)
        self.header = PageHeader("NAV · 设备与设置", window.show_system_home,
            destination="返回系统首页", subtitle="设备连接、地图资产与系统维护")
        layout.addLayout(self.header)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QScrollArea.NoFrame)
        self.scroll.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        enable_touch_scroll(self.scroll)
        content = QWidget()
        content.setMaximumWidth(1180)
        content.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        body = QVBoxLayout(content)
        self.content_layout = body
        body.setContentsMargins(0, 4, 0, 12)
        body.setSpacing(20)
        self.buttons = {}
        self.groups = []
        sections = [
            ("设备与导航", [
                ("sensors", "传感器", "雷达、相机、IMU 与 RTK", window.show_sensor_tools),
                ("navigation", "导航系统", "里程计、定位与路径规划", window.show_navigation_stack),
                ("navigation_access", "导航接入", "驱动、已有话题与算法来源", window.show_navigation_access),
                ("phone", "手机遥控", "手机连接与网络设置", window.show_phone_remote),
                ("manual_drive", "底盘遥控", "车型、速度、屏幕摇杆与外接手柄", window.show_manual_drive),
            ]),
            ("地图与采集", [
                ("maps", "地图管理", "地图包、地图处理与任务进度", window.show_map_library),
                ("recordings", "录包与回放", "数据采集、回放与录包管理", window.show_recordings),
            ]),
            ("系统维护", [
                ("logs", "日志管理", "查看、导出与清理运行日志", window.show_logs),
                ("diagnostics", "系统诊断", "设备状态与诊断报告", window.show_diagnostics),
            ]),
        ]
        for title, entries in sections:
            label = QLabel(title)
            label.setObjectName("settingsGroup")
            body.addWidget(label)
            grid = QGridLayout()
            grid.setSpacing(12)
            keys = []
            for key, title, detail, callback in entries:
                button = QPushButton()
                button.setObjectName("settingsCard")
                button.setAccessibleName(title)
                button.setToolTip(detail)
                button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                card = QHBoxLayout(button)
                card.setContentsMargins(18, 14, 18, 14)
                texts = QVBoxLayout()
                texts.setSpacing(5)
                heading = QLabel(title)
                heading.setObjectName("settingsCardTitle")
                heading.setAttribute(Qt.WA_TransparentForMouseEvents)
                description = QLabel(detail)
                description.setObjectName("settingsCardDetail")
                description.setWordWrap(True)
                description.setAttribute(Qt.WA_TransparentForMouseEvents)
                texts.addWidget(heading)
                texts.addWidget(description)
                card.addLayout(texts, 1)
                arrow = QLabel("›")
                arrow.setObjectName("settingsCardArrow")
                arrow.setAttribute(Qt.WA_TransparentForMouseEvents)
                card.addWidget(arrow)
                button.clicked.connect(callback)
                self.buttons[key] = button
                keys.append(key)
            self.groups.append((grid, keys))
            body.addLayout(grid)
        body.addStretch(1)
        self.scroll.setWidget(content)
        layout.addWidget(self.scroll, 1)
        footer = QHBoxLayout()
        self.fullscreen_button = QPushButton("全屏 / 窗口")
        self.fullscreen_button.clicked.connect(window.toggle_fullscreen)
        self.exit_button = QPushButton("退出程序")
        self.exit_button.setObjectName("dangerButton")
        self.exit_button.clicked.connect(window.close)
        footer.addWidget(self.fullscreen_button)
        footer.addStretch(1)
        footer.addWidget(self.exit_button)
        layout.addLayout(footer)
        self.set_compact_mode(False)

    def set_compact_mode(self, compact, *, portrait=False):
        self.layout().setContentsMargins(*(10, 6, 10, 8) if compact else (28, 20, 28, 22))
        self.content_layout.setSpacing(8 if compact else 20)
        for grid, keys in self.groups:
            columns = 1 if portrait else min(3, len(keys))
            for index, key in enumerate(keys):
                button = self.buttons[key]
                grid.removeWidget(button)
                grid.addWidget(button, index // columns, index % columns)
                button.layout().setContentsMargins(*(14, 10, 14, 10) if compact else (18, 14, 18, 14))
        self.setStyleSheet("""
            QWidget#systemSettings { background: #0b121a; }
            QLabel#settingsGroup { color: #95afc0; font-size: %dpx; font-weight: 650; }
            QPushButton#settingsCard { background: #142330; border: 1px solid #30495b;
                border-radius: 12px; padding: 0; min-height: %dpx; }
            QPushButton#settingsCard:hover, QPushButton#settingsCard:focus {
                background: #1b3040; border-color: #00a9df; }
            QPushButton#settingsCard:pressed { background: #0e1d28; }
            QLabel#settingsCardTitle { background: transparent; color: #ecf4f8;
                font-size: %dpx; font-weight: 700; }
            QLabel#settingsCardDetail { background: transparent; color: #99b2c2; font-size: %dpx; }
            QLabel#settingsCardArrow { background: transparent; color: #00a9df; font-size: 30px; }
        """ % (14 if compact else 16, 76 if compact else 112,
                   20 if compact else 21, 13 if compact else 14))

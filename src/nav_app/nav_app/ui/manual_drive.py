"""WOOSH manual driving page with screen stick and ROS Joy input."""
import ipaddress
import math
from PyQt5.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QTabWidget, QVBoxLayout, QWidget)
from ..features.manual_drive import ManualDrive, WooshTeleopRos
from ..ros_launch_process import RosLaunchProcess
from .page_header import PageHeader
from .gamepad import GamepadPanel


class VirtualStick(QWidget):
    moved = pyqtSignal(float, float, bool)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(180,180)
        self.setAttribute(Qt.WA_AcceptTouchEvents)
        self.position = QPointF()
        self.held = False

    def move_stick(self, point):
        radius = max(1., min(self.width(),self.height())*.38)
        vector = (point-QPointF(self.width()/2,self.height()/2))/radius
        length = math.hypot(vector.x(),vector.y())
        self.position = vector/max(1.,length)
        self.held = True
        self.moved.emit(self.position.x(),-self.position.y(),True)
        self.update()

    def reset(self):
        self.position = QPointF()
        self.held = False
        self.moved.emit(0.,0.,False)
        self.update()

    def mousePressEvent(self,event):
        if event.button()==Qt.LeftButton:
            self.move_stick(event.localPos());event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self,event):
        if self.held:
            self.move_stick(event.localPos());event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self,event):
        if event.button()==Qt.LeftButton:
            self.reset();event.accept()
        else:
            super().mouseReleaseEvent(event)

    def event(self,event):
        if event.type() in (QEvent.TouchBegin,QEvent.TouchUpdate):
            self.move_stick(event.touchPoints()[0].pos());event.accept();return True
        if event.type() in (QEvent.TouchEnd,QEvent.TouchCancel):
            self.reset();event.accept();return True
        return super().event(event)

    def paintEvent(self,event):
        del event
        painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing)
        center=QPointF(self.width()/2,self.height()/2)
        radius=min(self.width(),self.height())*.38
        painter.setPen(QPen(QColor("#365a70"),2));painter.setBrush(QColor("#112632"))
        painter.drawEllipse(center,radius,radius)
        painter.drawLine(center+QPointF(-radius,0),center+QPointF(radius,0))
        painter.drawLine(center+QPointF(0,-radius),center+QPointF(0,radius))
        painter.setPen(QColor("#a8c8d9"))
        painter.drawText(QRectF(0,0,self.width(),24),Qt.AlignCenter,"前进")
        painter.drawText(QRectF(0,self.height()-24,self.width(),24),Qt.AlignCenter,"后退")
        painter.setBrush(QColor("#00a9df"));painter.setPen(Qt.NoPen)
        painter.drawEllipse(center+self.position*radius,23,23)


class ManualDrivePage(QWidget):
    def __init__(self,window):
        super().__init__()
        self.host=window
        self.policy=ManualDrive(self.busy)
        self.agent_process=RosLaunchProcess("WOOSH 遥控 Agent",self)
        layout=QVBoxLayout(self);layout.setSpacing(7)
        self.header=PageHeader("NAV · 底盘遥控",window.show_system_home,
            destination="返回系统首页")
        layout.addLayout(self.header)
        link=QHBoxLayout()
        link.addWidget(QLabel("底盘 IP"))
        self.ip=QLineEdit("169.254.128.2");self.ip.setMaximumWidth(170)
        self.ip.setToolTip("此地址用于启动新的 Agent；连接现有 Agent 时，以其启动参数为准")
        link.addWidget(self.ip)
        self.start_agent_button=QPushButton("启动 Agent")
        self.start_agent_button.clicked.connect(self.start_agent)
        link.addWidget(self.start_agent_button)
        self.connect_button=QPushButton("检查连接")
        self.connect_button.setToolTip("连接现有 /woosh_robot Agent，仅请求状态，不改变控制模式")
        self.connect_button.clicked.connect(self.connect_agent)
        link.addWidget(self.connect_button)
        link.addStretch(1)
        settings=QHBoxLayout()
        settings.addWidget(QLabel("车型"))
        self.chassis=QComboBox()
        for title,key in [("请选择实际车型",None),("阿克曼","ackermann"),("全向","omni"),("差速","differential")]:
            self.chassis.addItem(title,key)
        settings.addWidget(self.chassis,1)
        settings.addWidget(QLabel("直线速度"))
        self.linear=QDoubleSpinBox();self.linear.setRange(0,.5);self.linear.setSingleStep(.05)
        self.linear.setValue(.15);self.linear.setSuffix(" m/s");settings.addWidget(self.linear)
        settings.addWidget(QLabel("转动速度"))
        self.angular=QDoubleSpinBox();self.angular.setRange(0,1.);self.angular.setSingleStep(.05)
        self.angular.setValue(.30);self.angular.setSuffix(" rad/s");settings.addWidget(self.angular)
        layout.addLayout(settings)
        self.link_status=QLabel("未连接厂家 Agent")
        self.link_status.setTextFormat(Qt.PlainText);self.link_status.setWordWrap(True)
        layout.addWidget(self.link_status)
        self.tabs=QTabWidget();layout.addWidget(self.tabs,1)
        screen=QWidget();body=QHBoxLayout(screen)
        self.stick=VirtualStick();self.stick.moved.connect(self.policy.screen_input)
        body.addWidget(self.stick,1)
        help_box=QVBoxLayout()
        self.horizontal=QComboBox()
        self.horizontal.addItem("摇杆横向：转向 / 旋转",False)
        self.horizontal.addItem("摇杆横向：左右平移（全向）",True)
        help_box.addWidget(self.horizontal)
        help_label=QLabel("选择车型并允许遥控后，按住摇杆移动。\n松开即请求零速，离开页面取消遥控。\n阿克曼禁止侧移及零速原地转向。\n车型选择约束指令，不改变实际底盘硬件。")
        help_label.setWordWrap(True);help_box.addWidget(help_label,1)
        self.requested=QLabel("请求：X 0.00 · Y 0.00 m/s · 转动 0.00 rad/s")
        self.requested.setWordWrap(True);help_box.addWidget(self.requested)
        self.measured=QLabel("实测：等待速度反馈")
        self.measured.setWordWrap(True);help_box.addWidget(self.measured)
        body.addLayout(help_box,2)
        self.tabs.addTab(screen,"屏幕摇杆")
        self.gamepad=GamepadPanel(window.node, lambda: dict(chassis=self.chassis.currentData(),
            linear=self.linear.value(), angular=self.angular.value()))
        self.tabs.addTab(self.gamepad,"外接手柄")
        connection=QWidget();connection_layout=QVBoxLayout(connection)
        connection_layout.addLayout(link)
        explanation=QLabel("""1. 启动厂家 Agent，或检查现有连接。
2. 选择实际车型和速度上限，无需切换底盘模式。
3. 切到屏幕摇杆或外接手柄页，勾选允许遥控。
连接检查只读取状态；勾选允许遥控后才能移动。""")
        explanation.setWordWrap(True);connection_layout.addWidget(explanation,1)
        self.process_status=QLabel("厂家服务：/woosh_robot/robot/Twist · 软件零速不替代车身急停")
        self.process_status.setWordWrap(True);self.process_status.setTextFormat(Qt.PlainText)
        connection_layout.addWidget(self.process_status)
        self.tabs.addTab(connection,"连接")
        self.tabs.setCurrentIndex(2)
        self.message=QLabel(self.policy.message);self.message.setWordWrap(True)
        self.message.setTextFormat(Qt.PlainText);self.message.setObjectName("statusBar")
        layout.addWidget(self.message)
        footer=QHBoxLayout()
        self.allow=QCheckBox("允许遥控当前 Agent 的底盘")
        self.allow.toggled.connect(self.allow_changed);footer.addWidget(self.allow)
        self.stop_button=QPushButton("停止遥控 · 发零速")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.clicked.connect(lambda: self.stop("已请求零速并取消遥控",force=True))
        footer.addWidget(self.stop_button,1);layout.addLayout(footer)
        for widget in (self.chassis,self.horizontal):
            widget.currentIndexChanged.connect(self.settings_changed)
        for widget in (self.linear,self.angular):
            widget.valueChanged.connect(self.settings_changed)
        self.gamepad.configuration_changed.connect(self.gamepad_configuration_changed)
        self.tabs.currentChanged.connect(self.source_changed)
        self.agent_process.log_received.connect(self.process_log)
        window.installEventFilter(self)
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(50)
        self.settings_changed();self.set_compact_mode(True);self.tick()

    def busy(self):
        return "导航后台正在运行，请先在首页停止导航" if self.host._navigation_service_active() else ""

    def settings_changed(self,*_args):
        if self.policy.armed:
            self.policy.stop("参数已变化，请重新允许遥控")
        self.stick.reset()
        self.policy.chassis=self.chassis.currentData()
        self.policy.linear,self.policy.angular=self.linear.value(),self.angular.value()
        if self.policy.chassis!="omni":
            self.horizontal.blockSignals(True);self.horizontal.setCurrentIndex(0);self.horizontal.blockSignals(False)
        self.horizontal.setEnabled(self.policy.chassis=="omni" and not self.policy.armed)
        self.policy.strafe=bool(self.horizontal.currentData())


    def source_changed(self,index):
        self.stop("输入方式已切换，请重新允许遥控")
        if index in (0,1):self.policy.source="screen" if index==0 else "gamepad"

    def connect_agent(self):
        if self.policy.transport is not None:
            return True
        try:
            self.policy.transport=WooshTeleopRos(self.host.node)
        except (ImportError,OSError,RuntimeError) as error:
            self.policy.message="SDK 连接失败，请先安装厂家 ARM64 包："+str(error)
            return False
        self.policy.message="正在只读检查 Agent 状态，尚未允许运动"
        return True

    def start_agent(self):
        try:
            address=str(ipaddress.IPv4Address(self.ip.text().strip()))
        except ValueError as error:
            self.policy.message="底盘 IP 无效："+str(error);return
        if not self.connect_agent():return
        if self.agent_process.active or self.policy.transport.clients["Twist"].service_is_ready():
            self.policy.message="Agent 已运行，使用现有连接";return
        self.agent_process.start_launch(["run","woosh_robot_agent","agent","--ros-args",
            "-r","__ns:=/woosh_robot","-p","ip:="+address])

    def gamepad_configuration_changed(self):
        self.stop("手柄配置或连接已变化，请重新允许遥控")
        profile=self.gamepad.profile
        if profile is not None and self.gamepad.calibration is None:
            for widget in (self.chassis,self.linear,self.angular):widget.blockSignals(True)
            self.chassis.setCurrentIndex(self.chassis.findData(profile["chassis"]))
            self.linear.setValue(profile["linear"]);self.angular.setValue(profile["angular"])
            for widget in (self.chassis,self.linear,self.angular):widget.blockSignals(False)
            self.policy.axes=tuple(profile["axes"]);self.policy.deadman=profile["deadman"]
            self.policy.signs=tuple(profile["signs"]);self.policy.centers=tuple(profile["centers"])
            self.settings_changed()

    def allow_changed(self,checked):
        if checked:self.policy.arm()
        else:self.policy.stop("遥控许可已取消")
        self.tick()

    def stop(self,reason,force=False):
        self.policy.stop(reason,force)
        self.stick.reset()
        self.allow.blockSignals(True);self.allow.setChecked(False);self.allow.blockSignals(False)

    def process_log(self,text):
        self.process_status.setText(text.strip()[-180:])

    def tick(self):
        self.gamepad.tick(self.policy.clock())
        self.policy.joy=self.gamepad.sample
        self.policy.tick();health=self.policy.health()
        modes={0:"未定义",1:"自动",2:"手动",3:"维护"}
        mode=modes[health["mode"]] if health["ready"] else "未知"
        estop=("未触发" if health["estop_released"] else "已触发") if health["ready"] else "未知"
        self.link_status.setText(health["reason"]+" · 模式："+mode+" · 急停："+estop)
        message=self.policy.message
        self.message.setText(message)
        self.allow.blockSignals(True);self.allow.setChecked(self.policy.armed);self.allow.blockSignals(False)
        self.allow.setEnabled(self.policy.armed or (self.tabs.currentIndex() in (0,1) and health["ready"] and
            health["estop_released"] and self.policy.chassis is not None and not self.busy() and
            (self.tabs.currentIndex()==0 or self.gamepad.ready)))
        for widget in (self.chassis,self.linear,self.angular,self.connect_button,self.start_agent_button):
            widget.setEnabled(not self.policy.armed)
        self.horizontal.setEnabled(self.policy.chassis=="omni" and not self.policy.armed)
        vx,vy,w=self.policy.last_command
        self.requested.setText(f"请求：X {vx:+.2f} · Y {vy:+.2f} m/s · 转动 {w:+.2f} rad/s")
        measured=health["measured"]
        self.measured.setText("实测：等待速度反馈" if measured is None else
            f"实测：X {measured[0]:+.2f} m/s · 转动 {measured[1]:+.2f} rad/s\nSDK 未提供侧向速度反馈")
        self.gamepad.feedback.setText(self.requested.text()+"\n"+self.measured.text())

    def hideEvent(self,event):
        if self.policy.armed or any(self.policy.last_command):self.stop("已离开遥控页面，发送零速")
        super().hideEvent(event)

    def eventFilter(self,watched,event):
        if watched is self.host and event.type()==QEvent.WindowDeactivate and self.policy.armed:
            self.stop("窗口失去焦点，遥控已取消")
        return super().eventFilter(watched,event)

    def set_compact_mode(self,compact,*,portrait=False):
        del portrait
        self.layout().setContentsMargins(*(8,6,8,8) if compact else (22,18,22,22))
        self.layout().setSpacing(4 if compact else 7)
        self.setStyleSheet("QLabel { font-size: 13px; } QComboBox,QLineEdit,QSpinBox,QDoubleSpinBox { min-width: 0; min-height: 26px; font-size: 13px; } QPushButton { min-height: 30px; padding: 0 8px; font-size: 13px; }")
        self.stick.setMinimumSize(135 if compact else 180,135 if compact else 180)
        self.stop_button.setMinimumHeight(40 if compact else 48)

    def shutdown(self):
        self.host.removeEventFilter(self)
        self.stop("正在关闭遥控")
        self.timer.stop()
        self.gamepad.shutdown();self.agent_process.shutdown()
        if self.policy.transport is not None:
            self.policy.transport.close();self.policy.transport=None

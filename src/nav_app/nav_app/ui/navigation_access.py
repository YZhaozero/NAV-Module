"""Persistent input and backend settings for each robot profile."""
import os
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QLabel, QPushButton, QComboBox, QLineEdit, QCheckBox, QFileDialog, QFrame, QScrollArea)
from .page_header import PageHeader
from .touch_scroll import enable_touch_scroll


class NavigationAccessPage(QWidget):
    def __init__(self, stack_page, back, robot_profile="", parent=None):
        super().__init__(parent)
        self.stack_page = stack_page
        self.profile = robot_profile or "default"
        path = os.environ.get("NAV_ACCESS_SETTINGS")
        self.settings = QSettings(path, QSettings.IniFormat) if path else QSettings("NAV", "NavigationAccess")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 16, 22, 18)
        self.header = PageHeader("NAV · 导航接入设置", back, destination="返回导航系统",
            subtitle="选择导航后端、传感器输入和定位方式")
        layout.addLayout(self.header)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        enable_touch_scroll(self.scroll)
        self.scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        body = QVBoxLayout(content)
        body.setContentsMargins(0, 4, 0, 4)
        presets = QHBoxLayout()
        presets.addWidget(QLabel("接入预设"))
        self.presets = QComboBox()
        self.presets.addItem("WOOSH · 桥接传感器 + FAST-LIVO", "woosh")
        if self.profile == "woosh":
            self.presets.addItem("WOOSH · .5 雷达＋IMU + 相机融合", "woosh_native5")
        self.presets.addItem("已有 scan + 里程计 · AMCL 定位", "external")
        self.presets.addItem("本机驱动 + 点云处理", "local")
        presets.addWidget(self.presets, 1)
        preset_button = QPushButton("载入预设")
        preset_button.clicked.connect(lambda: self.load_preset(self.presets.currentData()))
        presets.addWidget(preset_button)
        body.addLayout(presets)
        self.pipeline = QLabel()
        self.pipeline.setStyleSheet("background: #143346; color: #8ee3fa; padding: 14px; border-radius: 9px; font-size: 18px; font-weight: 650;")
        self.pipeline.setWordWrap(True)
        body.addWidget(self.pipeline)
        self.cards_grid = QGridLayout()
        self.cards_grid.setSpacing(12)
        body.addLayout(self.cards_grid)
        self.cards = []
        self.fields = {}
        self.rows = {}
        definitions = [
            ("01  传感器驱动", "已有 ROS 话题时直接接入，避免重复启动驱动。", [
                ("lidar_driver_mode", "雷达驱动", [("使用已有话题", "external"), ("由 NAV 启动", "nav")]),
                ("lidar_topic", "点云输入", "/foo/livox/lidar_192_168_128_3"),
                ("imu_topic", "IMU 输入", "/foo/livox/imu_192_168_128_3"),
                ("camera_driver_mode", "相机驱动", [("使用已有话题", "external"), ("由 NAV 启动", "nav"), ("不使用", "off")]),
                ("image_topic", "视觉算法图像", "/foo/camera_1/color/image_raw")]),
            ("02  避障数据", "已有融合 scan 可直接供 Nav2 使用。", [
                ("scan_source", "LaserScan 来源", [("使用已有 LaserScan", "external"), ("由 NAV 将点云转换", "convert")]),
                ("scan_topic", "LaserScan 话题", "/foo/scan"),
                ("laser_target_frame", "转换目标坐标系", "base_footprint")]),
            ("03  里程计", "已有里程计与 odom→base_link TF 时可跳过 LIO。", [
                ("odometry_backend", "里程计来源", [("NAV · FAST-LIVO2", "fast_livo"), ("NAV · DLIO", "dlio"), ("使用已有里程计与 TF", "external")]),
                ("external_odom_topic", "已有 Odometry", "/foo/odom"),
                ("fast_livo_lidar_topic", "FAST-LIVO 雷达话题", "/foo/livox/lidar_192_168_128_3"),
                ("woosh_fast_livo_profile", "WOOSH 雷达安装", [(".3 雷达（桥接）", "livox3"), (".5 雷达（本机采集）", "native5")]),
                ("fast_livo_vision", "FAST-LIVO 视觉", True)]),
            ("04  地图定位", "每段 TF 只保留一个来源，定位必须与所选地图一致。", [
                ("localization_backend", "定位来源", [("NAV · 点云 Localizer", "pointcloud_localizer"), ("Nav2 · AMCL", "amcl"), ("使用已有 map→odom TF", "external")])]),
        ]
        definitions.insert(0, ("导航后端", "", [
            ("navigation_engine", "规划后端", [("MeshNav · 三维网格", "meshnav"), ("Nav2 · 栅格", "nav2")]),
            ("meshnav_map", "三角网格 PLY", "")]))
        for title, detail, fields in definitions:
            card = QFrame()
            card.setObjectName("accessCard")
            box = QVBoxLayout(card)
            heading = QLabel(title)
            heading.setStyleSheet("font-size: 20px; font-weight: 700; color: #edf6fb;")
            box.addWidget(heading)
            hint = QLabel(detail)
            hint.setWordWrap(True)
            hint.setStyleSheet("color: #9ab4c5; font-size: 14px;")
            if detail:
                box.addWidget(hint)
            form = QFormLayout()
            form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
            form.setSpacing(10)
            for key, label, options in fields:
                if isinstance(options, list):
                    widget = QComboBox()
                    for text, value in options:
                        widget.addItem(text, value)
                    widget.currentIndexChanged.connect(self.refresh_summary)
                elif isinstance(options, bool):
                    widget = QCheckBox("使用相机参与里程计估计")
                    widget.toggled.connect(self.refresh_summary)
                else:
                    widget = QLineEdit()
                    widget.setMinimumWidth(0)
                    widget.textChanged.connect(self.refresh_summary)
                self.fields[key] = widget
                field = widget
                if key == "meshnav_map":
                    widget.setPlaceholderText("留空自动查找同名 .mesh.ply")
                    browse = QPushButton("选择")
                    browse.clicked.connect(self.choose_mesh_map)
                    field = QWidget()
                    row = QHBoxLayout(field)
                    row.setContentsMargins(0, 0, 0, 0)
                    row.addWidget(widget, 1)
                    row.addWidget(browse)
                form.addRow(label, field)
                self.rows[key] = (form, field)
            box.addLayout(form)
            box.addStretch(1)
            self.cards.append(card)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet("background: #102530; color: #b1d6e6; padding: 14px; border-radius: 9px; font-size: 15px;")
        body.addWidget(self.summary)
        self.contract = QLabel("标准坐标链：map → odom → base_link。外部里程计需要提供对应 TF；相机仅作预览时不阻塞导航。底盘执行保持原配置，本页不会授权运动。")
        self.contract.setWordWrap(True)
        self.contract.setStyleSheet("color: #91aaba; font-size: 14px;")
        body.addWidget(self.contract)
        body.addStretch(1)
        self.scroll.setWidget(content)
        layout.addWidget(self.scroll, 1)
        footer = QHBoxLayout()
        self.status = QLabel()
        self.status.setWordWrap(True)
        footer.addWidget(self.status, 1)
        reset = QPushButton("撤销未保存修改")
        reset.clicked.connect(lambda: self.set_values(self.applied))
        footer.addWidget(reset)
        self.save_button = QPushButton("保存接入设置")
        self.save_button.setObjectName("primaryButton")
        self.save_button.clicked.connect(self.save)
        footer.addWidget(self.save_button)
        layout.addLayout(footer)
        self.setStyleSheet("QFrame#accessCard { background: #14212d; border: 1px solid #304758; border-radius: 10px; } QFrame#accessCard QLabel { border: none; }")
        self.applied = self.preset("woosh" if self.profile == "woosh" else "local")
        if self.profile != "woosh":
            self.applied["odometry_backend"] = stack_page.odometry_backend.currentData()
            self.applied["fast_livo_vision"] = stack_page.fast_livo_vision.isChecked()
        saved = self.settings.value("profiles/" + self.profile, {})
        self.applied.update(saved)
        self.set_values(self.applied)
        self.stack_page.apply_access_settings(self.applied)
        self.stack_page.controller.overall_state_changed.connect(self.update_running)
        self.update_running(self.stack_page.controller.overall_state)

    @staticmethod
    def preset(key):
        native5 = key == "woosh_native5"
        woosh = key in ("woosh", "woosh_native5")
        external = key == "external"
        prefix = "/foo" if woosh or external else ""
        return dict(preset=key, navigation_engine="meshnav" if woosh else "nav2", meshnav_map="",
            lidar_driver_mode="nav" if native5 else ("external" if woosh or external else "nav"),
            camera_driver_mode="off" if external else ("external" if woosh else "nav"),
            scan_source="convert" if native5 else ("external" if woosh or external else "convert"),
            odometry_backend="external" if external else "fast_livo",
            localization_backend="amcl" if external else "pointcloud_localizer",
            woosh_fast_livo_profile="native5" if native5 else "livox3",
            lidar_topic="/livox/lidar_192_168_128_5" if native5 else ("/foo/livox/lidar_192_168_128_3" if woosh else "/livox/lidar/pointcloud"),
            fast_livo_lidar_topic="/livox/lidar_192_168_128_5" if native5 else ("/foo/livox/lidar_192_168_128_3" if woosh else "/livox/lidar"),
            imu_topic="/livox/imu_192_168_128_5" if native5 else ("/foo/livox/imu_192_168_128_3" if woosh else "/livox/imu"),
            image_topic="/foo/camera_1/color/image_raw" if woosh else "/camera/camera/color/image_raw",
            scan_topic="/scan" if native5 else prefix + "/scan", external_odom_topic=prefix + "/odom",
            fast_livo_vision=not external, laser_target_frame="base_footprint" if woosh else "base_link")

    def choose_mesh_map(self):
        field = self.fields["meshnav_map"]
        path, _ = QFileDialog.getOpenFileName(
            self, "选择三角网格", field.text() or str(self.stack_page.map_storage_dir), "三角网格 (*.ply)")
        if path:
            field.setText(path)

    def load_preset(self, key):
        self.set_values(self.preset(key))
        self.status.setText("预设已载入，保存后用于下次启动。")

    def set_values(self, values):
        self.presets.setCurrentIndex(self.presets.findData(values.get("preset", "woosh" if self.profile == "woosh" else "local")))
        for key, widget in self.fields.items():
            widget.blockSignals(True)
            if isinstance(widget, QComboBox): widget.setCurrentIndex(widget.findData(values[key]))
            elif isinstance(widget, QCheckBox): widget.setChecked(values[key])
            else: widget.setText(values[key])
            widget.blockSignals(False)
        self.refresh_summary()

    def values(self):
        return dict(preset=self.presets.currentData(), **{key: (widget.currentData() if isinstance(widget, QComboBox) else
                     widget.isChecked() if isinstance(widget, QCheckBox) else widget.text().strip())
                for key, widget in self.fields.items()})

    def refresh_summary(self, *_):
        value = self.values()
        odom = value["odometry_backend"]
        mesh = value["navigation_engine"] == "meshnav"
        backend = self.fields["navigation_engine"].currentText()
        self.pipeline.setText(f"传感器与话题   →   里程计与定位   →   {backend}")
        visible = dict(external_odom_topic=odom == "external", fast_livo_vision=odom == "fast_livo",
            fast_livo_lidar_topic=odom == "fast_livo",
            woosh_fast_livo_profile=self.profile == "woosh" and odom == "fast_livo",
            meshnav_map=mesh, scan_source=not mesh, scan_topic=not mesh,
            laser_target_frame=not mesh and value["scan_source"] == "convert",
            imu_topic=odom != "external", lidar_topic=mesh or odom != "external" or value["scan_source"] == "convert",
            image_topic=value["camera_driver_mode"] != "off")
        for key, shown in visible.items():
            form, widget = self.rows[key]
            widget.setVisible(shown)
            form.labelForField(widget).setVisible(shown)
        self.cards[2].setVisible(not mesh)
        self.summary.setText("%s\n地图输入：%s  ·  里程计：%s  ·  定位：%s" % (
            backend, "MeshNav 三维网格" if mesh else self.fields["scan_source"].currentText(), self.fields["odometry_backend"].currentText(),
            self.fields["localization_backend"].currentText()))

    def save(self):
        value = self.values()
        if value["odometry_backend"] == "fast_livo" and value["fast_livo_vision"] and value["camera_driver_mode"] == "off":
            self.status.setText("请提供相机输入，或关闭 FAST-LIVO 视觉。")
            return
        mesh = value["navigation_engine"] == "meshnav"
        if mesh and value["localization_backend"] == "amcl":
            self.status.setText("MeshNav 需要点云定位或已有地图 TF，请调整定位来源。")
            return
        if mesh and value["meshnav_map"]:
            from pathlib import Path
            path = Path(value["meshnav_map"]).expanduser()
            if path.suffix.lower() != ".ply" or not path.is_file():
                self.status.setText("请选择存在的 PLY 三角网格文件。")
                return
        required = [] if mesh else ["scan_topic"]
        if value["odometry_backend"] == "external": required.append("external_odom_topic")
        else: required += ["fast_livo_lidar_topic" if value["odometry_backend"] == "fast_livo" else "lidar_topic", "imu_topic"]
        if not mesh and value["scan_source"] == "convert": required += ["lidar_topic", "laser_target_frame"]
        if any(not value[key] for key in required):
            self.status.setText("请填写当前组合需要的话题与坐标系。")
            return
        self.stack_page.apply_access_settings(value)
        self.settings.setValue("profiles/" + self.profile, value)
        self.settings.sync()
        if self.settings.status() != QSettings.NoError:
            self.status.setText("设置已应用，但保存失败，请检查配置目录权限。")
            return
        self.applied = dict(value)
        self.status.setText("已保存；下次进入导航时按本配置启动。")

    def update_running(self, state):
        editable = state in ("stopped", "error")
        self.scroll.setEnabled(editable)
        self.save_button.setEnabled(editable)
        if not editable: self.status.setText("请先停止导航，再修改接入设置。")
        else: self.status.setText("配置独立保存在当前机器人预设中。")

    def resizeEvent(self, event):
        columns = 1 if self.width() < 1000 else 2
        for index, card in enumerate(self.cards):
            self.cards_grid.removeWidget(card)
            self.cards_grid.addWidget(card, index // columns, index % columns)
        super().resizeEvent(event)

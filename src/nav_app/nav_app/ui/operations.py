"""Mapping/navigation tools integrated with the existing desktop workspace."""
from dataclasses import replace
import json
import os
from pathlib import Path
import time
import uuid

from PyQt5.QtCore import QObject, QTimer, Qt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QPushButton, QLabel, QLineEdit, QListWidget, QFileDialog, QInputDialog,
    QMessageBox, QCheckBox, QFormLayout, QComboBox, QSpinBox, QPlainTextEdit,
    QGridLayout, QScrollArea, QSplitter)

from ..scene_catalog import discover_scenes, rename_scene, normalize_scene_title
from ..map_processing import load_grid_map
from ..features.map_library import prepare_map_deletion, delete_map
from ..features.navigation_bundle import build
from ..features.capture_quality import atomic_json
from ..features.localization_tools import validate_scan_context_cache
from ..features.log_store import log_store
from ..features.task_coordinator import TaskCoordinator
from ..features.rosbag import RosbagController
from ..ros_launch_process import RosLaunchProcess
from .background_task import BackgroundTask
from .rosbag_page import RosbagPage
from .bag_library_page import BagLibraryPage
from .diagnostics_page import DiagnosticsPage
from .log_management import LogManagementPage
from .rtk_panel import RtkPanel
from .wtrtk_page import RtkMonitorTab
from .map_tasks import MapTasksPage, STATES, STAGES
from .trajectory_page import TrajectoryPage
from ..features.wtrtk import UnavailableRtkAdapter
from nav_rtk.bridge.serial_discovery import list_rtk_serial_ports
from nav_rtk.parser import correction_settings


class MapLibraryPage(QWidget):
    def __init__(self, window, busy, task_submit=None):
        super().__init__()
        self.window, self.busy = window, busy
        self.task_submit = task_submit
        self.root = window.map_storage_dir
        self.worker = BackgroundTask(self)
        self.preview_worker = BackgroundTask(self)
        self._preview_loaded = dict(cloud=None, grid=None)
        self._preview_request = None
        self._pending_preview = None
        self._editing_manifest = None
        self.scenes = []
        layout = QVBoxLayout(self)
        directory_row = QHBoxLayout()
        self.directory = QLineEdit(str(self.root))
        directory_row.addWidget(QLabel('地图目录'))
        directory_row.addWidget(self.directory, 1)
        choose_directory = QPushButton('选择目录')
        choose_directory.clicked.connect(self.choose_directory)
        directory_row.addWidget(choose_directory)
        layout.addLayout(directory_row)
        self.splitter = QSplitter(Qt.Horizontal)
        catalogue = QWidget()
        catalogue_layout = QVBoxLayout(catalogue)
        catalogue_layout.setContentsMargins(0, 0, 0, 0)
        self.maps = QListWidget()
        self.maps.setWordWrap(True)
        self.maps.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.maps.currentRowChanged.connect(self.select_bundle)
        catalogue_layout.addWidget(self.maps, 1)
        actions = QGridLayout()
        for index, (text, callback) in enumerate([
                ('刷新', self.refresh), ('新地图包', self.new_bundle), ('保存地图包', self.register),
                ('从 PCD 生成', self.generate), ('用于导航', self.load), ('核对坐标', self.verify),
                ('改名', self.rename), ('删除', self.delete)]):
            button = QPushButton(text)
            button.setObjectName('dangerButton' if text == '删除' else
                                 'primaryButton' if text in ('保存地图包', '用于导航') else 'secondaryButton')
            button.clicked.connect(callback)
            actions.addWidget(button, index // 3, index % 3)
        catalogue_layout.addLayout(actions)
        self.splitter.addWidget(catalogue)
        assets = QWidget()
        assets_layout = QVBoxLayout(assets)
        form = QFormLayout()
        self.bundle_title = QLineEdit()
        form.addRow('地图包名称', self.bundle_title)
        self.cloud_file, self.sc_file, self.grid_file = QLineEdit(), QLineEdit(), QLineEdit()
        self.asset_fields = dict(cloud=self.cloud_file, scan_context=self.sc_file, grid=self.grid_file)
        for kind, title in [('cloud', 'PCD 点云'), ('scan_context', 'SC 索引'), ('grid', '栅格地图')]:
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.addWidget(self.asset_fields[kind], 1)
            browse = QPushButton('选择')
            browse.clicked.connect(lambda _=False, key=kind: self.choose_asset(key))
            row_layout.addWidget(browse)
            if kind != 'scan_context':
                preview = QPushButton('查看')
                preview.clicked.connect(lambda _=False, key=kind: self.open_asset(key))
                process = QPushButton('处理')
                process.clicked.connect(lambda _=False, key=kind: self.open_asset(key, edit=True))
                row_layout.addWidget(preview)
                row_layout.addWidget(process)
            form.addRow(title, row)
        assets_layout.addLayout(form)
        self.coordinates_verified = QCheckBox('已核对 PCD 与栅格的坐标、方向和比例')
        for field in (self.cloud_file, self.grid_file):
            field.textChanged.connect(lambda _: self.coordinates_verified.setChecked(False))
        assets_layout.addWidget(self.coordinates_verified)
        self.preview_tabs = QTabWidget()
        panel_factory = type(window.editor_map_panel)
        self.cloud_preview = panel_factory(cloud_3d=True)
        self.cloud_preview.set_display_mode('cloud')
        self.cloud_preview.set_cloud_display_options('original', 'XYZ', (False,)*3, 'orthographic', 'height')
        self.grid_preview = panel_factory()
        for panel in (self.cloud_preview, self.grid_preview):
            panel.setMinimumSize(160, 120)
        self.preview_tabs.addTab(self.cloud_preview, '点云预览')
        self.preview_tabs.addTab(self.grid_preview, '栅格预览')
        assets_layout.addWidget(self.preview_tabs, 1)
        scroll = QScrollArea()
        self.asset_scroll = scroll
        scroll.setWidgetResizable(True)
        scroll.setWidget(assets)
        self.splitter.addWidget(scroll)
        self.splitter.setSizes([300, 800])
        self.splitter.setStretchFactor(1, 1)
        layout.addWidget(self.splitter, 1)
        self.status = QLabel('分别选择 PCD、SC 和栅格；查看可预览，处理可进入地图处理页。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.preview_tabs.currentChanged.connect(self.request_preview)
        for kind in ('cloud', 'grid'):
            self.asset_fields[kind].editingFinished.connect(lambda key=kind: self.open_asset(key))
        self.refresh()

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh(preserve_edits=True)

    def refresh(self, *, preserve_edits=False):
        previous = self._editing_manifest
        self.root = Path(self.directory.text()).expanduser()
        self.scenes = discover_scenes(self.root)
        if preserve_edits:
            self.maps.blockSignals(True)
        else:
            self.new_bundle()
        self.maps.clear()
        for index, scene in enumerate(self.scenes):
            self.maps.addItem(f'{scene.title} · {scene.summary}')
            if previous is not None and scene.manifest == previous:
                self.maps.setCurrentRow(index)
        self.maps.blockSignals(False)
        has_draft = bool(self.bundle_title.text().strip() or any(field.text().strip() for field in self.asset_fields.values()))
        if self.maps.currentRow() < 0 and self.scenes and not has_draft:
            self.maps.setCurrentRow(0)
        if self.isVisible():
            self.request_preview(force=True)

    def choose_directory(self):
        directory = QFileDialog.getExistingDirectory(self, '选择地图目录', str(self.root))
        if directory:
            self.directory.setText(directory)
            self.refresh()

    def new_bundle(self):
        self._editing_manifest = None
        self.maps.setCurrentRow(-1)
        self.bundle_title.clear()
        for field in self.asset_fields.values():
            field.clear()
        self.coordinates_verified.setChecked(False)
        self._preview_loaded = dict(cloud=None, grid=None)
        self._pending_preview = None
        self.cloud_preview.set_pointcloud(None)
        self.grid_preview.set_map(None, (0., 0.), 0., 0.)
        self.status.setText('选择地图包后自动预览；也可分别选择 PCD 或栅格文件。')

    def select_bundle(self, index):
        if index < 0:
            return
        scene = self.scenes[index]
        self._editing_manifest = scene.manifest
        self.bundle_title.setText(scene.title)
        self.cloud_file.setText(scene.cloud)
        self.sc_file.setText(scene.scan_context)
        self.grid_file.setText(scene.grid)
        self.coordinates_verified.setChecked(scene.verified)
        self._preview_loaded = dict(cloud=None, grid=None)
        self.cloud_preview.set_pointcloud(None)
        self.grid_preview.set_map(None, (0., 0.), 0., 0.)
        if self.isVisible():
            self.request_preview(force=True)

    def choose_asset(self, kind):
        filters = dict(cloud='PCD 点云 (*.pcd)', scan_context='Scan Context (*.sc.v2)',
                       grid='栅格地图 (*.yaml *.yml *.pgm)')
        field = self.asset_fields[kind]
        path, _ = QFileDialog.getOpenFileName(self, '选择地图文件', field.text() or str(self.root), filters[kind])
        if path:
            field.setText(path)
            if kind in ('cloud', 'grid'):
                self.open_asset(kind)

    def request_preview(self, _index=None, *, force=False):
        if not force and not self.isVisible():
            return
        kind = 'cloud' if self.preview_tabs.currentWidget() is self.cloud_preview else 'grid'
        path = self.asset_fields[kind].text().strip()
        if not path:
            self._pending_preview = None
            self._preview_loaded[kind] = None
            if kind == 'cloud':
                self.cloud_preview.set_pointcloud(None)
            else:
                self.grid_preview.set_map(None, (0., 0.), 0., 0.)
            self.status.setText('请先选择 PCD 点云' if kind == 'cloud' else '请先选择栅格地图')
            return
        request = (kind, path)
        if not force and self._preview_loaded[kind] == path:
            self._pending_preview = None
            self.status.setText(f'已打开：{path}')
            return
        if self.preview_worker.busy and self._preview_request == request:
            self._pending_preview = None
            return
        self._pending_preview = request
        self.start_pending_preview()

    def start_pending_preview(self):
        if self.preview_worker.busy or self._pending_preview is None:
            return
        kind, path = self._pending_preview
        self._pending_preview = None
        self._preview_request = (kind, path)
        self._preview_loaded[kind] = None
        if kind == 'cloud':
            self.cloud_preview.set_pointcloud(None)
        else:
            self.grid_preview.set_map(None, (0., 0.), 0., 0.)
        self.status.setText('正在加载点云预览…' if kind == 'cloud' else '正在加载栅格预览…')
        from ..map_processing import load_grid_map, load_pointcloud
        self.preview_worker.start(
            lambda: load_pointcloud(path, max_points=300000) if kind == 'cloud' else load_grid_map(path),
            lambda value, error: self.preview_finished(kind, path, value, error))

    def preview_finished(self, kind, path, value, error):
        self._preview_request = None
        active = 'cloud' if self.preview_tabs.currentWidget() is self.cloud_preview else 'grid'
        if kind == active and self.asset_fields[kind].text().strip() == path:
            if error:
                self.status.setText(f'预览读取失败：{path}；{error}')
            else:
                self._preview_loaded[kind] = path
                self.asset_loaded(kind, False, path, value, '')
        self.start_pending_preview()

    def open_asset(self, kind, edit=False):
        if not edit:
            self.preview_tabs.blockSignals(True)
            self.preview_tabs.setCurrentWidget(self.cloud_preview if kind == 'cloud' else self.grid_preview)
            self.preview_tabs.blockSignals(False)
            self.request_preview(force=True)
            return
        if not self.available():
            return
        path = self.asset_fields[kind].text().strip()
        if not path:
            self.status.setText('请先选择 PCD 点云' if kind == 'cloud' else '请先选择栅格地图')
            return
        from ..map_processing import load_grid_map, load_pointcloud
        self.status.setText('正在加载完整地图…' if edit else '正在加载预览…')
        self.worker.start(
            lambda: load_pointcloud(path, max_points=None if edit else 300000) if kind == 'cloud' else load_grid_map(path),
            lambda value, error: self.asset_loaded(kind, edit, path, value, error))

    def asset_loaded(self, kind, edit, path, value, error):
        if error:
            self.status.setText('地图读取失败：' + error)
            return
        window = self.window
        if edit:
            if kind == 'cloud':
                window._set_local_cloud(value, reset_original=True)
            else:
                window._set_local_grid(value, reset_original=True)
                window.grid_resolution_spin.setValue(value.resolution)
                window.node.grid_frame = value.frame_id
            window.set_editor_map_mode('cloud' if kind == 'cloud' else 'grid')
            window.map_status.setText(f'已从地图包打开：{path}')
            window.show_map_tools()
        elif kind == 'cloud':
            self.cloud_preview.set_pointcloud(value.points, colors=value.colors, color_source=value.color_source)
            self.preview_tabs.setCurrentWidget(self.cloud_preview)
        else:
            self.grid_preview.set_map(value.occupancy, value.origin[:2], value.resolution, value.origin[2])
            self.preview_tabs.setCurrentWidget(self.grid_preview)
        self.status.setText(f'已打开：{path}')
        if not edit:
            QTimer.singleShot(0, lambda: self.asset_scroll.ensureWidgetVisible(self.preview_tabs, 0, 0))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.splitter.setOrientation(Qt.Vertical if self.width() < 650 else Qt.Horizontal)

    def selected(self):
        index = self.maps.currentRow()
        if index < 0:
            raise ValueError('请先选择地图包')
        return self.scenes[index]

    def available(self):
        reason = self.busy()
        if self.worker.busy or reason:
            self.status.setText(reason or '地图处理尚未完成')
            return False
        return True

    def register(self):
        if not self.available():
            return
        try:
            title = normalize_scene_title(self.bundle_title.text())
            cloud = Path(self.cloud_file.text().strip()).expanduser().resolve(strict=True)
            grid = Path(self.grid_file.text().strip()).expanduser().resolve(strict=True)
            if cloud.suffix.lower() != '.pcd':
                raise ValueError('地图包点云请选择 PCD 文件')
            if grid.suffix.lower() not in ('.yaml', '.yml'):
                raise ValueError('登记地图包请选择栅格 YAML；PGM 可先查看或进入地图处理页')
            sc = self.sc_file.text().strip()
            if sc:
                sc = str(Path(sc).expanduser().resolve(strict=True))
            confirmed = self.coordinates_verified.isChecked()
            root = self.root.resolve()
            manifest = self._editing_manifest or root / f'{uuid.uuid4().hex}.scene.json'
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))
            return
        self.status.setText('正在核对地图文件和 SC 与 PCD 的匹配关系…')
        def save_bundle():
            from ..map_processing import load_grid_map, load_pointcloud
            load_grid_map(grid)
            load_pointcloud(cloud, max_points=1)
            if sc:
                valid, reason = validate_scan_context_cache(str(cloud), sc)
                if not valid:
                    raise ValueError(reason)
            document = json.loads(manifest.read_text(encoding='utf-8')) if manifest.is_file() else dict(version=2, destinations=[])
            base = manifest.parent
            changed = ((base / document.get('cloud', '')).resolve() != cloud
                       or (base / document.get('grid', '')).resolve() != grid)
            if changed:
                document.pop('generated', None)
            document.update(version=2, title=title, cloud=os.path.relpath(cloud, base),
                grid=os.path.relpath(grid, base), frame='map', coordinates_verified=confirmed,
                scan_context=os.path.relpath(sc, base) if sc else '', scan_context_verified=bool(sc))
            base.mkdir(parents=True, exist_ok=True)
            atomic_json(manifest, document)
            return str(manifest)
        self.worker.start(save_bundle, self.registered)

    def registered(self, manifest, error):
        if error:
            self.status.setText('地图包保存失败：' + error)
            return
        self._editing_manifest = Path(manifest)
        self.refresh()
        self.status.setText(f'地图包已保存：{manifest}')

    def generate(self):
        if not self.available():
            return
        cloud, _ = QFileDialog.getOpenFileName(self, '选择建图 PCD', str(self.root), 'PCD (*.pcd)')
        if not cloud:
            return
        title, ok = QInputDialog.getText(self, '生成导航地图包', '名称；原始 XYZ 保持不变')
        if not ok:
            return
        aligned = QMessageBox.question(self, '点云方向', '点云 Z 轴是否已经与重力方向对齐？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes
        if not aligned:
            self.status.setText('请先在地图处理页核对点云方向，再生成通行栅格')
            return
        self.status.setText('导航地图任务已加入后台队列，可在“处理进度”查看')
        payload = dict(
            cloud=cloud,
            directory=str(self.root),
            title=title,
            gravity_aligned=True,
        )
        if self.task_submit is not None:
            self.task_submit('navigation', payload, self.generated)
        else:
            self.worker.start(
                lambda: build(cloud, self.root, title, gravity_aligned=True),
                self.generated)

    def generated(self, result, error):
        self.refresh()
        self.status.setText(error if error else f"已生成 {result['scene']}；{result.get('warning', '')}；上车前请检查通行区域")

    def load(self):
        if not self.available():
            return
        try:
            scene = self.selected()
            self.load_scene(scene)
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def load_scene(self, scene):
        try:
            if not scene.ready:
                raise ValueError(scene.summary)
            if scene.scan_context:
                valid, reason = validate_scan_context_cache(scene.cloud, scene.scan_context)
                if not valid:
                    raise ValueError(reason)
            grid = load_grid_map(scene.grid)
            page = self.window.navigation_stack_page
            page.localization_map.setText(scene.cloud)
            page.navigation_map.setText(scene.grid)
            page.scan_context_map.setText(scene.scan_context)
            if not self.window.load_navigation_localization_map(scene.cloud):
                raise ValueError('定位地图预览加载失败，请查看地图处理页')
            self.window._set_local_grid(grid, reset_original=True)
            self.window.node.grid_frame = grid.frame_id
            self.status.setText(f'已选择 {scene.title}；返回系统首页，点击进入导航即可启动')
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def verify(self):
        if not self.available():
            return
        try:
            scene = self.selected()
            if not scene.manifest:
                raise ValueError('请先登记地图包')
            from ..map_processing import load_grid_map, load_pointcloud
            load_grid_map(scene.grid)
            load_pointcloud(scene.cloud)
            if QMessageBox.question(self, '核对地图坐标', '已核对 PCD 与栅格使用相同 map 原点、方向和比例？',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
                document = json.loads(scene.manifest.read_text())
                document['coordinates_verified'] = True
                atomic_json(scene.manifest, document)
                self.refresh()
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def rename(self):
        if not self.available():
            return
        try:
            scene = self.selected()
            if not scene.manifest:
                raise ValueError('请先登记地图包')
            title, ok = QInputDialog.getText(self, '地图包改名', '名称', text=scene.title)
            if ok:
                rename_scene(scene.manifest, title)
                self.refresh()
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def delete(self):
        if not self.available():
            return
        try:
            plan = prepare_map_deletion(self.selected(), self.root, self.scenes)
            names = '\n'.join(str(p) for p in plan.targets)
            if QMessageBox.question(self, '删除地图包', '以下文件将永久删除；共享和外部文件保留：\n' + names,
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
                return
            delete_map(plan)
            self.refresh()
            self.status.setText('已删除选中地图包；共享和外部资产已保留')
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))


class RtkPage(QWidget):
    def __init__(self, node, busy):
        super().__init__()
        self.busy = busy
        self.adapter = getattr(node, 'rtk_live', UnavailableRtkAdapter())
        self.adapter.debug_busy_reason = busy
        self.process = RosLaunchProcess('RTK 串口接入', self)
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.port = QComboBox()
        self.port.setPlaceholderText('请选择串口')
        self.baud = QComboBox()
        self.baud.addItems(['230400', '115200', '460800', '921600', '9600'])
        row.addWidget(self.port, 1)
        row.addWidget(self.baud)
        self.manual_port = QCheckBox('手动输入')
        self.manual_port.toggled.connect(self.port.setEditable)
        row.addWidget(self.manual_port)
        layout.addLayout(row)
        row = QHBoxLayout()
        for text, callback in [('搜索串口', self.search), ('连接', self.connect_receiver), ('断开', self.disconnect_receiver)]:
            button = QPushButton(text)
            if text in ('删除', '断开', '停用差分', '清除账号'):
                button.setObjectName('dangerButton')
            elif text in ('用于导航', '从 PCD 生成', '连接', '保存账号', '启用差分'):
                button.setObjectName('primaryButton')
            else:
                button.setObjectName('secondaryButton')
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        tabs = QTabWidget()
        self.monitor = RtkMonitorTab(self.adapter)
        self.gnss = RtkPanel(getattr(node, 'rtk_monitor', None))
        tabs.addTab(self.monitor, '接收机')
        tabs.addTab(self.gnss, 'GNSS 质量')
        settings = QWidget()
        form = QFormLayout(settings)
        self.fields = {}
        for name, label in [('host', '服务器'), ('mountpoint', '挂载点'), ('username', '账号'), ('password', '密码')]:
            field = QLineEdit()
            if name == 'password':
                field.setEchoMode(QLineEdit.Password)
            self.fields[name] = field
            form.addRow(label, field)
        self.ntrip_port = QSpinBox()
        self.ntrip_port.setRange(1, 65535)
        self.ntrip_port.setValue(2101)
        form.addRow('端口', self.ntrip_port)
        self.datum = QComboBox()
        self.datum.addItems(['unknown', 'WGS84', 'CGCS2000', 'ITRF2008'])
        form.addRow('坐标基准', self.datum)
        self.tls = QCheckBox('使用 TLS')
        form.addRow(self.tls)
        for text, callback in [('保存账号', self.save_account), ('启用差分', lambda: self.corrections(True)),
                               ('停用差分', lambda: self.corrections(False)), ('清除账号', self.clear_account)]:
            button = QPushButton(text)
            if text in ('删除', '断开', '停用差分', '清除账号'):
                button.setObjectName('dangerButton')
            elif text in ('用于导航', '从 PCD 生成', '连接', '保存账号', '启用差分'):
                button.setObjectName('primaryButton')
            button.clicked.connect(callback)
            form.addRow(button)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(settings)
        tabs.addTab(scroll, 'NTRIP 差分')
        layout.addWidget(tabs, 1)
        self.status = QLabel('连接后发布 /rtk/fix；ENU 仅供观测，不向导航发布 TF。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.process.log_received.connect(lambda text: self.status.setText(text[-700:]))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(250)
        self.search()
        try:
            account = correction_settings.load()
            if account:
                for name, field in self.fields.items():
                    field.setText(account[name])
                self.ntrip_port.setValue(account['port'])
                self.datum.setCurrentText(account['datum'])
                self.tls.setChecked(account['tls'])
        except (OSError, ValueError) as exc:
            self.status.setText(f'差分设置读取失败：{exc}')

    def showEvent(self, event):
        super().showEvent(event)
        self.search()

    def search(self):
        previous = self.port.currentText()
        ports = list_rtk_serial_ports()
        self.port.clear()
        self.port.addItems(ports)
        self.port.setCurrentIndex(self.port.findText(previous))
        if self.manual_port.isChecked():
            self.port.setEditText(previous)
        self.port.setPlaceholderText(f'找到 {len(ports)} 个串口，请点击选择' if ports else '未发现串口')
        self.status.setText(
            f"发现 {len(ports)} 个串口：{'、'.join(ports)}；请选择后连接，搜索不会打开设备"
            if ports else '未发现 ttyTHS/ttyUSB 串口；请检查设备连接，或开启手动输入填写路径')

    def connect_receiver(self):
        from nav_rtk.bridge.serial_discovery import require_explicit_port
        try:
            if self.busy():
                raise ValueError(self.busy())
            port = require_explicit_port(self.port.currentText())
            if not self.process.active:
                self.process.start_launch(['launch', 'nav_rtk', 'rtk.launch.py',
                    f'port:={port}', f'baudrate:={self.baud.currentText()}'])
                self.gnss.start()
        except ValueError as exc:
            self.status.setText(str(exc))

    def disconnect_receiver(self):
        if self.busy():
            self.status.setText(self.busy())
            return
        self.process.stop_launch()
        self.gnss.process.stop_launch()

    def save_account(self):
        try:
            correction_settings.save(dict(provider='custom', port=self.ntrip_port.value(),
                **{name: field.text() for name, field in self.fields.items()},
                tls=self.tls.isChecked(), datum=self.datum.currentText(), auto_start=False))
            self.status.setText('账号已保存；仅在点击启用差分后连接服务')
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def corrections(self, enabled):
        try:
            if self.busy():
                raise ValueError(self.busy())
            if enabled and not correction_settings.load():
                raise ValueError('请先保存有效的差分账号')
            correction_settings.set_requested(enabled)
            self.status.setText('已请求启用差分，请检查接收机状态' if enabled else '已请求停用差分')
        except (OSError, ValueError) as exc:
            self.status.setText(str(exc))

    def clear_account(self):
        self.corrections(False)
        if self.busy():
            return
        correction_settings.clear()
        for field in self.fields.values():
            field.clear()
        self.status.setText('差分账号已清除')

    def refresh(self):
        self.monitor.refresh()
        if self.adapter.available:
            self.adapter.publish_debug_guard()

    def shutdown(self):
        self.timer.stop()
        self.process.shutdown()
        self.gnss.shutdown()
        self.adapter.stop()


class OperationsController(QObject):
    """Shared task ownership; each widget is hosted by its own workspace."""
    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.controller = RosbagController(window.node, self)
        self.controller.admission_guard = self.admission
        self.pending_replay = None
        self.replay_topics = ()
        self.replay_node = ""
        self.controller.play_process.state_changed.connect(self.playback_state)
        self.replay_deadline = 0.
        window.mapping_controller.admission_guard = self.algorithm_admission
        window.navigation_stack_controller.start_guard = self.navigation_admission
        window.sensor_driver_controller.start_guard = lambda: (not self.controller.playing, '回放期间不能启动现场驱动')
        task_root = Path(os.environ.get(
            'NAV_TASK_STATE_DIR',
            Path.home() / '.local/state/nav/map-tasks'))
        self.task_coordinator = TaskCoordinator(task_root)
        self.task_callbacks = {}
        self.library = MapLibraryPage(
            window, self.busy, self.submit_map_task)
        self.map_tasks = MapTasksPage(
            self.task_coordinator.cancel,
            self.retry_map_task,
            self.use_map_task,
        )
        window.localization_tools_controller.task_submit = self.submit_map_task
        window.localization_tools_controller.task_cancel = (
            self.task_coordinator.cancel)
        self.bags = RosbagPage(self.controller, embedded=True)
        self.audit_process = RosLaunchProcess('rosbag 校验', self)
        audit_row = QHBoxLayout()
        audit = QPushButton('校验所选录包')
        audit.setObjectName('primaryButton')
        audit.clicked.connect(self.audit)
        cancel_audit = QPushButton('暂停校验')
        cancel_audit.clicked.connect(self.audit_process.stop_launch)
        audit_row.addWidget(audit)
        audit_row.addWidget(cancel_audit)
        self.bags.layout().addLayout(audit_row)
        self.recordings = BagLibraryPage(self.controller,
            lambda: self.bags.record_directory.text(), self.busy, self.bags)
        self.bags.tabs.addTab(self.recordings, '录包管理')
        self.recordings.load_requested.connect(self.load_recording)
        self.recordings.audit_requested.connect(self.audit_bag_path)
        self.audit_process.state_changed.connect(self.audit_state)
        self.bags.return_requested.connect(window.show_navigation_setup)
        self.bags.mapping_view_requested.connect(window.show_mapping)
        self.bags.mapping_requested.connect(lambda request: self.replay(request, 'mapping'))
        self.bags.navigation_requested.connect(lambda request: self.replay(request, 'navigation'))
        self.logs = LogManagementPage(self.confirm)
        self.rtk = RtkPage(window.node, self.busy)
        self.trajectory = TrajectoryPage(self, window)
        from .phone_page import PhoneRemotePage
        self.phone = PhoneRemotePage(window)
        self.diagnostics_page = DiagnosticsPage(self.snapshot, window.map_storage_dir, window)
        window.mapping_page.navigation_requested.connect(self.prepare_mapping_navigation)
        window.mapping_page.progress_requested.connect(self.show_mapping_progress)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        window.mapping_controller.process.state_changed.connect(
            lambda state: self._release_task_owner('mapping', state))
        window.navigation_stack_controller.overall_state_changed.connect(
            lambda state: self._release_task_owner('navigation', state))
        self.controller.record_process.state_changed.connect(
            lambda state: self._release_task_owner('recording', state))

    def prepare_mapping_navigation(self, cloud, title, grid_method="observed"):
        page = self.window.mapping_page
        method_name = "地形分析" if grid_method == "terrain" else "原有投影"
        page.navigation_job_id = self.submit_map_task('navigation', dict(
            cloud=cloud, directory=str(self.window.map_storage_dir),
            title=title+' · '+method_name, gravity_aligned=True, grid_method=grid_method), self.mapping_navigation_completed)
        page.navigation_status.setText(f"地图已保存；使用{method_name}生成栅格，SC 按原流程生成，进度见‘处理进度’")

    def mapping_navigation_completed(self, result, error):
        self.library.refresh()
        self.window.mapping_page.navigation_status.setText(
            "自动处理失败：" + error + "；PCD 已保留，可在处理进度中重试"
            if error else "导航地图已就绪：栅格和 SC 索引已生成，可在地图包中查看。" + result.get("warning", ""))

    def show_mapping_progress(self):
        self.window.show_map_progress()

    def submit_map_task(self, kind, payload, completed=None):
        identifier = self.task_coordinator.submit(kind, payload)
        if completed is not None:
            self.task_callbacks[identifier] = completed
        return identifier

    def retry_map_task(self, job):
        self.task_coordinator.submit(
            job['kind'], job['payload'], priority=job['priority'], retry=True)

    def use_map_task(self, job):
        scene_path = str((job.get('result') or {}).get('scene') or '')
        self.library.refresh()
        scene = next(
            (value for value in self.library.scenes
             if value.manifest and str(value.manifest) == scene_path),
            None,
        )
        if scene is None:
            self.library.status.setText('任务结果中的地图包不存在，请重新生成')
            return
        self.library.load_scene(scene)
        self.window.show_map_library()

    def _release_task_owner(self, owner, state):
        if state in ('stopped', 'error'):
            snapshot = self.task_coordinator.snapshot()
            if snapshot.get('foreground') == owner:
                self.task_coordinator.foreground(None)

    def _refresh_map_tasks(self):
        snapshot = self.task_coordinator.snapshot()
        self.map_tasks.refresh(snapshot['jobs'], snapshot.get('error', ''), self.library.root)
        for job in snapshot['jobs']:
            page = self.window.mapping_page
            if job['id'] == page.navigation_job_id and job['state'] not in ('succeeded', 'failed', 'cancelled'):
                progress = job.get('progress') or {}
                stage = STAGES.get(progress.get('stage'), '等待处理')
                page.navigation_status.setText(f"自动处理：{stage} · {STATES[job['state']]}；详细进度见处理进度")
            callback = self.task_callbacks.get(job['id'])
            if callback is None or job['state'] not in (
                'succeeded', 'failed', 'cancelled'):
                continue
            del self.task_callbacks[job['id']]
            error = '' if job['state'] == 'succeeded' else (
                job.get('error') or '任务已暂停')
            callback(job.get('result'), error)

    def busy(self):
        w = self.window
        if self.controller.library_delete_in_progress:
            return '录包正在删除'
        if self.controller.recording:
            return '录包或写盘尚未完成'
        if self.audit_process.active:
            return '录包校验尚未结束'
        if self.controller.playing:
            return '回放尚未停止'
        if w.mapping_controller.ros.save_in_progress:
            return '地图保存回执尚未完成'
        if w.mapping_controller.process.active:
            return '建图尚未停止'
        if w.navigation_stack_controller.overall_state in ('starting', 'running', 'stopping'):
            return '导航系统尚未停止'
        if self.library.worker.busy:
            return '地图包处理尚未完成'
        return ''

    def navigation_admission(self):
        if self.window.mapping_controller.process.active:
            return False, '请先停止建图'
        if self.controller.playing and self.pending_replay != 'navigation':
            return False, '请从录包回放页启动回放导航'
        return self.algorithm_admission('navigation')

    def algorithm_admission(self, operation):
        if self.controller.library_delete_in_progress:
            return False, '录包正在删除'
        reason = ('地图包正在处理' if self.library.worker.busy else
                  '录包正在校验' if self.audit_process.active else '')
        if self.rtk.adapter.available and self.rtk.adapter.debug_blocks_tasks:
            reason = 'RTK 正在独占调试，请先恢复接收'
        if reason:
            return False, reason
        owner = ('mapping' if operation == 'map_snapshot'
                 and self.window.mapping_controller.process.active else operation)
        if not self.task_coordinator.reserve_idle(owner):
            return False, '后台地图任务正在占用处理资源，请暂停后重试'
        return True, ''

    def admission(self, operation):
        if operation == 'recording':
            _allowed, reason = self.algorithm_admission(operation)
        else:
            reason = self.busy()
            if any(p.active for p in self.window.sensor_processes.values()):
                reason = '请先停止现场传感器驱动，再回放'
        return not bool(reason), reason

    def confirm(self, text, callback, title):
        if QMessageBox.question(self.window, title, text, QMessageBox.Yes | QMessageBox.No,
                                QMessageBox.No) == QMessageBox.Yes:
            callback()

    def load_recording(self, path):
        if self.bags.load_bag(path):
            self.bags.tabs.setCurrentIndex(1)

    def audit(self):
        if self.bags.metadata is None:
            self.bags.set_status('请先选择录包目录')
            return
        self.audit_bag_path(str(self.bags.metadata.path))

    def audit_bag_path(self, path):
        reason = self.busy()
        if reason:
            self.bags.set_status(reason)
            self.recordings.status.setText(reason)
            return
        bag = Path(path)
        arguments = ['run', 'nav_app', 'audit_bag', str(bag)]
        if (bag / 'nav_manifest.json').is_file():
            arguments.append('--finalize')
        self.audit_process.start_launch(arguments)

    def audit_state(self, state):
        messages = {'starting': '正在启动录包校验…', 'running': '正在逐条校验录包…',
            'stopping': '正在暂停校验，已有数据保留', 'stopped': '录包校验完成',
            'error': '录包校验未通过，请查看录包详情与日志'}
        self.bags.set_status(messages[state])
        self.recordings.status.setText(messages[state])
        if state in ('stopped', 'error'):
            self.recordings.refresh()
        self.recordings.selection_changed()

    def replay(self, request, kind):
        allowed, reason = self.admission('playback')
        if not allowed:
            self.bags.set_status(reason)
            return
        try:
            self.prepare_replay(request, kind)
        except (OSError, ValueError, KeyError) as exc:
            self.bags.set_status(str(exc))
            return
        success, status = self.controller.start_playback(replace(request, start_paused=True))
        self.bags.set_status(status)
        if success:
            self.pending_replay = kind
            self.replay_deadline = time.monotonic() + 60.
            self.window.node.set_clock_mode(True)

    def prepare_replay(self, request, kind):
        """Restore FAST-LIVO calibration before starting historical sensor inputs."""
        w = self.window
        page = w.mapping_page if kind == 'mapping' else w.navigation_stack_page
        backend = w.mapping_controller.backend.key if kind == 'mapping' else page.odometry_backend.currentData()
        if kind == 'mapping':
            topics = [page.pointcloud_combo.currentText(), page.imu_combo.currentText()]
            vision = page.vision_enabled.isChecked()
            image = page.image_topic.text()
        elif backend == 'fast_livo':
            topics = [page.fast_livo_lidar_topic.text(), '/livox/imu']
            vision = page.fast_livo_vision.isChecked()
            image = page.fast_livo_image_topic.text()
        else:
            topics = [page.dlio_pointcloud_topic.text(), page.dlio_imu_topic.text()]
            vision, image = False, ''
        if backend == 'fast_livo' and vision:
            topics.append(image)
        targets = {dict(request.remaps).get(name, name) for name in request.topics}
        missing = set(topics) - targets
        if missing:
            raise ValueError('所选回放话题缺少算法输入：' + ', '.join(sorted(missing)))
        self.replay_topics = tuple(topics)
        self.replay_node = 'laserMapping' if backend == 'fast_livo' else 'dlio_odom_node'
        if backend != 'fast_livo':
            return
        from ament_index_python.packages import get_package_share_directory
        from ..features.bag_calibration import restore_bag_calibration
        params = page.params_file if kind == 'mapping' else page.fast_livo_params
        camera = page.camera_params_file if kind == 'mapping' else page.fast_livo_camera_params
        base = params.text().strip() or str(Path(get_package_share_directory('fast_livo')) / 'config/mid360_live.yaml')
        algorithm, intrinsics = restore_bag_calibration(request.bag_path, base,
            Path.home() / '.local/state/nav/replay-config', require_camera=vision)
        params.setText(algorithm)
        camera.setText(intrinsics)

    def playback_state(self, state):
        if state in ('stopped', 'error'):
            if self.window.mapping_controller.process.active:
                self.window.mapping_controller.process.stop_launch()
            if self.window.navigation_stack_controller._sensor_source == 'bag':
                self.window.navigation_stack_controller.stop()
            self.bags.set_status('回放已结束；使用回放数据的算法正在停止')

    def snapshot(self, resources=False):
        performance = getattr(self.window.node, 'performance', None)
        localization = getattr(self.window.node, 'localization_tools', None)
        from ..features.capture_quality import resource_snapshot
        return dict(time=time.time(), ar_projection=dict(getattr(self.window.node, 'ar_status', {})), resources=resource_snapshot(self.window.map_storage_dir) if resources else {},
            performance=performance.snapshot() if performance else {},
            localization=localization.health.sample if localization else {},
            localization_valid=localization.health.valid(self.window.node.get_clock().now().nanoseconds) if localization else False,
            mapping=self.window.mapping_controller.process.active,
            navigation=self.window.navigation_stack_controller.overall_state,
            recording=self.controller.recording, playback=self.controller.playing,
            rtk=self.rtk.adapter.serial_description(), log_store=log_store().health())

    def export_diagnostics(self):
        self.diagnostics_page.export_json()

    def refresh(self):
        self._refresh_map_tasks()
        if self.window.pages.currentWidget() is self.window.diagnostics_workspace_page:
            self.diagnostics_page.refresh()
        if (self.window.pages.currentWidget() is self.window.recordings_workspace_page
                and self.bags.tabs.currentWidget() is self.recordings):
            self.recordings.selection_changed()
        if not self.pending_replay:
            return
        w = self.window
        kind = self.pending_replay
        if not self.controller.playing:
            self.pending_replay = None
            self.bags.set_status('回放已停止，未启动算法')
            return
        if time.monotonic() > self.replay_deadline:
            self.pending_replay = None
            self.bags.set_status('等待算法输入订阅超时；请检查话题映射和算法日志')
            self.controller.play_process.stop_launch()
            return
        if kind == 'resume':
            ready = all(any(info.node_name == self.replay_node
                            for info in w.node.get_subscriptions_info_by_topic(topic))
                        for topic in self.replay_topics)
            if ready:
                success, status = self.controller.resume_playback()
                self.bags.set_status(status)
                if success:
                    self.pending_replay = None
            return
        if kind == 'mapping':
            page = w.mapping_page
            page.use_sim_time.setChecked(True)
            success, status = page.start_mapping()
            if success:
                w.show_mapping()
        else:
            config = replace(w.navigation_stack_page.configuration(), sensor_source='bag', use_sim_time=True)
            success, status = w.navigation_stack_controller.start(config)
            if success:
                w.show_navigation_stack()
        self.bags.set_status(status)
        self.pending_replay = 'resume' if success else None
        if not success:
            self.controller.play_process.stop_launch()

    def shutdown(self):
        self.timer.stop()
        self.task_coordinator.close()
        self.trajectory.shutdown()
        self.phone.shutdown()
        self.audit_process.shutdown()
        self.controller.shutdown()
        self.rtk.shutdown()

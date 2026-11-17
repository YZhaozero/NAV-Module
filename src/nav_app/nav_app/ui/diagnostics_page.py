"""Readable runtime diagnostics and upstream structured evidence exports."""
import copy
import json
from pathlib import Path
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QLabel, QTreeWidget, QTreeWidgetItem, QPlainTextEdit, QPushButton, QLineEdit,
    QFileDialog)

from ..features import device_diagnostics
from ..features.capture_quality import atomic_json
from ..features.log_store import health_warning
from .background_task import BackgroundTask


class DiagnosticsPage(QWidget):
    def __init__(self, snapshot, directory, parent=None):
        super().__init__(parent)
        self.snapshot = snapshot
        self.directory = Path(directory)
        self.worker = BackgroundTask(self)
        self.resources = {}
        self.sampled_at = 0.
        self.exporting = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        self.summary = QLabel('等待运行诊断')
        self.summary.setObjectName('sectionTitle')
        self.summary.setWordWrap(True)
        self.advice = QLabel('启动建图或导航后查看输入时序、处理积压和定位质量')
        self.advice.setObjectName('hint')
        self.advice.setWordWrap(True)
        layout.addWidget(self.summary)
        layout.addWidget(self.advice)
        tabs = QTabWidget()
        self.metrics = QTreeWidget()
        self.metrics.setHeaderLabels(['项目', '当前状态'])
        self.metrics.setColumnWidth(0, 190)
        tabs.addTab(self.metrics, '运行状态')
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        tabs.addTab(self.details, '原始详情')
        layout.addWidget(tabs, 1)
        self.fields = {}
        for group, names in [
            ('任务', ['导航系统', '建图', '录包', '回放', 'RTK']),
            ('算法与定位', ['诊断心跳', '状态年龄', '处理占用', '雷达积压', '定位匹配', '原始输入']),
            ('设备资源', ['磁盘可用', '可用内存', '内存余量', '温度', '日志写盘'])]:
            parent_item = QTreeWidgetItem([group])
            self.metrics.addTopLevelItem(parent_item)
            parent_item.setExpanded(True)
            for name in names:
                item = QTreeWidgetItem([name, '等待数据'])
                parent_item.addChild(item)
                self.fields[name] = item
        actions = QHBoxLayout()
        self.refresh_button = QPushButton('刷新资源')
        self.refresh_button.clicked.connect(self.sample_resources)
        self.json_button = QPushButton('导出 JSON')
        self.json_button.setObjectName('primaryButton')
        self.json_button.clicked.connect(self.export_json)
        self.bundle_button = QPushButton('导出诊断包')
        self.bundle_button.setObjectName('primaryButton')
        self.bundle_button.clicked.connect(self.export_bundle)
        for button in (self.refresh_button, self.json_button, self.bundle_button):
            actions.addWidget(button)
        layout.addLayout(actions)
        self.status = QLabel('资源信息在后台采集')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.export_path = QLineEdit()
        self.export_path.setReadOnly(True)
        self.export_path.setPlaceholderText('导出后显示完整路径，可复制')
        self.export_path.hide()
        layout.addWidget(self.export_path)

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh()

    def refresh(self):
        snapshot = self.snapshot()
        performance = snapshot['performance']
        decision = performance.get('decision', {})
        self.summary.setText(decision.get('title', '等待算法诊断数据'))
        self.advice.setText(decision.get('advice', '启动建图或导航后查看原始输入和定位质量'))
        states = {'stopped': '已停止', 'starting': '启动中', 'running': '运行中', 'stopping': '停止中', 'error': '异常'}
        values = {'导航系统': states.get(snapshot['navigation'], snapshot['navigation']),
                  'RTK': snapshot['rtk']}
        for name, key in [('建图', 'mapping'), ('录包', 'recording'), ('回放', 'playback')]:
            values[name] = '运行中' if snapshot[key] else '已停止'
        latest = performance.get('latest')
        fresh = latest is not None and time.monotonic() - latest['received_monotonic'] <= 3.
        data = latest['data'] if fresh else {}
        values.update({'诊断心跳': '持续更新' if fresh else '未收到或已过期',
            '状态年龄': f"{data['state_age_s']:.3f} s" if 'state_age_s' in data else '等待数据',
            '处理占用': f"{data['busy_ratio']:.0%}" if 'busy_ratio' in data else '等待数据',
            '雷达积压': f"{data['lidar_backlog_s']:.3f} s" if 'lidar_backlog_s' in data else '等待数据',
            '原始输入': performance.get('error') or ('正在观测' if latest else '等待数据')})
        localization = snapshot['localization']
        values['定位匹配'] = ('有效' if snapshot['localization_valid'] else '无效或已过期') if localization else '等待数据'
        for name, key in [('磁盘可用', 'disk_free_bytes'), ('可用内存', 'memory_available_bytes'), ('内存余量', 'memory_headroom_bytes')]:
            value = self.resources.get(key)
            values[name] = f'{value/1024**3:.2f} GiB' if value is not None else '尚无资源数据'
        temperatures = self.resources.get('temperature_c', {})
        values['温度'] = ' · '.join(f'{name} {value:.1f} °C' for name, value in temperatures.items()) or '未提供温度数据'
        values['日志写盘'] = health_warning(snapshot['log_store']) or '正常'
        for name, value in values.items():
            self.fields[name].setText(1, value)
            self.fields[name].setToolTip(1, value)
        self.details.setPlainText(json.dumps({**snapshot, 'resources': self.resources}, ensure_ascii=False, indent=2))
        if time.monotonic() - self.sampled_at > 5. and not self.worker.busy:
            self.sample_resources()

    def sample_resources(self):
        if self.worker.busy:
            return
        self.worker.start(lambda: device_diagnostics.collect(self.directory), self.resources_loaded)

    def resources_loaded(self, value, error):
        self.sampled_at = time.monotonic()
        if error:
            self.resources = {}
            self.status.setText('资源采集失败：' + error)
        else:
            self.resources = value
            self.status.setText('资源信息已更新')
        self.refresh()

    def export_json(self):
        if self.worker.busy:
            self.status.setText('后台操作尚未完成，请稍后导出')
            return
        path, _ = QFileDialog.getSaveFileName(self, '保存诊断', str(self.directory / 'nav-diagnostics.json'), 'JSON (*.json)')
        if not path:
            return
        snapshot = copy.deepcopy(self.snapshot())
        self.exporting = True
        def work():
            snapshot['resources'] = device_diagnostics.collect(self.directory)
            atomic_json(Path(path), snapshot)
            return path
        self.worker.start(work, self.exported)

    def export_bundle(self):
        if self.worker.busy:
            self.status.setText('后台操作尚未完成，请稍后导出')
            return
        snapshot = copy.deepcopy(self.snapshot())
        self.exporting = True
        self.status.setText('正在生成诊断包…')
        self.worker.start(lambda: device_diagnostics.export_diagnostics(
            Path.home() / '.local/state/nav/diagnostics', snapshot=snapshot), self.exported)

    def exported(self, path, error):
        self.exporting = False
        if error:
            self.status.setText('导出失败：' + error)
        else:
            self.status.setText('诊断已导出，可复制下方路径')
            self.export_path.setText(str(path))
            self.export_path.show()

"""Small-screen view of device-owned processing; the page never owns a job."""
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem, QLabel, QPushButton, QProgressBar
from .touch_scroll import enable_touch_scroll
from ..scene_catalog import discover_scenes


STATES = dict(queued='等待处理', running='正在处理', cancelling='正在结束当前步骤', waiting_foreground='等待采集或导航结束',
              succeeded='已完成', failed='需要重试', cancelled='已暂停')
KINDS = dict(navigation='准备导航地图', audit='校验采集数据',
             scan_context='生成定位索引', recovery='恢复地图')
STAGES = dict(source='核对点云', grid='生成通行图', scan_context='生成定位索引', index='生成定位索引', scene='登记地图',
              audit='校验采集数据', evidence='整理采集记录', compress='压缩采集数据',
              verify_index='核对定位索引', verify='核对导航地图',
              storage='等待可用存储空间', complete='已完成', register='登记地图', floor='识别地面')


class MapTasksPage(QWidget):
    def __init__(self, cancel, retry, use_map, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.rows = QListWidget(); self.rows.setWordWrap(True)
        enable_touch_scroll(self.rows)
        layout.addWidget(self.rows, 1)
        self.detail = QLabel('保存后的地图会在设备空闲时自动处理。'); self.detail.setWordWrap(True)
        self.detail.setTextFormat(Qt.PlainText); layout.addWidget(self.detail)
        self.progress = QProgressBar(); layout.addWidget(self.progress)
        actions = QHBoxLayout(); layout.addLayout(actions)
        self.pause = QPushButton('暂停处理'); self.retry = QPushButton('继续或重试'); self.use = QPushButton('使用这张地图导航')
        for button in (self.pause,self.retry,self.use): button.setMinimumHeight(48); actions.addWidget(button)
        self.pause.clicked.connect(lambda: self.selected() and cancel(self.selected()['id']))
        self.retry.clicked.connect(lambda: self.selected() and retry(self.selected()))
        self.use.clicked.connect(lambda: self.selected() and use_map(self.selected()))
        self.rows.currentItemChanged.connect(self.selection_changed)
        self.jobs = {}; self.signature = None; self.service_error = ''

    def selected(self):
        row = self.rows.currentItem()
        return self.jobs.get(row.data(Qt.UserRole)) if row else None

    def refresh(self, jobs, service_error='', directory=None):
        self.service_error = str(service_error or '')
        scenes = {str(scene.manifest.resolve()): scene
                  for scene in discover_scenes(directory) if scene.manifest} if directory is not None else {}
        visible = []
        for job in jobs:
            scene_path = (job.get('result') or {}).get('scene')
            scene = scenes.get(str(Path(scene_path).resolve())) if scene_path else None
            if directory is not None and job['kind'] == 'navigation' and job['state'] == 'succeeded' and scene is None:
                continue
            name = scene.title if scene else job['payload'].get('title') or KINDS.get(job['kind'], '地图处理')
            text = name+' · '+KINDS.get(job['kind'], '地图处理')+' · '+STATES.get(job['state'], '等待处理')
            source = job['payload'].get('cloud')
            if source:
                text += '\nPCD：'+Path(source).name
            text += ' · 提交于 '+datetime.fromtimestamp(job['created']).strftime('%Y-%m-%d %H:%M:%S')
            if scene and scene.error:
                text += '\n'+scene.error
            visible.append((job, text))
        previous = self.selected(); identifier = previous['id'] if previous else None
        self.jobs = {job['id']: job for job, _ in visible}
        signature = [(job['id'], job['state'], job.get('attempt'), text) for job, text in visible]
        if self.signature != signature:
            self.rows.clear()
            for job, text in visible:
                item = QListWidgetItem(text)
                item.setData(Qt.UserRole, job['id']); self.rows.addItem(item)
                if job['id'] == identifier: self.rows.setCurrentItem(item)
            if self.rows.currentRow() < 0 and visible: self.rows.setCurrentRow(0)
            self.signature = signature
        self.selection_changed()

    def selection_changed(self, *_):
        job = self.selected()
        for button in (self.pause,self.retry,self.use): button.setEnabled(False)
        self.progress.hide()
        if self.service_error:
            self.detail.setText('地图处理记录写入失败，已有数据保留。检查存储后重试：'+self.service_error)
            self.retry.setEnabled(bool(job) and job['state'] != 'succeeded')
            return
        if not job:
            self.detail.setText('保存后的地图会在设备空闲时自动处理。'); return
        state = job['state']; value = job.get('progress') or {}
        stage = STAGES.get(value.get('stage'),'处理地图')
        self.detail.setText(job.get('error') or (STATES.get(state,'等待处理')+' · '+stage))
        done,total = value.get('done'),value.get('total')
        if state == 'running' and isinstance(done,(int,float)) and isinstance(total,(int,float)) and total > 0:
            self.progress.show(); self.progress.setRange(0,1000); self.progress.setValue(min(1000,int(done/total*1000)))
            self.progress.setFormat(f'{int(done)} / {int(total)}')
        self.pause.setEnabled(state in ('queued','running','waiting_foreground'))
        self.retry.setEnabled(state in ('failed','cancelled'))
        scene = (job.get('result') or {}).get('scene')
        self.use.setEnabled(state == 'succeeded' and bool(scene) and Path(scene).is_file())

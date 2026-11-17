"""Qt recording inventory using the upstream scan and confirmed deletion model."""
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import Qt, QSettings, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QComboBox,
    QLineEdit, QPushButton, QTreeWidget, QTreeWidgetItem, QLabel, QFileDialog,
    QMessageBox, QPlainTextEdit)

from ..features.bag_library import scan_bags, disk_summary, prepare_bag_deletion, delete_bag
from .background_task import BackgroundTask
from .touch_scroll import enable_touch_scroll


class BagLibraryPage(QWidget):
    load_requested = pyqtSignal(str)
    audit_requested = pyqtSignal(str)

    def __init__(self, controller, directory, busy, parent=None):
        super().__init__(parent)
        self.controller, self.directory, self.busy = controller, directory, busy
        self.worker = BackgroundTask(self)
        self.settings = QSettings('NAV', 'nav_app')
        self.rows = []
        self.deleting = False
        self.controller.library_delete_in_progress = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        row = QHBoxLayout()
        self.roots = QComboBox()
        self.roots.setMinimumWidth(0)
        self.roots.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.roots.addItem('全部录包目录', '')
        add = QPushButton('添加目录')
        add.clicked.connect(self.add_directory)
        refresh = QPushButton('刷新')
        refresh.clicked.connect(self.refresh)
        self.search = QLineEdit()
        self.search.setPlaceholderText('搜索名称或状态')
        self.search.textChanged.connect(self.populate)
        row.addWidget(self.roots, 2)
        row.addWidget(add)
        row.addWidget(refresh)
        row.addWidget(self.search, 1)
        layout.addLayout(row)
        self.files = QTreeWidget()
        self.files.setHeaderLabels(['录包名称', '状态', '时长', '大小'])
        self.files.setRootIsDecorated(False)
        self.files.setColumnWidth(0, 220)
        self.files.setColumnWidth(1, 180)
        self.files.setColumnWidth(2, 90)
        self.files.currentItemChanged.connect(self.selection_changed)
        enable_touch_scroll(self.files)
        layout.addWidget(self.files, 1)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumHeight(64)
        layout.addWidget(self.details)
        actions = QHBoxLayout()
        self.sort = QComboBox()
        self.sort.addItem('最新优先', 'modified')
        self.sort.addItem('大小优先', 'size_bytes')
        self.sort.addItem('时长优先', 'duration_seconds')
        self.sort.currentIndexChanged.connect(self.populate)
        self.load_button = QPushButton('载入回放')
        self.load_button.setObjectName('workspaceButton')
        self.load_button.clicked.connect(self.load_selected)
        self.audit_button = QPushButton('校验')
        self.audit_button.setObjectName('primaryButton')
        self.audit_button.clicked.connect(self.audit_selected)
        self.delete_button = QPushButton('删除')
        self.delete_button.setObjectName('dangerButton')
        self.delete_button.clicked.connect(self.request_delete)
        for widget in (self.sort, self.load_button, self.audit_button, self.delete_button):
            actions.addWidget(widget)
        layout.addLayout(actions)
        self.status = QLabel('选择目录并刷新，查看录包状态和完整性结果')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.selection_changed()
        self.roots.activated.connect(self.refresh)

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh()

    def scan_roots(self):
        roots = [self.directory(), *self.settings.value('bag_library_roots', [], type=list)]
        return list(dict.fromkeys(str(Path(p).expanduser()) for p in roots if str(p).strip()))

    def add_directory(self):
        path = QFileDialog.getExistingDirectory(self, '选择录包目录', self.directory())
        if path:
            roots = self.scan_roots()
            self.settings.setValue('bag_library_roots', list(dict.fromkeys([*roots, path])))
            self.refresh()

    def refresh(self, *_):
        if self.worker.busy:
            return
        roots = self.scan_roots()
        if not roots:
            self.status.setText("请先选择录包保存目录")
            return
        selected = self.roots.currentData()
        self.roots.blockSignals(True)
        self.roots.clear()
        self.roots.addItem('全部录包目录', '')
        for root in roots:
            self.roots.addItem(root, root)
        self.roots.setCurrentIndex(max(0, self.roots.findData(selected)))
        self.roots.blockSignals(False)
        paths = [selected] if selected in roots else roots
        active = self.controller.record_output if self.controller.recording else None
        self.status.setText('正在扫描录包目录…')
        self.worker.start(lambda: (scan_bags(paths, active), disk_summary(paths[0])), self.loaded)
        self.selection_changed()

    def loaded(self, value, error):
        if error:
            self.status.setText('录包列表读取失败：' + error)
        else:
            (self.rows, notices), space = value
            self.populate()
            self.status.setText(f'{len(self.rows)} 个录包 · {space}' + ('；' + '；'.join(notices) if notices else ''))
        self.selection_changed()

    def populate(self, *_):
        selected = self.files.currentItem()
        path = selected.data(0, Qt.UserRole).path if selected else None
        self.files.clear()
        query = self.search.text().strip().casefold()
        key = self.sort.currentData()
        for entry in sorted(self.rows, key=lambda item: getattr(item, key), reverse=True):
            if query and query not in (entry.path.name + ' ' + entry.status).casefold():
                continue
            item = QTreeWidgetItem([entry.path.name, entry.status,
                f'{entry.duration_seconds:.1f} s', f'{entry.size_bytes/1024**2:.1f} MiB'])
            item.setData(0, Qt.UserRole, entry)
            item.setToolTip(0, str(entry.path))
            self.files.addTopLevelItem(item)
            if entry.path == path:
                self.files.setCurrentItem(item)
        self.selection_changed()

    def selection_changed(self, *_):
        item = self.files.currentItem()
        entry = item.data(0, Qt.UserRole) if item else None
        idle = not self.worker.busy and not self.busy()
        enabled = bool(entry and idle and entry.status != '正在录制或整理')
        self.load_button.setEnabled(enabled and entry.can_load if entry else False)
        self.audit_button.setEnabled(enabled)
        self.delete_button.setEnabled(enabled)
        self.details.setPlainText('' if entry is None else
            f'{entry.path}\n{entry.messages:,} 条消息 · {len(entry.topics)} 个话题 · '
            f'{datetime.fromtimestamp(entry.modified):%Y-%m-%d %H:%M}\n{entry.detail}')

    def load_selected(self):
        item = self.files.currentItem()
        if item and self.load_button.isEnabled():
            self.load_requested.emit(str(item.data(0, Qt.UserRole).path))

    def audit_selected(self):
        item = self.files.currentItem()
        if item and self.audit_button.isEnabled():
            self.audit_requested.emit(str(item.data(0, Qt.UserRole).path))

    def request_delete(self):
        item = self.files.currentItem()
        reason = self.busy()
        if not item or self.worker.busy or reason:
            self.status.setText(reason or '请先选择录包，等待目录操作完成')
            return
        path = item.data(0, Qt.UserRole).path
        roots = self.scan_roots()
        self.deleting = self.controller.library_delete_in_progress = True
        self.worker.start(lambda: prepare_bag_deletion(path, roots), self.confirm_delete)
        self.selection_changed()

    def confirm_delete(self, plan, error):
        if error:
            self.deleted(None, None, error)
            return
        confirmed = QMessageBox.question(self, '删除录包',
            f'永久删除以下录包及全部分片？\n{plan.path}\n大小 {plan.size_bytes/1024**2:.1f} MiB',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes
        if not confirmed:
            self.deleted(None, None, '')
            return
        self.status.setText('正在删除已确认的录包…')
        self.worker.start(lambda: delete_bag(plan), lambda value, issue: self.deleted(plan, value, issue))

    def deleted(self, plan, size, error):
        self.deleting = self.controller.library_delete_in_progress = False
        if error:
            self.status.setText('删除未完成：' + error)
        elif plan is not None:
            self.rows = [row for row in self.rows if row.path != plan.path]
            self.populate()
            self.status.setText(f'录包已删除，释放 {size/1024**2:.1f} MiB')
        else:
            self.status.setText('已取消删除')
        self.selection_changed()

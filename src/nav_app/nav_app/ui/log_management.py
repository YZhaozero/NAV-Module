"""Manage complete application-run folders, not individual log files."""
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QBoxLayout, QLabel, QPushButton, QListWidget, QListWidgetItem, QLineEdit
from ..features.log_store import log_store, health_warning
from .background_task import BackgroundTask
from .touch_scroll import enable_touch_scroll


def display_time(value):
    if not value:
        return "结束时间未知"
    return f"{value[:4]}-{value[4:6]}-{value[6:8]} {value[9:11]}:{value[11:13]}:{value[13:15]}"


class LogManagementPage(QWidget):
    def __init__(self, confirm, parent=None):
        super().__init__(parent)
        self.confirm = confirm
        self.worker = BackgroundTask(self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.location = QLabel()
        self.location.setStyleSheet("font-size: 16px;")
        self.location.setWordWrap(True)
        self.location.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.location)
        row = QHBoxLayout()
        hint = QLabel("每次运行一个日志包；结束后可导出或删除")
        hint.setStyleSheet("font-size: 16px;")
        hint.setWordWrap(True)
        row.addWidget(hint, 1)
        refresh = QPushButton("刷新")
        refresh.clicked.connect(self.refresh)
        row.addWidget(refresh)
        layout.addLayout(row)
        self.space = QLabel()
        self.space.setWordWrap(True)
        layout.addWidget(self.space)
        self.files = QListWidget()
        self.files.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.files.setTextElideMode(Qt.ElideMiddle)
        enable_touch_scroll(self.files)
        self.files.currentItemChanged.connect(self.selection_changed)
        layout.addWidget(self.files, 1)
        self.status = QLabel()
        self.status.setStyleSheet("font-size: 16px;")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.export_path = QLineEdit()
        self.export_path.setReadOnly(True)
        self.export_path.setPlaceholderText("导出后在此显示 ZIP 完整路径，可复制")
        self.export_path.setMinimumWidth(0)
        self.export_path.setStyleSheet("font-size: 16px;")
        self.export_path.hide()
        layout.addWidget(self.export_path)
        row = QHBoxLayout()
        self.export_selected = QPushButton("导出完整包")
        self.export_selected.clicked.connect(self.request_export)
        row.addWidget(self.export_selected)
        self.delete_selected = QPushButton("删除选中日志包")
        self.delete_selected.clicked.connect(lambda: self.request_delete(False))
        self.delete_all = QPushButton("清理已结束日志包")
        self.delete_all.clicked.connect(lambda: self.request_delete(True))
        row.addWidget(self.delete_selected)
        self.actions = QBoxLayout(QBoxLayout.LeftToRight)
        self.actions.addLayout(row, 2)
        self.actions.addWidget(self.delete_all, 1)
        layout.addLayout(self.actions)
        self.export_selected.setObjectName("primaryButton")
        self.delete_selected.setObjectName("dangerButton")
        self.delete_all.setObjectName("dangerButton")
        self.rows = []
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(2000)

    def resizeEvent(self, event):
        self.actions.setDirection(QBoxLayout.TopToBottom if self.width() < 600 else QBoxLayout.LeftToRight)
        super().resizeEvent(event)

    def refresh_health(self):
        health = log_store().health()
        free = health.get("free")
        text = f"磁盘可用 {free / 1024**3:.2f} GiB" if free is not None else "正在检查日志磁盘空间…"
        warning = health_warning(health)
        self.space.setText(text + (" · " + warning if warning else ""))
        self.space.setToolTip(health.get("error", ""))
        self.space.setStyleSheet("font-size: 16px; color: " + ("#ffbe52;" if warning else "#9bb6c0;"))

    def selection_changed(self, *_):
        if not hasattr(self, "delete_selected"):
            return
        item = self.files.currentItem()
        enabled = bool(item and not item.data(Qt.UserRole)["active"] and not self.worker.busy)
        self.delete_selected.setEnabled(enabled)
        self.export_selected.setEnabled(enabled)
        self.delete_all.setEnabled(not self.worker.busy and any(not row["active"] for row in self.rows))

    def refresh(self, *_):
        if self.worker.busy:
            return
        store = log_store()
        self.location.setText("日志包目录：" + str(store.root))
        self.refresh_health()
        previous = self.files.currentItem()
        selected = previous.data(Qt.UserRole)["relative"] if previous else None
        self.status.setText("正在读取日志包…")
        self.worker.start(store.entries, lambda rows, error: self.loaded(rows, error, selected))
        self.selection_changed()

    def loaded(self, rows, error, selected=None):
        self.files.clear()
        if not error:
            self.rows = rows
            for row in self.rows:
                state = {"running": "运行中", "completed": "已结束", "failed": "异常退出",
                         "interrupted": "中断 · 结束时间未知"}.get(row["status"], "状态未知")
                if row.get("dropped"):
                    state += f' · 缺失 {row["dropped"]} 条'
                end = display_time(row["ended"]) if row["ended"] else state
                size = (f'{row["size"] / 1024:.1f} KB' if row["size"] < 1024**2
                        else f'{row["size"] / 1024**2:.2f} MB')
                item = QListWidgetItem(f'{display_time(row["started"])} →\n{end}\n{state} · {size}')
                item.setToolTip(str(log_store().root / row["relative"]) + "\n" + state)
                item.setData(Qt.UserRole, row)
                self.files.addItem(item)
                if row["relative"] == selected:
                    self.files.setCurrentItem(item)
            if self.files.count() and self.files.currentRow() < 0:
                self.files.setCurrentRow(0)
            self.status.setText(f"{len(self.rows)} 个日志包 · 运行中不可导出或删除")
            self.selection_changed()
        else:
            self.rows = []
            self.status.setText("无法读取日志包目录：" + error)
            self.selection_changed()

    def request_export(self):
        item = self.files.currentItem()
        if self.worker.busy or item is None or item.data(Qt.UserRole)["active"]:
            return
        row = dict(item.data(Qt.UserRole))
        self.status.setText("正在后台导出日志包，可返回继续使用设备…")
        def finished(value, error):
            self.status.setText("导出失败：" + error if error else "完整日志包已导出，复制下方路径后可通过 SSH 获取")
            if not error:
                self.export_path.setText(value)
                self.export_path.setToolTip(value)
                self.export_path.show()
            self.selection_changed()
        self.worker.start(lambda: log_store().export(row), finished)
        self.selection_changed()

    def request_delete(self, all_packages):
        if self.worker.busy:
            return
        item = self.files.currentItem()
        rows = [row for row in self.rows if not row["active"]] if all_packages else (
            [item.data(Qt.UserRole)] if item and not item.data(Qt.UserRole)["active"] else [])
        if not rows:
            return
        def remove():
            def work():
                result = log_store().delete(rows)
                return result, log_store().entries()
            def finished(value, error):
                if error:
                    self.status.setText("删除未完成：" + error)
                else:
                    (removed, skipped), remaining = value
                    self.loaded(remaining, "")
                    self.status.setText(f"已删除 {removed} 个日志包（不可恢复）；跳过 {skipped} 个变化或使用中的包")
                self.selection_changed()
            # Returning from the confirmation may already have started a scan.
            if self.worker.busy:
                QTimer.singleShot(100, remove)
                return
            self.status.setText("正在删除日志包…")
            self.worker.start(work, finished)
            self.selection_changed()
        self.confirm(f"删除这 {len(rows)} 个完整日志包及其中的全部日志？删除后不可恢复，不影响地图和录包。", remove, "删除日志包")


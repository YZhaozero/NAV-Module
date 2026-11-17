"""Shared frame for task-specific workspaces, with one return action."""
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from .page_header import PageHeader


class WorkspacePage(QWidget):
    def __init__(self, title, back, *, subtitle=""):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        self.header = PageHeader(title, back, destination="返回系统首页", subtitle=subtitle)
        layout.addLayout(self.header)
        self.body = QVBoxLayout()
        self.body.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(self.body, 1)
        self.set_compact_mode(False)

    def set_compact_mode(self, compact):
        self.layout().setContentsMargins(*(8, 6, 8, 8) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(7 if compact else 12)

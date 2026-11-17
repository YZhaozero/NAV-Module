"""Daily mapping workspace sharing the settings page's mapping workflow."""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox, QLabel

from .page_header import PageHeader


class MappingWorkspace(QWidget):
    return_requested = pyqtSignal()
    settings_requested = pyqtSignal()

    def __init__(self, settings):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 22)
        layout.setSpacing(10)
        self.header = PageHeader("NAV · 建图", self.return_requested.emit,
            destination="返回系统首页", subtitle="完成采集后保存并结束，自动准备栅格和重定位地图")
        self.settings_button = QPushButton("参数配置")
        self.settings_button.setObjectName("secondaryButton")
        self.settings_button.clicked.connect(self.settings_requested.emit)
        self.header.add_action(self.settings_button)
        layout.addLayout(self.header)
        settings.runtime_actions.insertWidget(2, settings.reset_view)
        settings.runtime_actions.addWidget(settings.finish_button)
        layout.addLayout(settings.runtime_actions)
        method_row = QHBoxLayout()
        method_row.addWidget(QLabel("结束后栅格方法"))
        self.grid_method_combo = QComboBox()
        self.grid_method_combo.setModel(settings.grid_method_combo.model())
        self.grid_method_combo.setCurrentIndex(settings.grid_method_combo.currentIndex())
        self.grid_method_combo.currentIndexChanged.connect(settings.grid_method_combo.setCurrentIndex)
        settings.grid_method_combo.currentIndexChanged.connect(self.grid_method_combo.setCurrentIndex)
        method_row.addWidget(self.grid_method_combo, 1)
        layout.addLayout(method_row)
        layout.addWidget(settings.map_card, 1)
        save_row = QHBoxLayout()
        save_row.addWidget(settings.save_status, 1)
        save_row.addWidget(settings.save_button)
        layout.addLayout(save_row)
        self.map_panel = settings.map_panel

    def set_compact_mode(self, compact, *, portrait=False):
        self.layout().setContentsMargins(
            *(8, 6, 8, 8) if compact else (22, 18, 22, 22))
        self.layout().setSpacing(6 if compact else 10)
        self.map_panel.setMinimumSize(160, 120)

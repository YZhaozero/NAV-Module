"""Shared page navigation: title on the left, one return action at top right."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSizePolicy


class PageHeader(QHBoxLayout):
    def __init__(self, title, callback, *, destination, subtitle=''):
        super().__init__()
        self.setContentsMargins(0, 0, 0, 0)
        self.setSpacing(6)
        titles = QVBoxLayout()
        titles.setSpacing(1)
        label = QLabel(title)
        label.setObjectName('title')
        label.setWordWrap(True)
        titles.addWidget(label)
        if subtitle:
            detail = QLabel(subtitle)
            detail.setObjectName('subtitle')
            detail.setWordWrap(True)
            titles.addWidget(detail)
        self.addLayout(titles, 1)
        self.actions = QHBoxLayout()
        self.actions.setSpacing(6)
        self.actions.setAlignment(Qt.AlignTop)
        self.back_button = QPushButton('返回')
        self.back_button.setObjectName('backButton')
        self.back_button.setProperty('pageBack', True)
        self.back_button.setToolTip(destination)
        self.back_button.setAccessibleName(destination)
        self.back_button.setFixedWidth(88)
        self.back_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.back_button.clicked.connect(callback)
        self.actions.addWidget(self.back_button, 0, Qt.AlignTop)
        self.addLayout(self.actions)

    def add_action(self, button):
        self.actions.insertWidget(self.actions.count()-1, button, 0, Qt.AlignTop)

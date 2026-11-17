"""Keep Qt alive while the desktop integration suite creates multiple windows."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pytest
from PyQt5.QtWidgets import QApplication


@pytest.fixture(scope='session', autouse=True)
def application():
    app = QApplication.instance() or QApplication([])
    yield app

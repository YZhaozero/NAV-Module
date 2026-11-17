"""Recorder/player events reach the bounded buffer and disk without a log widget."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from nav_app.features import log_store as logs
from nav_app.features.rosbag import RosbagController
from nav_app.ui import rosbag_page


@pytest.mark.parametrize("kind,filename", [
    ("record", "record-events.log"),
    ("play", "playback-events.log"),
])
def test_process_events_reach_page_buffer_and_persistent_log(tmp_path, monkeypatch, kind, filename):
    app = QApplication.instance() or QApplication([])
    store = logs.LogStore(tmp_path)
    monkeypatch.setattr(logs, "log_store", lambda: store)
    monkeypatch.setattr(rosbag_page, "previous_capture_warning", lambda: "")
    errors = []
    monkeypatch.setattr(sys, "excepthook", lambda *error: errors.append(error))
    controller = RosbagController()
    page = rosbag_page.RosbagPage(controller)
    process = controller.record_process if kind == "record" else controller.play_process
    buffer = page.record_log if kind == "record" else page.play_log
    other = page.play_log if kind == "record" else page.record_log
    try:
        for index in range(120):
            process.log_received.emit(f"{kind} event {index:03d}\n")
        app.processEvents()
        assert not errors, [str(error[1]) for error in errors]
        assert len(buffer.lines) == 100
        assert f"{kind} event 119" in buffer.toPlainText()
        assert other.toPlainText() == ""
        assert store.flush()
        saved = (store.session / "recording" / filename).read_text()
        # Bounded display history must not truncate the persistent recording log.
        assert all(saved.count(f"{kind} event {index:03d}") == 1 for index in range(120))
    finally:
        controller.shutdown()
        page.close()
        page.deleteLater()
        app.processEvents()
        store.finish()

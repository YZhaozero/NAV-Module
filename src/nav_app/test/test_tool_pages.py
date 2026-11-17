"""Exercise migrated Qt workflows, file boundaries and diagnostic freshness."""
import json
from pathlib import Path
import time
import zipfile

import pytest
import yaml
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtWidgets import QMessageBox

from nav_app.features.rosbag import RosbagController
from nav_app.ui.bag_library_page import BagLibraryPage
from nav_app.ui.diagnostics_page import DiagnosticsPage
from nav_app.features import device_diagnostics


def wait_for(app, condition):
    deadline = time.monotonic() + 4.
    while not condition() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(.01)
    assert condition()


@pytest.fixture
def library(tmp_path, application):
    bag = tmp_path / 'records' / 'capture'
    bag.mkdir(parents=True)
    info = dict(storage_identifier='sqlite3', message_count=20,
        duration=dict(nanoseconds=2_000_000_000), starting_time=dict(nanoseconds=1_000_000_000),
        topics_with_message_count=[dict(topic_metadata=dict(name='/imu', type='sensor_msgs/msg/Imu'), message_count=20)])
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))
    (bag / 'part.db3').write_bytes(b'example payload')
    page = BagLibraryPage(RosbagController(), lambda: str(bag.parent), lambda: '')
    page.settings = QSettings(str(tmp_path / 'settings.ini'), QSettings.IniFormat)
    page.refresh()
    wait_for(application, lambda: not page.worker.busy)
    page.files.setCurrentItem(page.files.topLevelItem(0))
    yield page, bag
    page.close()


def test_inventory_loads_selected_bag_and_filters_without_mutating_files(library):
    page, bag = library
    before = {p.name: p.read_bytes() for p in bag.iterdir()}
    loaded = []
    page.load_requested.connect(loaded.append)
    page.load_button.click()
    assert loaded == [str(bag)]
    page.search.setText('missing')
    assert page.files.topLevelItemCount() == 0
    assert not page.load_button.isEnabled()
    page.search.setText('capture')
    assert page.files.topLevelItemCount() == 1
    assert before == {p.name: p.read_bytes() for p in bag.iterdir()}


def test_pending_recording_has_audit_action_without_allowing_load(library, application):
    page, bag = library
    (bag / 'nav_manifest.json').write_text(json.dumps(dict(state='stored', storage_flush_confirmed=True)))
    page.refresh()
    wait_for(application, lambda: not page.worker.busy)
    page.files.setCurrentItem(page.files.topLevelItem(0))
    assert not page.load_button.isEnabled() and page.audit_button.isEnabled()
    audited = []
    page.audit_requested.connect(audited.append)
    page.audit_button.click()
    assert audited == [str(bag)]


@pytest.mark.parametrize('confirmed', [False, True])
def test_delete_confirmation_blocks_new_capture_and_preserves_other_files(library, application, monkeypatch, confirmed):
    page, bag = library
    outside = bag.parent / 'map.pcd'
    outside.write_bytes(b'preserve map')
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Yes if confirmed else QMessageBox.No)
    page.request_delete()
    assert page.controller.library_delete_in_progress
    success, reason = page.controller.start_recording(bag.parent, 'other', ['/imu'])
    assert not success and '删除' in reason
    wait_for(application, lambda: not page.deleting)
    assert bag.exists() is not confirmed
    assert outside.read_bytes() == b'preserve map'
    assert not page.controller.library_delete_in_progress


def test_changed_bag_is_not_deleted_after_confirmation(library, application, monkeypatch):
    page, bag = library
    def change(*_):
        (bag / 'part.db3').write_bytes(b'changed during confirmation')
        return QMessageBox.Yes
    monkeypatch.setattr(QMessageBox, 'question', change)
    page.request_delete()
    wait_for(application, lambda: not page.deleting)
    assert bag.is_dir() and '变化' in page.status.text()
    assert (bag / 'part.db3').read_bytes() == b'changed during confirmation'


def state():
    return dict(navigation='running', mapping=False, recording=False, playback=False,
        rtk='等待串口', localization=dict(valid=True), localization_valid=False,
        log_store=dict(level='ok'), performance=dict(latest=dict(
            received_monotonic=time.monotonic()-10., data=dict(busy_ratio=.1)),
            decision=dict(title='证据不足', advice='检查输入')))


def test_diagnostic_panel_does_not_show_stale_localization_or_telemetry_as_healthy(tmp_path):
    page = DiagnosticsPage(state, tmp_path)
    page.sampled_at = time.monotonic()
    page.refresh()
    assert page.fields['定位匹配'].text(1) == '无效或已过期'
    assert page.fields['诊断心跳'].text(1) == '未收到或已过期'
    assert page.fields['处理占用'].text(1) == '等待数据'
    assert page.fields['导航系统'].text(1) == '运行中'
    page.close()


def test_diagnostic_resource_error_is_visible(tmp_path, application, monkeypatch):
    page = DiagnosticsPage(state, tmp_path)
    def fail(_):
        raise OSError('disk offline')
    monkeypatch.setattr(device_diagnostics, 'collect', fail)
    page.sample_resources()
    wait_for(application, lambda: not page.worker.busy)
    assert 'disk offline' in page.status.text()
    page.close()


def test_diagnostic_bundle_contains_current_application_state(tmp_path, application, monkeypatch):
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    monkeypatch.setattr(device_diagnostics, 'collect', lambda _: dict(disk_free_bytes=1024))
    page = DiagnosticsPage(state, tmp_path)
    page.export_bundle()
    assert page.exporting
    wait_for(application, lambda: not page.exporting)
    archive_path = Path(page.export_path.text())
    assert archive_path.name.startswith('nav-diagnostics-')
    with zipfile.ZipFile(archive_path) as archive:
        document = json.loads(archive.read('status.json'))
        assert document['application']['navigation'] == 'running'
        assert document['application']['localization_valid'] is False
        assert document['disk_free_bytes'] == 1024
    page.close()

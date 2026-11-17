import json
import os
import subprocess
import sys
import threading
import time
import zipfile
import pytest
from types import SimpleNamespace
from pathlib import Path
from nav_app.features.log_store import LogStore, log_store, process_channel
import nav_app.features.log_store as logs


def test_session_segments_keep_entire_run_and_finish_names(tmp_path):
    store = LogStore(tmp_path, max_bytes=120)
    for i in range(20):
        store.write("sensors", "lidar", f"frame {i} " + "x" * 45)
    assert store.flush()
    running = store.session
    assert running.name.endswith("__running")
    rows = store.entries()
    assert len(rows) == 1 and rows[0]["active"]
    assert store.delete(rows) == (0, 1)
    segments = sorted((running / "sensors").glob("*.log"))
    assert len(segments) > 4
    assert all(p.stat().st_size <= 120 for p in segments)
    combined = "".join(p.read_text() for p in segments)
    assert all(f"frame {i} " in combined for i in range(20))
    store.finish()
    assert not running.exists()
    metadata = json.loads((store.session / "session.json").read_text())
    assert store.session.name == metadata["started"] + "__" + metadata["ended"]
    assert metadata["status"] == "completed"
    assert store.entries()[0]["status"] == "completed"
    finished = store.session
    store.finish()
    assert store.session == finished


def test_async_producer_never_waits_for_slow_disk_and_bounds_queue(tmp_path, monkeypatch):
    store = LogStore(tmp_path, queue_bytes=512)
    entered, release = threading.Event(), threading.Event()
    original = store._write
    def slow(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return original(*args, **kwargs)
    monkeypatch.setattr(store, "_write", slow)
    store.write("device", "startup", "first")
    assert entered.wait(2)
    try:
        before = time.monotonic()
        for i in range(100):
            store.write("sensors", "rtk", str(i))
        assert time.monotonic() - before < .2
        assert store.health()["queued_bytes"] <= 512
        assert store.health()["dropped"] > 0
        assert not store.finish(timeout=.01)
    finally:
        release.set()
        assert store.finish()
    metadata = json.loads((store.session / "session.json").read_text())
    assert metadata["dropped_records"] > 0
    assert "日志缺失" in (store.session / "device/logging.log").read_text()


def test_low_space_warning_pause_and_recovery(tmp_path, monkeypatch):
    usage = SimpleNamespace(total=20 * 1024**3, free=1024**3, used=0)
    monkeypatch.setattr(logs.shutil, "disk_usage", lambda _: usage)
    store = LogStore(tmp_path)
    store.write("device", "startup", "start")
    assert store.flush()
    assert store.health()["level"] == "warning"
    assert "偏低" in logs.health_warning(store.health())
    usage.free = 100 * 1024**2
    store._disk_checked = 0
    store.write("sensors", "lidar", "dropped for disk reserve")
    assert store.flush()
    assert store.health()["level"] == "critical"
    assert store.health()["dropped"] == 1
    assert not (store.session / "sensors/lidar.log").exists()
    usage.free = 10 * 1024**3
    store._disk_checked = 0
    store.write("sensors", "lidar", "recovered")
    assert store.finish()
    assert "recovered" in (store.session / "sensors/lidar.log").read_text()
    assert store.health()["level"] == "ok"
    assert "1 条日志未保存" in logs.health_warning(store.health())


def test_export_complete_package_and_keep_sources(tmp_path):
    store = LogStore(tmp_path / "logs", max_bytes=120)
    for category in logs.CATEGORIES:
        for i in range(8):
            store.write(category, "events", f"{category} event {i} " + "x" * 40)
    assert store.flush()
    with pytest.raises(ValueError, match="仍在运行"):
        store.export(store.entries()[0])
    assert store.finish()
    row = store.entries()[0]
    result = Path(store.export(row))
    assert result.parent == tmp_path / "log-exports"
    assert result.suffix == ".zip"
    assert result.stat().st_mode & 0o777 == 0o600
    with zipfile.ZipFile(result) as archive:
        assert archive.testzip() is None
        for path in store.session.rglob("*.log"):
            assert archive.read(store.session.name + "/" + str(path.relative_to(store.session))) == path.read_bytes()
        assert all(".active.lock" not in name for name in archive.namelist())
        assert json.loads(archive.read(store.session.name + "/session.json"))["status"] == "completed"
    assert store.session.exists()
    assert not list(result.parent.glob("*.partial"))


def test_export_blocks_low_space_changed_files_and_symlinks(tmp_path, monkeypatch):
    store = LogStore(tmp_path / "logs")
    store.write("device", "startup", "first")
    assert store.finish()
    row = store.entries()[0]
    original_usage = logs.shutil.disk_usage
    monkeypatch.setattr(logs.shutil, "disk_usage", lambda _: SimpleNamespace(free=1, total=10**9))
    with pytest.raises(ValueError, match="空间不足"):
        store.export(row)
    monkeypatch.setattr(logs.shutil, "disk_usage", original_usage)
    (store.session / "device/startup.log").write_text("changed")
    with pytest.raises(ValueError, match="已经变化"):
        store.export(row)
    row = store.entries()[0]
    (store.session / "device/secret.log").symlink_to(tmp_path / "outside")
    with pytest.raises(ValueError):
        store.export(row)
    assert not list((tmp_path / "log-exports").glob("*.zip"))


def test_export_lock_blocks_delete_and_cleans_partial_on_failure(tmp_path, monkeypatch):
    store = LogStore(tmp_path / "logs")
    store.write("device", "startup", "start")
    assert store.finish()
    row = store.entries()[0]
    def fail(*args):
        assert store.delete([row]) == (0, 1)
        raise OSError("disk write failed")
    monkeypatch.setattr(logs.shutil, "copyfileobj", fail)
    with pytest.raises(OSError, match="disk write failed"):
        store.export(row)
    assert not list((tmp_path / "log-exports").iterdir())
    assert store.delete(store.entries()) == (1, 0)


def test_legacy_diagnostics_export_flushes_async_events(tmp_path, monkeypatch):
    from nav_app.features import device_diagnostics
    monkeypatch.setenv("NAV_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setattr(device_diagnostics, "collect", lambda _: dict(test=True))
    directory = tmp_path / "diagnostics"
    device_diagnostics.record_process_event("test", "stopped", exit_code=0, directory=directory)
    result = device_diagnostics.export_diagnostics(directory)
    with zipfile.ZipFile(result) as archive:
        event = json.loads(archive.read("process-events.jsonl"))
        assert event["process"] == "test" and event["exit_code"] == 0
        assert not any(name.startswith("logs/") for name in archive.namelist())
    log_store().finish()


def test_whole_package_deletion_and_update_guard(tmp_path):
    old = LogStore(tmp_path)
    old.write("device", "startup", "first run")
    old.write("localization", "localizer", "waiting")
    old.finish()
    running = LogStore(tmp_path)
    running.write("device", "startup", "second run")
    assert running.flush()
    rows = running.entries()
    assert len(rows) == 2
    ended = next(row for row in rows if not row["active"])
    (old.session / "device/startup.log").write_text("changed")
    assert running.delete([ended]) == (0, 1)
    assert running.delete(running.entries()) == (1, 1)
    assert running.session.exists()
    assert not old.session.exists()
    running.finish()


def test_symlinks_unknown_files_and_outside_paths_are_preserved(tmp_path):
    store = LogStore(tmp_path / "logs")
    outside = tmp_path / "outside.log"
    outside.write_text("preserve me")
    store.write("sensors", "../outside", "bad")
    assert store.error
    store.write("sensors", "lidar", "valid")
    assert store.flush()
    (store.session / "sensors/rtk.log").symlink_to(outside)
    store.write("sensors", "rtk", "bad")
    assert store.flush()
    assert store.error
    store.finish()
    assert store.entries() == []
    assert store.delete([dict(relative="../outside.log", identity="")]) == (0, 1)
    assert outside.read_text() == "preserve me"
    (store.session / "sensors/rtk.log").unlink()
    rows = store.entries()
    assert len(rows) == 1
    (store.session / "unknown").mkdir()
    assert store.delete(rows) == (0, 1)
    assert store.session.exists()


def test_crash_is_interrupted_and_does_not_invent_end_time(tmp_path):
    script = "from nav_app.features.log_store import LogStore; import os; s=LogStore(os.environ['TEST_LOG_ROOT']); s.write('device','startup','run'); s.flush(); os._exit(0)"
    subprocess.run([sys.executable, "-c", script], env=dict(os.environ, TEST_LOG_ROOT=str(tmp_path)), check=True)
    store = LogStore(tmp_path)
    rows = store.entries()
    assert len(rows) == 1
    assert rows[0]["status"] == "interrupted"
    assert rows[0]["ended"] is None
    assert not rows[0]["active"]
    assert store.delete(rows) == (1, 0)


def test_second_process_cannot_delete_a_running_package(tmp_path):
    store = LogStore(tmp_path)
    store.write("device", "startup", "run")
    assert store.flush()
    script = "from nav_app.features.log_store import LogStore; import os; s=LogStore(os.environ['TEST_LOG_ROOT']); r=s.entries(); assert r[0]['active']; assert s.delete(r)==(0,1)"
    subprocess.run([sys.executable, "-c", script], env=dict(os.environ, TEST_LOG_ROOT=str(tmp_path)), check=True)
    assert store.session.exists()
    store.finish()


def test_process_output_and_all_paths_use_english_names(tmp_path, monkeypatch):
    from PyQt5.QtCore import QCoreApplication
    from nav_app.ros_launch_process import RosLaunchProcess
    monkeypatch.setenv("NAV_LOG_DIR", str(tmp_path))
    app = QCoreApplication.instance() or QCoreApplication([])
    process = RosLaunchProcess("雷达")
    process.log_received.emit("driver started password:=private token=abc")
    store = log_store()
    assert store.flush()
    text = (store.session / "sensors/lidar.log").read_text()
    assert "driver started" in text and "private" not in text and "abc" not in text
    for name in ("里程计", "定位后端", "导航后端", "点云转 LaserScan", "GNSS 监测", "未知算法"):
        category, filename = process_channel(name)
        assert category.isascii() and filename.isascii()
        store.write(category, filename, "test")
    assert store.flush()
    assert all(str(p.relative_to(store.session)).isascii() for p in store.session.rglob("*"))
    store.finish()


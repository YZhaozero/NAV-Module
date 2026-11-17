"""Bounded, local diagnostics. No pairing codes, environment dumps or raw images."""
from pathlib import Path
import copy
import json
import os
import platform
import subprocess
import time
import zipfile
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from ..ui.background_task import BackgroundTask
from .capture_quality import atomic_json, resource_snapshot


def service_status():
    fields = ("ActiveState", "SubState", "Result", "ExecMainStatus", "NRestarts",
              "MemoryCurrent", "MemoryPeak", "MemorySwapCurrent", "ControlGroup",
              "WatchdogUSec", "WatchdogTimestampMonotonic")
    try:
        result = subprocess.run(["systemctl", "--user", "show", "nav-app.service",
            *[f"--property={key}" for key in fields]], text=True, capture_output=True, timeout=2)
        rows = dict(line.split("=",1) for line in result.stdout.splitlines() if "=" in line)
        group = rows.pop("ControlGroup", "")
        if group.startswith("/") and ".." not in Path(group).parts:
            events = Path("/sys/fs/cgroup") / group.lstrip("/") / "memory.events"
            if events.is_file():
                rows["memory_events"] = dict(line.split() for line in events.read_text().splitlines())
        return rows if result.returncode == 0 else {"available": False}
    except (OSError, subprocess.SubprocessError, ValueError):
        return {"available": False}


def collect(directory):
    result = resource_snapshot(directory)
    result.update(schema_version=1, service=service_status(), kernel=platform.release(),
                  app_pid=os.getpid(), install_prefix=os.environ.get("NAV_INSTALL_PREFIX", ""))
    for name, path in (("uptime", "/proc/uptime"), ("boot_id", "/proc/sys/kernel/random/boot_id")):
        try:
            result[name] = Path(path).read_text().strip()
        except OSError:
            result[name] = None
    return result


def save_sample(directory, snapshot):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    atomic_json(directory / "status.json", snapshot)
    history = directory / "history.jsonl"
    if history.exists() and history.stat().st_size > 2*1024**2:
        history.replace(directory / "history.previous.jsonl")
    with history.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(snapshot, ensure_ascii=False, allow_nan=False) + "\n")


def record_process_event(name, state, *, exit_code=None, expected=False, directory=None):
    """Small lifecycle events only; commands, environment and raw logs stay excluded."""
    directory = Path(directory or Path.home()/".local/state/nav/diagnostics")
    data = json.dumps(dict(unix_time=time.time(),process=str(name)[:80],state=state,
        exit_code=exit_code,expected=bool(expected)),ensure_ascii=False) + "\n"
    def save():
        directory.mkdir(parents=True,exist_ok=True)
        target = directory/"process-events.jsonl"
        if target.exists() and target.stat().st_size > 256*1024:
            target.replace(directory/"process-events.previous.jsonl")
        with target.open("a",encoding="utf-8") as stream:
            stream.write(data)
    from .log_store import log_store
    log_store().submit(save, len(data) * 4 + 128)


def export_diagnostics(directory, *, snapshot=None):
    from .log_store import log_store
    if not log_store().flush():
        raise ValueError("诊断日志仍在保存，请稍后重试导出")
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    sample = collect(directory)
    if snapshot is not None:
        sample["application"] = snapshot
    save_sample(directory, sample)
    target = directory / ("nav-diagnostics-" + time.strftime("%Y%m%d-%H%M%S") + f"-{time.time_ns()%1000000:06d}.zip")
    with zipfile.ZipFile(target, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        # Allowlist only our structured telemetry; never traverse home/config/logs.
        for name in (
            "status.json", "history.jsonl", "history.previous.jsonl",
            "process-events.jsonl", "process-events.previous.jsonl",
            "relocalization-attempts.jsonl",
            "relocalization-attempts.previous.jsonl",
            "relocalization-latest.json",
        ):
            path = directory / name
            if path.is_file() and not path.is_symlink():
                archive.write(path, name)
    return target


class DeviceDiagnostics(QObject):
    updated = pyqtSignal(object)
    exported = pyqtSignal(str, str)

    def __init__(self, parent=None, directory=None):
        super().__init__(parent)
        self.directory = Path(directory or Path.home()/".local/state/nav/diagnostics")
        self.directory.mkdir(parents=True, exist_ok=True)
        self.worker = BackgroundTask(self)
        self.latest = {}
        self.export_pending = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.sample)
        self.timer.start(5000)
        QTimer.singleShot(0, self.sample)

    def sample(self):
        if self.worker.busy:
            return
        if self.export_pending:
            self.export_pending = False
            self.worker.start(lambda: export_diagnostics(self.directory),
                lambda value,error: self.exported.emit(str(value or ""), error))
            return
        parent = self.parent()
        pose_health = copy.deepcopy(getattr(
            getattr(parent, "node", None), "walking_pose_diagnostics", {}))
        def work():
            result = collect(self.directory)
            result["walking_pose"] = pose_health
            save_sample(self.directory, result)
            return result
        def finished(value, error):
            self.latest = value or {"error": error}
            from .log_store import log_store
            log_store().write("device", "health", json.dumps(self.latest, ensure_ascii=False))
            self.updated.emit(self.latest)
            if self.export_pending:
                self.sample()
        self.worker.start(work, finished)

    def export(self):
        self.export_pending = True
        self.sample()

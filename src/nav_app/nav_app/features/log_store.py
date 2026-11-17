"""One managed log package per application run, with English category directories."""
from collections import deque
from datetime import datetime
from pathlib import Path
import atexit
import fcntl
import hashlib
import html
import json
import os
import re
import stat
import shutil
import tempfile
import threading
import time
import zipfile

_LOCK = threading.RLock()
_STORE_LOCK = threading.RLock()
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_NAME = re.compile(r"^[a-zA-Z0-9_-]+$")
_STAMP = r"\d{8}-\d{6}-\d{6}"
_PACKAGE = re.compile(rf"^{_STAMP}__(?:running|{_STAMP})$")
CATEGORIES = {"device": "设备运行", "sensors": "传感器", "mapping": "建图",
              "navigation": "导航", "localization": "定位", "recording": "录包回放"}


def health_warning(health):
    messages = []
    if health.get("level") == "critical":
        messages.append("磁盘不足256 MiB，日志暂停写入，请清理空间")
    elif health.get("level") == "warning":
        messages.append("磁盘空间偏低，请备份并清理旧日志")
    elif health.get("level") == "unknown":
        messages.append("无法检查日志磁盘空间")
    if health.get("dropped"):
        messages.append(f'本次有 {health["dropped"]} 条日志未保存')
    if health.get("error"):
        messages.append("日志写入异常，请进入日志管理查看")
    return "；".join(messages)


def _stamp():
    return datetime.now().strftime("%Y%m%d-%H%M%S-%f")


def _identity(path):
    info = path.lstat()
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_mode)


class LogStore:
    def __init__(self, root=None, max_bytes=4 * 1024**2, queue_bytes=8 * 1024**2):
        self.root = Path(root or os.environ.get("NAV_LOG_DIR") or
                         Path.home() / ".local/state/nav/logs").absolute()
        self.max_bytes = max(1, int(max_bytes))
        self.error = ""
        self.session = None
        self._lock_file = None
        self._closed = False
        self._failed = False
        self._segments = {}
        self._condition = threading.Condition()
        self._queue = deque()
        self._queue_bytes = 0
        self._queue_limit = queue_bytes
        self._accepted = self._completed = 0
        self._dropped = 0
        self._reported_drops = 0
        self._closing = False
        self._thread = None
        self._disk = {}
        self._disk_checked = 0.

    def write(self, category, name, text):
        """Enqueue only. No filesystem operations or disk lock on the caller thread."""
        if category not in CATEGORIES or not _NAME.fullmatch(name):
            self.error = "无效日志分类或文件名"
            return
        text = str(text)
        # Bound both memory and producer-side work, including unusually large output.
        size = len(text) * 4 + 128
        self._enqueue((category, name, text, time.time(), size))

    def submit(self, work, size=4096):
        """Internal structured-diagnostics write, subject to the same bounded queue."""
        self._enqueue((None, None, work, time.time(), size))

    def _enqueue(self, item):
        size = item[4]
        with self._condition:
            if self._closing:
                return
            if size > 512 * 1024 or size + self._queue_bytes > self._queue_limit:
                self._dropped += 1
                return
            self._queue.append(item)
            self._queue_bytes += size
            self._accepted += 1
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="nav-log-writer", daemon=True)
                self._thread.start()
            self._condition.notify_all()

    def flush(self, timeout=5.):
        """Bounded barrier for tests/export/shutdown; never used by sensor callbacks."""
        with self._condition:
            target = self._accepted
            return self._condition.wait_for(lambda: self._completed >= target, timeout)

    def finish(self, timeout=5.):
        with self._condition:
            self._closing = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive():
                self.error = "日志收尾未完成；强制退出可能丢失尚未写入的日志"
                return False
        return True

    def _check_disk(self):
        if time.monotonic() - self._disk_checked < 5 and self._disk:
            return
        self._disk_checked = time.monotonic()
        try:
            target = self.root
            while not target.exists():
                target = target.parent
            usage = shutil.disk_usage(target)
            ratio = usage.free / max(usage.total, 1)
            level = "critical" if usage.free < 256 * 1024**2 else (
                "warning" if usage.free < 1024**3 or ratio < .1 else "ok")
            self._disk = dict(level=level, free=usage.free, total=usage.total)
        except OSError as exc:
            self._disk = dict(level="unknown", error=str(exc))

    def health(self):
        """Cached only: safe to query from the GUI timer without statvfs or scanning."""
        with self._condition:
            return dict(self._disk, dropped=self._dropped, queued_bytes=self._queue_bytes,
                        error=self.error)

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._queue or self._closing, 1.)
                item = self._queue.popleft() if self._queue else None
                if item:
                    self._queue_bytes -= item[4]
                done = self._closing and item is None
            if done:
                break
            self._check_disk()
            if item is None:
                continue
            category, name, text, received, _ = item
            try:
                if self._disk.get("level") == "critical":
                    with self._condition:
                        self._dropped += 1
                else:
                    self._report_loss()
                    if category is None:
                        text()
                    elif not self._write(category, name, text, received):
                        with self._condition:
                            self._dropped += 1
            except (OSError, RuntimeError, ValueError, TypeError, MemoryError) as exc:
                self.error = str(exc)
                with self._condition:
                    self._dropped += 1
            finally:
                with self._condition:
                    self._completed += 1
                    self._condition.notify_all()
        if self._disk.get("level") != "critical":
            self._report_loss()
        self._finish()

    def _report_loss(self):
        with self._condition:
            count = self._dropped
        if count > self._reported_drops:
            if self._write("device", "logging", f"日志缺失：累计 {count} 条记录未保存（队列满、超大记录、空间不足或写入失败）"):
                self._reported_drops = count

    def _check_root(self):
        if self.root.is_symlink():
            raise ValueError("日志目录不能是符号链接")

    def _save_metadata(self):
        target = self.session / "session.json"
        temporary = self.session / "session.json.tmp"
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self._metadata, stream, ensure_ascii=False, indent=2)
            temporary.replace(target)
        finally:
            if temporary.exists() and not temporary.is_symlink():
                temporary.unlink()

    def start(self):
        with _LOCK:
            if self.session is not None or self._closed:
                return
            self._check_root()
            self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
            while True:
                started = _stamp()
                session = self.root / (started + "__running")
                try:
                    session.mkdir(mode=0o700)
                    break
                except FileExistsError:
                    continue
            lock_file = None
            try:
                fd = os.open(session / ".active.lock", os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o600)
                lock_file = os.fdopen(fd, "r+")
                fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.session, self._lock_file = session, lock_file
                self._metadata = dict(schema="nav-log-session-v1", started=started,
                                      ended=None, status="running", pid=os.getpid(),
                                      timezone=time.strftime("%z"))
                for category in CATEGORIES:
                    (session / category).mkdir(mode=0o700)
                self._save_metadata()
            except (OSError, RuntimeError, ValueError):
                if lock_file is not None:
                    lock_file.close()
                self.session, self._lock_file = None, None
                raise

    def _finish(self):
        with _LOCK:
            if self.session is None or self._closed:
                return
            try:
                self._check_root()
                if self.session.is_symlink():
                    raise ValueError("日志包不能是符号链接")
                ended = _stamp()
                self._metadata.update(ended=ended, status="failed" if self._failed else "completed",
                                      dropped_records=self._dropped, logging_error=self.error)
                self._save_metadata()
                target = self.root / (self._metadata["started"] + "__" + ended)
                if target.exists():
                    raise ValueError("结束时间对应日志包已存在")
                self.session.rename(target)
                self.session = target
            except (OSError, ValueError) as exc:
                self.error = str(exc)
            finally:
                self._closed = True
                if self._lock_file is not None:
                    self._lock_file.close()
                    self._lock_file = None

    def _directory(self, category):
        if category not in CATEGORIES:
            raise ValueError("未知日志分类")
        self._check_root()
        self.start()
        if self.session.is_symlink() or (self.session / category).is_symlink():
            raise ValueError("日志目录不能是符号链接")
        return self.session / category

    def _write(self, category, name, text, received=None):
        if self._closed:
            return
        try:
            if not _NAME.fullmatch(name):
                raise ValueError("日志文件名必须使用英文、数字、连字符或下划线")
            clean = _ANSI.sub("", str(text)).replace("\r", "").strip()
            clean = re.sub(r"""(?i)((?:password|passwd|token|secret|authorization)["']?\s*[:=]+\s*)[^\s,}]+""",
                           r"\1[已隐藏]", clean)
            clean = re.sub(r"(https?://)[^\s/@]+:[^\s/@]+@", r"\1[已隐藏]@", clean)
            if not clean:
                return True
            data = (time.strftime("%Y-%m-%d %H:%M:%S ", time.localtime(received)) + clean + "\n").encode("utf-8")
            with _LOCK:
                directory = self._directory(category)
                key = (category, name)
                index = self._segments.get(key, 0)
                for start in range(0, len(data), self.max_bytes):
                    chunk = data[start:start + self.max_bytes]
                    while True:
                        target = directory / (name + (f".{index:06d}" if index else "") + ".log")
                        if target.is_symlink():
                            raise ValueError("日志文件不能是符号链接")
                        if not target.exists() or target.stat().st_size + len(chunk) <= self.max_bytes:
                            break
                        index += 1
                    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
                    with os.fdopen(fd, "ab") as stream:
                        stream.write(chunk)
                self._segments[key] = index
                self.error = ""
                return True
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            return False

    def _scan(self, package):
        """Allow only our fixed-depth package layout; never traverse symlinks."""
        snapshot, files, folders = [], [], []
        def add(path, directory=False):
            identity = _identity(path)
            if not (stat.S_ISDIR(identity[4]) if directory else stat.S_ISREG(identity[4])):
                raise ValueError("日志包包含非普通文件")
            snapshot.append((str(path.relative_to(package)), identity))
            (folders if directory else files).append(path)
        add(package, True)
        for path in package.iterdir():
            if path.name in CATEGORIES:
                add(path, True)
                for child in path.iterdir():
                    if path.name == "sensors" and child.name == "evidence":
                        add(child, True)
                        for process in child.iterdir():
                            if not re.fullmatch(r"(?:livox|rtk|capture)-\d+-\d+", process.name):
                                raise ValueError("日志包包含未知采集来源")
                            add(process, True)
                            for trace in process.iterdir():
                                if not re.fullmatch(r"(?:packets-\d+\.bin|events-\d+\.jsonl|status\.json(?:\.tmp)?)", trace.name):
                                    raise ValueError("日志包包含未知采集文件")
                                add(trace)
                        continue
                    if not re.fullmatch(r"[a-zA-Z0-9_-]+(?:\.\d{6,})?\.log", child.name):
                        raise ValueError("日志包包含未知文件")
                    add(child)
            elif path.name in ("session.json", ".active.lock", "session.json.tmp"):
                add(path)
            else:
                raise ValueError("日志包包含未知目录或文件")
        fingerprint = hashlib.sha256(repr(sorted(snapshot)).encode()).hexdigest()
        return fingerprint, files, folders

    @staticmethod
    def _open_lock(package):
        fd = os.open(package / ".active.lock", os.O_RDWR | os.O_NOFOLLOW)
        stream = os.fdopen(fd, "r+")
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close()
            return None
        return stream

    def entries(self):
        rows = []
        with _LOCK:
            self._check_root()
            if not self.root.exists():
                return rows
            for package in self.root.iterdir():
                if not _PACKAGE.fullmatch(package.name) or package.is_symlink() or not package.is_dir():
                    continue
                try:
                    fingerprint, files, _ = self._scan(package)
                    metadata = json.loads((package / "session.json").read_text())
                    if metadata.get("schema") != "nav-log-session-v1":
                        continue
                    lock = self._open_lock(package)
                    active = lock is None
                    if lock is not None:
                        lock.close()
                    status = "running" if active else metadata.get("status")
                    if status == "running":
                        status = "running" if active else "interrupted"
                    rows.append(dict(relative=package.name, identity=fingerprint,
                                     started=metadata["started"], ended=metadata.get("ended"),
                                     status=status, active=active, size=sum(p.stat().st_size for p in files),
                                     dropped=metadata.get("dropped_records", 0),
                                     files=len(files), categories=[key for key in CATEGORIES if any(p.parent.name == key for p in files)]))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
        return sorted(rows, key=lambda row: row["started"], reverse=True)

    def delete(self, entries):
        removed, skipped = 0, 0
        with _LOCK:
            current = {row["relative"]: row for row in self.entries()}
            for entry in entries:
                row = current.get(entry["relative"])
                if row is None or row["active"] or row["identity"] != entry["identity"]:
                    skipped += 1
                    continue
                package = self.root / row["relative"]
                lock = self._open_lock(package)
                if lock is None:
                    skipped += 1
                    continue
                try:
                    fingerprint, files, folders = self._scan(package)
                    if fingerprint != entry["identity"]:
                        skipped += 1
                        continue
                    for path in files:
                        path.unlink()
                    for path in sorted(folders, key=lambda p: len(p.parts), reverse=True):
                        path.rmdir()
                    removed += 1
                finally:
                    lock.close()
        return removed, skipped

    def export(self, entry):
        """Export a stable, ended package; hold its lock against concurrent deletion."""
        rows = {row["relative"]: row for row in self.entries()}
        row = rows.get(entry["relative"])
        if not row or row["active"] or row["identity"] != entry["identity"]:
            raise ValueError("日志包仍在运行或已经变化，请刷新后选择已结束的包")
        package = self.root / row["relative"]
        lock = self._open_lock(package)
        if lock is None:
            raise ValueError("日志包正在使用，请稍后再试")
        temporary = None
        try:
            fingerprint, files, _ = self._scan(package)
            if fingerprint != row["identity"]:
                raise ValueError("日志包已变化，请刷新后重试")
            destination = self.root.parent / "log-exports"
            if destination.is_symlink():
                raise ValueError("导出目录不能是符号链接")
            destination.mkdir(mode=0o700, parents=True, exist_ok=True)
            usage = shutil.disk_usage(destination)
            if usage.free - row["size"] < max(256 * 1024**2, usage.total * .05):
                raise ValueError("磁盘空间不足，无法导出；请先备份并清理旧日志包或导出文件")
            fd, name = tempfile.mkstemp(prefix=package.name + "-", suffix=".partial", dir=destination)
            temporary = Path(name)
            with os.fdopen(fd, "wb") as stream, zipfile.ZipFile(
                    stream, "w", zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as archive:
                for category in CATEGORIES:
                    archive.writestr(package.name + "/" + category + "/", b"")
                for path in files:
                    if path.name in (".active.lock", "session.json.tmp"):
                        continue
                    info = path.lstat()
                    read_fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                    with os.fdopen(read_fd, "rb") as source:
                        current = os.fstat(source.fileno())
                        if not stat.S_ISREG(current.st_mode) or (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
                            raise ValueError("日志包文件已变化")
                        with archive.open(package.name + "/" + str(path.relative_to(package)), "w", force_zip64=True) as out:
                            shutil.copyfileobj(source, out, 256 * 1024)
            if self._scan(package)[0] != fingerprint:
                raise ValueError("日志包在导出期间发生变化，请重试")
            target = temporary.with_suffix(".zip")
            temporary.replace(target)
            return str(target)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
            lock.close()


_STORE = None


def log_store():
    global _STORE
    root = Path(os.environ.get("NAV_LOG_DIR") or Path.home() / ".local/state/nav/logs").absolute()
    with _STORE_LOCK:
        if _STORE is None or _STORE.root != root:
            if _STORE is not None:
                _STORE.finish()
            _STORE = LogStore(root)
            atexit.register(_STORE.finish)
        return _STORE


def install_exception_logging():
    import sys
    import traceback
    previous = sys.excepthook
    def report(kind, value, tb):
        log_store()._failed = True
        log_store().write("device", "errors", "".join(traceback.format_exception(kind, value, tb)))
        previous(kind, value, tb)
    sys.excepthook = report


def process_channel(name):
    channels = {
        "雷达": ("sensors", "lidar"), "相机": ("sensors", "camera"), "RTK": ("sensors", "rtk"),
        "里程计": ("localization", "odometry"), "定位后端": ("localization", "localizer"),
        "GNSS 监测": ("localization", "gnss-monitor"),
        "点云转 LaserScan": ("navigation", "laserscan"), "导航后端": ("navigation", "backend"),
    }
    if name in channels:
        return channels[name]
    if "rosbag" in name:
        return "recording", "record" if "录制" in name else "playback"
    if "建图" in name or "地图生成" in name:
        return "mapping", "scan-context" if "Scan Context" in name else "mapping"
    return "device", "process-" + hashlib.sha256(name.encode()).hexdigest()[:10]


class LogBuffer:
    """Small compatibility tail for controllers; deliberately has no UI widget."""
    def __init__(self, category="device", name="events", persist=True):
        self.category, self.name, self.persist = category, name, persist
        self.lines = deque(maxlen=100)

    def appendPlainText(self, text):
        self.lines.append(str(text)[-8192:])
        if self.persist:
            log_store().write(self.category, self.name, text)

    def appendHtml(self, text):
        self.appendPlainText(html.unescape(re.sub(r"<[^>]+>", "", text)))

    def setPlainText(self, text):
        self.clear()
        self.appendPlainText(text)

    def toPlainText(self):
        return "\n".join(self.lines)

    def clear(self):
        self.lines.clear()

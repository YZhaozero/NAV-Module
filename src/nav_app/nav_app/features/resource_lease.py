"""Kernel leases: live ROS processes may coexist; local modeling runs alone."""
import os
from pathlib import Path
import threading

_mutex = threading.RLock()
_owned = {}


class DeviceBusy(RuntimeError):
    pass


class ResourceLease:
    def __init__(self, mode, root=None):
        if mode not in ('live', 'model'): raise ValueError('Invalid resource mode')
        self.mode = mode
        self.path = (Path(root or os.environ.get('NAV_RUNTIME_DIR') or
                          Path.home()/'.local/state/nav')/'compute.lock').resolve()
        self.fd = None

    def acquire(self):
        with _mutex:
            if self.fd is not None:return self
            existing = _owned.get(self.path)
            if existing:
                if self.mode != 'live' or existing['mode'] != 'live':raise DeviceBusy('设备正在执行另一项任务')
                existing['count'] += 1;self.fd = existing['stream'].fileno();return self
            self.path.parent.mkdir(parents=True, exist_ok=True)
            stream = self.path.open('a+b')
            try:
                if os.name == 'nt':
                    import msvcrt
                    stream.seek(0);stream.write(b'0');stream.flush();stream.seek(0)
                    msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(stream.fileno(), (fcntl.LOCK_SH if self.mode=='live' else fcntl.LOCK_EX)|fcntl.LOCK_NB)
            except (BlockingIOError, PermissionError):
                stream.close();raise DeviceBusy('设备正在执行另一项任务') from None
            except BaseException:
                stream.close();raise
            _owned[self.path] = dict(mode=self.mode, stream=stream, count=1)
            self.fd = stream.fileno();return self

    def close(self):
        with _mutex:
            if self.fd is None:return
            existing = _owned[self.path];existing['count'] -= 1
            if not existing['count']:
                existing['stream'].close();del _owned[self.path]
            self.fd = None

    __enter__ = acquire

    def __exit__(self, *_): self.close()

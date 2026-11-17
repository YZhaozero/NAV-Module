"""Release completed bag shards from the service's charged page cache.

rosbag's byte cache does not bound the kernel file cache. Only closed shards are
synced/evicted while recording; the newest shard remains owned by the recorder.
No data is deleted, and dirty pages are never advised away before fdatasync.
"""
import os
from pathlib import Path
import re
import threading


def release_file_cache(path):
    if not hasattr(os, "posix_fadvise"):
        return False
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(path, flags)
    try:
        os.fdatasync(fd)
        os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
    finally:
        os.close(fd)
    return True


class BagCacheWriter:
    """One bounded worker; no ROS callback performs file sync or cache reclaim."""
    def __init__(self, directory, interval=.5):
        self.directory = Path(directory)
        self.interval = interval
        self.error = ""
        self.released_bytes = 0
        self.released_shards = 0
        self._last_index = -1
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="bag-closed-shard-cache")

    def start(self):
        self._thread.start()
        return self

    def sweep(self, final=False):
        # Index order matters: lexicographic order puts shard 10 before shard 2.
        shards = []
        for path in self.directory.iterdir() if self.directory.exists() else ():
            match = re.search(r"_(\d+)\.(?:db3|mcap)$", path.name)
            if match and not path.is_symlink():
                shards.append((int(match[1]), path))
        shards.sort()
        for index, path in (shards if final else shards[:-1]):
            if index <= self._last_index:
                continue
            size = path.stat().st_size
            if release_file_cache(path):
                self.released_bytes += size
                self.released_shards += 1
            self._last_index = index

    def _run(self):
        while not self._stop.wait(self.interval):
            try:
                self.sweep()
            except OSError as exc:
                self.error = str(exc)
                return

    def close(self):
        self._stop.set()
        self._thread.join()
        # Called only after the rosbag child has exited.
        try:
            self.sweep(final=True)
        except OSError as exc:
            self.error = str(exc)
        return dict(released_bytes=self.released_bytes,
                    released_shards=self.released_shards, error=self.error)

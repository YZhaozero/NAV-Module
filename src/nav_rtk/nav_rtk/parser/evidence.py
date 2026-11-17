"""Private, append-only transport evidence with bounded producer memory.

Only call bytes() for serial traffic or HTTP *body* bytes; authentication
headers/configurations must never enter the trace. No duration/file-count cap.
"""
import base64
from collections import deque
import json
import os
from pathlib import Path
import shutil
import threading
import time


class EvidenceWriter:
    def __init__(self, name="rtk", root=None, queue_bytes=512*1024, segment_bytes=4*1024**2):
        root = root or os.environ.get("NAV_EVIDENCE_DIR") or Path.home()/".local/state/nav/evidence"
        self.path = Path(root)/f"{name}-{os.getpid()}-{time.time_ns()}"
        self.limit, self.segment_bytes = queue_bytes, segment_bytes
        self._queue, self._bytes = deque(), 0
        self._condition = threading.Condition()
        self._closing = False
        self.accepted = self.written = self.dropped = self.failed = 0
        self.error = ""
        self._thread = threading.Thread(target=self._run, name="rtk-evidence", daemon=True)
        self._thread.start()

    def __call__(self, event, **details):
        # Bound producer work before JSON encoding; no arbitrary configs accepted.
        if any(len(str(value)) > 131072 for value in details.values()):
            with self._condition:
                self.dropped += 1
            return
        row = dict(wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(), event=event, **details)
        try:
            encoded = (json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n").encode()
        except (ValueError,TypeError):
            with self._condition:
                self.dropped += 1
                self.error = "invalid_evidence_record"
            return
        with self._condition:
            size = len(encoded)+128
            if self._closing or size > self.limit or self._bytes+size > self.limit:
                self.dropped += 1
                return
            self._queue.append((encoded,size)); self._bytes += size; self.accepted += 1
            self._condition.notify()

    def bytes(self, event, data):
        # Each serial/network read is bounded; splitting also bounds manual calls.
        view = memoryview(data)
        for start in range(0,len(view),4096):
            part = view[start:start+4096]
            self(event, bytes=len(part), base64=base64.b64encode(part).decode("ascii"))

    def snapshot(self):
        with self._condition:
            return dict(path=str(self.path), accepted=self.accepted, written=self.written,
                        dropped=self.dropped, failed=self.failed, queued_bytes=self._bytes,
                        queue_limit_bytes=self.limit, error=self.error,
                        closed=self._closing and not self._queue)

    def close(self):
        with self._condition:
            self._closing = True; self._condition.notify()
        self._thread.join(timeout=5)

    def _status(self):
        temp = self.path/"status.json.tmp"
        with os.fdopen(os.open(temp,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),"w") as stream:
            json.dump(self.snapshot(),stream)
        os.replace(temp,self.path/"status.json")

    def _run(self):
        stream = None; size = segment = 0; last_check = last_status = 0.; disk_ok = True
        try:
            self.path.mkdir(parents=True, mode=0o700)
            while True:
                with self._condition:
                    if not self._queue and not self._closing:
                        self._condition.wait(timeout=.2)
                    item = self._queue.popleft() if self._queue else None
                    if item: self._bytes -= item[1]
                    done = self._closing and not self._queue
                now = time.monotonic()
                if now-last_check >= 1:
                    disk_ok = shutil.disk_usage(self.path).free > 256*1024**2
                    if not disk_ok: self.error = "disk_below_256MiB"
                    last_check = now
                if item:
                    try:
                        if not disk_ok: raise OSError("disk_below_256MiB")
                        if stream is None or size >= self.segment_bytes:
                            if stream: stream.flush(); os.fsync(stream.fileno()); stream.close()
                            path = self.path/f"events-{segment:06d}.jsonl"
                            stream = os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),"wb")
                            segment += 1; size = 0
                        stream.write(item[0]); size += len(item[0]); self.written += 1
                    except OSError as exc:
                        self.failed += 1; self.error = type(exc).__name__+":"+str(exc)
                if done or now-last_status >= 1:
                    if stream: stream.flush(); os.fsync(stream.fileno())
                    self._status(); last_status = now
                if done: break
        except OSError as exc:
            self.error = type(exc).__name__+":"+str(exc)
            with self._condition:
                self.failed += len(self._queue); self._queue.clear(); self._bytes=0
                self._closing=True
        finally:
            if stream: stream.close()

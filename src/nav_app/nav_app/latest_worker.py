"""One running conversion and one replaceable pending sample; no growing queue."""
from threading import Condition, Thread


class LatestWorker:
    def __init__(self, convert):
        self._convert = convert
        self._condition = Condition()
        self._pending = self._result = None
        self._closed = False
        self._generation = 0
        self._running = False
        self.dropped = 0
        self._thread = Thread(target=self._run, name="nav-frame-converter", daemon=True)
        self._thread.start()

    def submit(self, sample):
        with self._condition:
            if self._closed:
                return
            if self._pending is not None:
                self.dropped += 1
            self._pending = sample
            self._condition.notify()

    def take(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def close(self):
        with self._condition:
            self._closed = True
            self._generation += 1
            self._pending = self._result = None
            self._condition.notify()
        self._thread.join(timeout=2.0)

    def clear(self):
        """Release a completed session and reject its in-flight conversion."""
        with self._condition:
            self._generation += 1
            self._pending = self._result = None

    @property
    def busy(self):
        with self._condition:return self._running or self._pending is not None

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                sample, self._pending = self._pending, None
                generation = self._generation
                self._running = True
            try:
                result = (sample, self._convert(sample), None)
            except Exception as exc:
                result = (sample, None, str(exc))
            with self._condition:
                self._running = False
                if not self._closed and generation == self._generation:
                    self._result = result
            # Do not retain the previous large cloud/image while waiting idle.
            sample = result = None

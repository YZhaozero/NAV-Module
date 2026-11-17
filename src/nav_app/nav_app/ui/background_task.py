"""Bounded background file work; deliver results only on the Qt owner thread."""
from threading import Thread
from PyQt5.QtCore import QObject, pyqtSignal


class BackgroundTask(QObject):
    finished = pyqtSignal(int, object, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.busy = False
        self.generation = 0
        self._callback = None
        self.finished.connect(self._finish)

    def start(self, work, callback):
        if self.busy:
            return False
        self.busy = True
        self.generation += 1
        token = self.generation
        self._callback = callback
        def run():
            try:
                value, error = work(), ""
            except (OSError, ValueError, TypeError, KeyError, RuntimeError, MemoryError) as exc:
                value = None
                error = ('可用内存不足，请结束其他占用后重试。' if isinstance(exc, MemoryError)
                         else str(exc) or '操作未完成，请重试并查看日志。')
            try:
                self.finished.emit(token, value, error)
            except RuntimeError as exc:
                import sys
                print(f"后台操作完成后界面已销毁：{exc}", file=sys.stderr)
        self._thread = Thread(target=run, name="nav-map-task", daemon=True)
        self._thread.start()
        return True

    def _finish(self, token, value, error):
        if token != self.generation:
            return
        self.busy = False
        callback, self._callback = self._callback, None
        if callback:
            callback(value, error)

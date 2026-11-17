"""Recorder with an independent stop sequence and parent-death cleanup."""
import ctypes
import os
import signal
import sys


def main():
    parent = os.getppid()
    if sys.platform == 'linux':
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), 'Cannot protect recorder parent lifetime')
        if parent == 1 or os.getppid() != parent:
            return 2
    os.execvp('ros2', ['ros2', *sys.argv[1:]])


if __name__ == '__main__':
    raise SystemExit(main())

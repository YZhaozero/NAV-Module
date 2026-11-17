from threading import Event
import time
from nav_app.latest_worker import LatestWorker


def wait_result(worker):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        result = worker.take()
        if result is not None:
            return result
        time.sleep(0.005)
    raise AssertionError("worker did not finish")


def test_burst_keeps_only_latest_pending_frame():
    entered, release = Event(), Event()
    seen = []
    def convert(value):
        seen.append(value)
        if value == 0:
            entered.set()
            assert release.wait(2)
        return value * 10
    worker = LatestWorker(convert)
    try:
        worker.submit(0)
        assert entered.wait(2)
        for frame in range(1, 101):
            worker.submit(frame)
        release.set()
        deadline = time.monotonic() + 2
        while 100 not in seen and time.monotonic() < deadline:
            time.sleep(0.005)
        result = wait_result(worker)
        assert result == (100, 1000, None)
        assert seen == [0, 100]
        assert worker.dropped == 99
    finally:
        release.set()
        worker.close()


def test_conversion_failure_does_not_kill_future_frames():
    worker = LatestWorker(lambda x: 10 / x)
    try:
        worker.submit(0)
        assert wait_result(worker)[2]
        worker.submit(2)
        assert wait_result(worker) == (2, 5, None)
    finally:
        worker.close()


def test_new_capture_rejects_previous_inflight_cloud_and_releases_idle_result():
    import numpy as np
    import weakref
    entered,release=Event(),Event()
    def convert(value):
        if value==1:entered.set();assert release.wait(2)
        return np.zeros((1000,3))
    worker=LatestWorker(convert)
    try:
        worker.submit(1);assert entered.wait(2)
        worker.clear();worker.submit(2);release.set()
        result=wait_result(worker)
        assert result[0]==2
        ref=weakref.ref(result[1]);del result
        deadline=time.monotonic()+1
        while ref() is not None and time.monotonic()<deadline:time.sleep(.005)
        assert ref() is None
    finally:release.set();worker.close()

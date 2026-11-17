import signal
import threading

from nav_app.record_guard import RecorderStop


class Child:
    def __init__(self): self.signals=[]
    def poll(self): return None
    def send_signal(self, signum): self.signals.append(signum)


def test_group_stop_is_not_forwarded_a_second_time():
    event=threading.Event(); stop=RecorderStop(event); child=Child()
    # The launcher signals the whole group, including this recorder child.
    child.send_signal(signal.SIGINT)
    stop.group_signal(signal.SIGINT,None)
    stop.request(child); stop.request(child)
    assert event.is_set() and child.signals==[signal.SIGINT]


def test_internal_failure_still_stops_recorder_once():
    event=threading.Event(); stop=RecorderStop(event); child=Child()
    event.set()
    stop.request(child); stop.request(child)
    assert child.signals==[signal.SIGINT]


def test_isolated_recorder_gets_one_signal_after_encoder_drain():
    event=threading.Event(); stop=RecorderStop(event, child_in_group=False); child=Child()
    stop.group_signal(signal.SIGINT,None)
    assert child.signals == []
    stop.request(child); stop.request(child)
    assert child.signals == [signal.SIGINT]

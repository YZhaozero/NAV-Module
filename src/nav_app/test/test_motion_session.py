from concurrent.futures import Future
from types import SimpleNamespace
import json
from nav_app.features.motion_session import MotionSession


def test_one_arm_per_explicit_route_fault_never_rearms():
    now=[0.];calls=[]
    def submit(_):
        future=Future();calls.append(future);return future
    s=MotionSession(SimpleNamespace(service_is_ready=lambda:True,call_async=submit),lambda:True,lambda:now[0])
    update=lambda armed:s.update(json.dumps(dict(schema=1,armed=armed,base_connected=True,stop_confirmed=not armed)))
    update(False);assert s.available();assert not s.poll(1,True)
    now[0]=.21;update(False);s.poll(1,True);assert len(calls)==1
    calls[0].set_result(SimpleNamespace(success=True));update(True);assert not s.poll(1,True) and s.armed
    now[0]=.55;assert s.poll(1,True);update(False);assert s.poll(1,True);assert len(calls)==1
    s.poll(1,False);s.poll(2,True);now[0]+=.21;update(False);s.poll(2,True);assert len(calls)==2


def test_stop_is_never_inferred_from_cancel_and_arm_timeout_fails():
    now=[0.];pending=Future()
    s=MotionSession(SimpleNamespace(service_is_ready=lambda:True,call_async=lambda _:pending),lambda:True,lambda:now[0])
    assert not s.available() and '等待' in s.stop_text()
    row=dict(schema=1,armed=False,base_connected=True,stop_confirmed=True)
    s.update(json.dumps(row));assert '已静止' in s.stop_text();s.poll(1,True)
    now[0]=.21;s.update(json.dumps(row));s.poll(1,True)
    now[0]=3.1;assert '超时' in s.poll(1,True);assert '等待' in s.stop_text()
    for text in ('[]','null','false','{}','x'*2100):
        s.update(text);assert not s.available()


def test_feedback_age_includes_transport_delay_and_recovers_only_on_valid_frame():
    now = [10.]
    session = MotionSession(SimpleNamespace(service_is_ready=lambda: True), lambda: None, lambda: now[0])
    row = dict(schema=1, armed=True, base_connected=True, stamp_ns=9_750_000_000)
    session.update(json.dumps(row), 10_000_000_000)
    assert session.available()
    now[0] += .06
    assert not session.fresh() and "310 毫秒" in session.unavailable_reason()
    session.update(json.dumps(row), 10_060_000_000)
    assert "时间戳" in session.unavailable_reason()
    row.update(stamp_ns=10_060_000_000, base_connected=False, base_reason="底盘服务离线")
    session.update(json.dumps(row), row["stamp_ns"])
    assert session.unavailable_reason() == "底盘服务离线"
    row.update(base_connected=True, armed=False, reason="permit_timeout")
    session.update(json.dumps(row), row["stamp_ns"])
    session.generation, session.armed = 1, True
    assert session.available() and "行驶许可超时" in session.poll(1, True)

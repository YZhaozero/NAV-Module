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

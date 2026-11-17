"""Manual policy tests use an in-memory service, never a robot namespace."""
from types import SimpleNamespace as NS
import math
import pytest
from nav_app.features.manual_drive import ManualDrive, ZERO, joystick_input, velocity


class Future:
    def __init__(self, done=True, ok=True):
        self.finished=done
        self.response=NS(ok=ok,msg="test rejection")
    def done(self):return self.finished
    def result(self):return self.response


class Transport:
    def __init__(self):
        self.commands=[];self.cancelled=[];self.immediate=True;self.accept=True
        self.state=dict(ready=True,estop_released=True,mode=1,measured=(0.,0.),reason="ready")
        self.joy=None;self.closed=False
    def health(self,now):return self.state.copy()
    def poll(self,now):pass
    def send(self,values):
        self.commands.append(tuple(values))
        return Future(self.immediate,self.accept)
    def cancel(self,future):self.cancelled.append(future)
    def close(self):self.closed=True


@pytest.fixture
def drive():
    clock=[10.]
    model=ManualDrive(lambda:"",clock=lambda:clock[0]);model.transport=Transport();model.chassis="ackermann"
    return model,model.transport,clock


@pytest.mark.parametrize("forward,turn,expected",[(0,1,0),(1,1,.4),(-1,1,-.4),(.5,.5,.1)])
def test_ackermann_never_strafes_or_spins_without_travel(forward,turn,expected):
    assert velocity("ackermann",forward,1,turn,.2,.4)==pytest.approx((forward*.2,0,expected))
    assert velocity("ackermann",1,0,1,0,.4)==ZERO


def test_omni_caps_total_translation_and_preserves_left_positive():
    vx,vy,w=velocity("omni",1,1,-1,.2,.4)
    assert math.hypot(vx,vy)==pytest.approx(.2)
    assert vx>0 and vy>0 and w==-.4
    assert velocity("omni",0,-1,0,.2,.4)==(0,-.2,0)
    assert velocity("differential",0,1,1,.2,.4)==(0,0,.4)


def test_polling_and_connecting_never_send_velocity(drive):
    model,transport,clock=drive
    for _ in range(10):model.tick();clock[0]+=.1
    assert transport.commands==[] and not model.armed


@pytest.mark.parametrize("field,value",[("ready",False),("estop_released",False)])
def test_missing_permission_or_estop_blocks_arming(drive,field,value):
    model,transport,_=drive;transport.state[field]=value
    assert not model.arm();model.tick()
    assert transport.commands==[]


def test_missing_chassis_busy_navigation_and_non_neutral_inputs_block_arm(drive):
    model,transport,_=drive
    model.chassis=None;assert not model.arm()
    model.chassis="ackermann";model.busy=lambda:"navigation busy";assert not model.arm()
    model.busy=lambda:"";model.screen_input(.5,1,True);assert not model.arm()
    assert transport.commands==[]


def test_screen_motion_and_release_send_immediate_zero_while_old_request_pending(drive):
    model,transport,_=drive;assert model.arm();transport.immediate=False
    model.screen_input(-.5,1,True);model.tick()
    assert transport.commands[-1]==pytest.approx((.15,0,.15))
    assert model.pending is not None
    model.screen_input(0,0,False)
    assert transport.commands[-1]==ZERO and model.stop_pending is not None
    assert len(transport.commands)==2


def test_stop_never_auto_rearms_and_only_zero_follows_timeout(drive):
    model,transport,clock=drive;assert model.arm();transport.immediate=False
    model.screen_input(0,1,True);model.tick();clock[0]+=.6;model.tick()
    assert not model.armed
    assert transport.commands[-1]==ZERO
    assert len(transport.cancelled)==1
    assert model.stop_pending is not None  # New stop gets its own deadline.
    clock[0]+=.6;model.tick()
    assert not model.armed and len(transport.commands)==2


def test_estop_and_navigation_occupation_cancel_motion(drive):
    model,transport,clock=drive;assert model.arm()
    model.screen_input(0,1,True);model.tick();transport.state["estop_released"]=False
    clock[0]+=.1;model.tick()
    assert not model.armed and transport.commands[-1]==ZERO
    transport.state["estop_released"]=True
    clock[0]+=.1;model.tick();assert not model.armed
    assert model.arm();model.screen_input(0,1,True);model.tick()
    model.busy=lambda:"navigation started";clock[0]+=.1;model.tick()
    assert not model.armed and transport.commands[-1]==ZERO


def test_service_rejection_is_visible_and_disarms(drive):
    model,transport,clock=drive;assert model.arm()
    transport.accept=False;model.screen_input(0,1,True);model.tick();clock[0]+=.1;model.tick()
    assert not model.armed and "拒绝" in model.message
    assert transport.commands[-1]==ZERO


def test_gamepad_requires_neutral_released_deadman_and_fresh_frames(drive):
    model,transport,clock=drive;model.source="gamepad"
    assert not model.arm()
    model.joy=((0,0,0,0),(0,0,0,0,1),clock[0]);assert not model.arm()
    model.joy=((0,0,0,0),(0,0,0,0,0),clock[0]);assert model.arm()
    model.joy=((.5,1,0,0),(0,0,0,0,1),clock[0]);model.tick()
    assert transport.commands[-1]==pytest.approx((.15,0,.15))
    clock[0]+=.1;model.joy=((.5,1,0,0),(0,0,0,0,0),clock[0]);model.tick()
    assert transport.commands[-1]==ZERO
    clock[0]+=.4;model.tick()
    assert not model.armed and "中断" in model.message


@pytest.mark.parametrize("sample",[
    ((0,0), (0,), 10.),
    ((0,float("nan"),0,0), (0,0,0,0,1),10.),
    ((0,0,0,0),(0,0,0,0,1),8.),
])
def test_invalid_gamepad_data_cannot_create_motion(sample):
    with pytest.raises(ValueError):joystick_input(sample,10.,"omni",(1,3,0),4)


def test_ackermann_two_axis_gamepad_does_not_require_unused_lateral_axis():
    values,held=joystick_input(((.5,1),(1,),10.),10.,"ackermann",(1,3,0),0)
    assert values==(1,0,.5) and held


def test_explicit_stop_sends_zero_but_read_only_close_does_not(drive):
    model,transport,_=drive;model.stop("readonly end")
    assert transport.commands==[]
    model.stop("operator stop",force=True)
    assert transport.commands==[ZERO]


@pytest.mark.parametrize("mode", [1, 2])
def test_control_mode_is_feedback_not_a_remote_control_prerequisite(drive, mode):
    model, transport, clock = drive
    transport.state["mode"] = mode
    assert model.arm()
    model.screen_input(0, 1, True)
    model.tick()
    assert transport.commands[-1][0] > 0
    transport.state["mode"] = 2 if mode == 1 else 1
    clock[0] += .1
    model.tick()
    assert model.armed and transport.commands[-1][0] > 0

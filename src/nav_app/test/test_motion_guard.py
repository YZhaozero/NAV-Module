import math
from nav_app.features.motion_guard import MotionGuard


def ready(guard):
    guard.localization(True);guard.permit(True);guard.feedback(True,True,guard.sequence,(0.,0.,0.))


def test_no_default_motion_and_physical_stop_needs_feedback():
    now=[0.];g=MotionGuard(clock=lambda:now[0]);g.velocity((5.,2.,3.))
    state=g.tick();assert state['velocity']==[0.,0.,0.] and not state['stop_confirmed']
    ready(g);assert g.arm()
    for _ in range(30):
        now[0]+=.05;ready(g);g.velocity((5.,2.,3.));state=g.tick()
        assert math.hypot(*state['velocity'][:2])<=.300001 and abs(state['velocity'][2])<=.5
    g.permit(False);g.feedback(True,True,g.sequence,(.1,0.,0.));state=g.tick()
    assert state['velocity']==[0.,0.,0.] and not state['stop_confirmed']
    g.feedback(True,True,g.sequence,(0.,0.,0.));assert g.tick()['stop_confirmed']


def test_dropped_command_or_feedback_latches_stop_until_rearmed():
    for drop in ('command','feedback','permit','localization'):
        now=[0.];g=MotionGuard(clock=lambda:now[0]);ready(g);assert g.arm();g.velocity((.2,0.,0.))
        for _ in range(9):
            now[0]+=.05
            if drop!='feedback':g.feedback(True,True,g.sequence,(0.,0.,0.))
            if drop!='permit':g.permit(True)
            if drop!='command':g.velocity((.2,0.,0.))
            g.localization(drop!='localization');state=g.tick()
        assert not state['armed'] and state['velocity']==[0.,0.,0.]
        ready(g);g.velocity((.2,0.,0.));assert not g.tick()['armed']
        assert g.arm()


def test_invalid_values_estop_and_old_ack_cannot_enable_motion():
    now=[0.];g=MotionGuard(clock=lambda:now[0]);ready(g);assert g.arm()
    g.velocity((float('nan'),0.,0.));assert not g.tick()['armed']
    ready(g);g.feedback(True,False,g.sequence,(0.,0.,0.));assert not g.arm()
    ready(g)
    for _ in range(8):now[0]+=.05;g.tick()
    g.feedback(True,True,0,(0.,0.,0.));g.permit(True);assert not g.arm()

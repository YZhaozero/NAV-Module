"""Integration and lifecycle checks, not outdoor accuracy measurements."""
import copy
import math
import threading
import time
from types import SimpleNamespace

import pytest
from nav_app.navigation_experience import experience_for
from nav_app.outdoor import OutdoorState
from nav_app.route_assistance import RouteAssistance
from test_navigation_trial import aligned_trial, projection, transform


def prepared(tmp_path=None):
    state,now,_=aligned_trial(tmp_path)
    state.trial.projection=projection()
    # Anchor the test road at the current true odometry location, with a left turn.
    path=[[39,0],[79,0],[79,40]]
    state.set_route(dict(points=[state.trial.projection.to_bd09(transform(state.alignment,p)) for p in path],
                         source='baidu_walking',coordinate_system='BD09',title='路口测试'))
    position=state._position_now()
    state.update_conversion(position,state.trial.projection.to_bd09(transform(state.alignment,[39.04,0])))
    state.control('start')
    return state,now


def ready_result(state, timeout=3.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        snapshot=state.snapshot()
        if snapshot['route_assist']['available'] or snapshot['route_assist']['state']=='unavailable':
            return snapshot
        time.sleep(.01)
    raise AssertionError('auxiliary worker did not return')


def test_switch_persists_before_alignment_and_does_not_start_capture(tmp_path):
    state=OutdoorState(tmp_path)
    experience=experience_for(state,SimpleNamespace())
    assert not experience.snapshot()['route_assist']['enabled']
    experience.command({'action':'route_assist','value':True})
    assert experience.snapshot()['route_assist']['enabled']
    assert not experience.snapshot()['route_assist']['available']
    assert not state.requests and not state.trial.active
    assert OutdoorState(tmp_path).route_assist.enabled
    experience.command({'action':'route_assist','value':False})
    assert not OutdoorState(tmp_path).route_assist.enabled
    with pytest.raises(ValueError):experience.command({'action':'route_assist','value':'false'})


def test_failed_preference_save_restores_previous_switch(tmp_path,monkeypatch):
    state=OutdoorState(tmp_path)
    def fail(*args,**kwargs):raise OSError('disk full')
    monkeypatch.setattr(state,'_save',fail)
    with pytest.raises(ValueError):state.set_route_assistance(True)
    assert not state.route_assist.enabled and not state.requests


def test_disabled_does_not_start_worker_or_modify_base_navigation():
    state,now=prepared()
    before=state.snapshot()
    for _ in range(10):
        result=state.snapshot()
        assert result['route_assist']['state']=='disabled'
        assert result['guidance']==before['guidance']
        assert result['position']==before['position']
    assert state.route_assist._thread is None


@pytest.mark.parametrize('absolute_sigma,expected',[(7.,8.2),(18.,19.2),(None,3.2),(.05,3.2),(float('nan'),3.2)])
def test_route_uncertainty_keeps_absolute_source_floor_and_distance_since_measurement(absolute_sigma,expected):
    state,now=prepared();before=state.snapshot();assist=state.route_assist
    assist.set_enabled(True)
    # Inspect the bounded submitted request without scheduling a real worker.
    assist._thread=SimpleNamespace(is_alive=lambda:True)
    position=dict(before['position'],absolute_horizontal_sigma_m=absolute_sigma,
                  distance_since_correction_m=120.)
    try:
        assist.snapshot(state,before['guidance'],position)
        assert assist._pending['sigma']==pytest.approx(expected)
        assert state._position_now()==before['position']
        assert state.guiding and state.trial.active
    finally:assist.close()


@pytest.mark.parametrize('yaw_sigma,status,expected',[(math.radians(7)*100,'estimated',20.417304764),
    (0.,'estimated',8.2),(None,'unknown_anchor',30.),(None,'unknown_heading',30.)])
def test_route_association_includes_heading_net_displacement_without_false_zero(yaw_sigma,status,expected):
    state,now=prepared();before=state.snapshot();assist=state.route_assist;assist.set_enabled(True)
    assist._thread=SimpleNamespace(is_alive=lambda:True)
    position=dict(before['position'],absolute_horizontal_sigma_m=7.,distance_since_correction_m=120.,
        yaw_displacement_sigma_m=yaw_sigma,yaw_displacement_uncertainty=status)
    try:
        assist.snapshot(state,before['guidance'],position)
        assert assist._pending['sigma']==pytest.approx(expected)
        assert state.guiding and state.trial.active
    finally:assist.close()


def test_repeated_reads_do_not_advance_motion_or_change_base_guidance():
    state,now=prepared()
    base=state.snapshot()
    state.set_route_assistance(True)
    try:
        first=ready_result(state)
        assert first['guidance']==base['guidance'] and first['position']==base['position']
        for _ in range(20):assert state.snapshot()['route_assist']==first['route_assist']
        assert first['route_assist']['sources']==['route','lidar']
        assert not first['route_assist']['imagery_available']
    finally:state.route_assist.close()


def test_disable_discards_inflight_result_without_sensor_commands(monkeypatch):
    state,now=prepared();entered=threading.Event();release=threading.Event()
    requests=list(state.requests)
    def slow(request,runtime):
        entered.set();release.wait(3.)
        return dict(enabled=True,available=True,state='wrong_turn',message='late',t=request['stamp']),None
    monkeypatch.setattr(state.route_assist,'_evaluate',slow)
    state.set_route_assistance(True);state.snapshot()
    assert entered.wait(2.)
    # Worker does not hold state.lock: toggle and odometry remain responsive.
    state.set_route_assistance(False)
    now[0]+=.25;state.feed_odom(now[0],[39.3,0,0],[0,0,0,1])
    assert state.guiding and state.trial.active and list(state.requests)==requests
    worker=state.route_assist._thread
    release.set()
    worker.join(timeout=2.)
    assert state.snapshot()['route_assist']['state']=='disabled'
    state.route_assist.close()


def test_route_replacement_discards_old_worker_result(monkeypatch):
    state,now=prepared();entered=threading.Event();release=threading.Event()
    real=state.route_assist._evaluate
    def slow(request,runtime):
        if not entered.is_set():entered.set();release.wait(3.)
        return real(request,runtime)
    monkeypatch.setattr(state.route_assist,'_evaluate',slow)
    state.set_route_assistance(True);state.snapshot();assert entered.wait(2.)
    state.control('stop')
    assert state.snapshot()['route_assist']['state']=='waiting'
    release.set()
    time.sleep(.05)
    assert not state.snapshot()['route_assist']['available']
    state.route_assist.close()


def test_worker_exception_is_auxiliary_only(monkeypatch):
    state,now=prepared();before=state.snapshot()
    def fail(*args):raise RuntimeError('test failure')
    monkeypatch.setattr(state.route_assist,'_evaluate',fail)
    state.set_route_assistance(True)
    try:
        result=ready_result(state)
        assert result['route_assist']['state']=='unavailable'
        assert result['guidance']==before['guidance']
        assert result['position']==before['position']
        assert state.guiding and state.trial.active
    finally:state.route_assist.close()


def test_geographic_adjustment_does_not_create_stationary_motion():
    state,now=prepared();assist=state.route_assist
    context=dict(key='same',points=tuple(tuple(p) for p in state.route['points']),
                 projection=state.trial.projection,route_length=state.route['distance_m'])
    runtime=None;base=copy.deepcopy(state.alignment);positions=[]
    for i in range(6):
        a={k:base[k] for k in ('yaw','tx','ty')};a['tx']+=i*.25
        req=dict(generation=1,context=context,alignment=a,point=(39.,0.,0.),
                 quaternion=(0.,0.,0.,1.),stamp=150.+i,progress=0.,sigma=2.)
        value,runtime=assist._evaluate(req,runtime)
        positions.append(value['position'])
        assert value['completed_turns']==[]
    assert all(p==positions[0] for p in positions)


def test_start_mid_route_and_resume_do_not_count_prior_turns_as_observed():
    state,now=prepared();assist=state.route_assist
    context=dict(key='same',points=tuple(tuple(p) for p in state.route['points']),
                 projection=state.trial.projection,route_length=state.route['distance_m'])
    req=dict(generation=1,context=context,alignment=state.alignment,point=(69.,0.,0.),
             quaternion=(0.,0.,0.,1.),stamp=150.,progress=30.,sigma=2.)
    value,runtime=assist._evaluate(req,None)
    assert value['progress_m']>25. and value['state']!='off_route'
    req.update(generation=2,point=(79.,20.,0.),progress=60.,stamp=151.)
    value,runtime=assist._evaluate(req,runtime)
    assert value['completed_turns']==[]


def test_odom_loss_clears_auxiliary_prompt_and_preserves_switch():
    state,now=prepared();state.set_route_assistance(True)
    try:
        assert ready_result(state)['route_assist']['available']
        now[0]+=2.
        result=state.snapshot()['route_assist']
        assert result['enabled'] and not result['available'] and result['state']=='waiting'
        assert state.guiding and state.trial.active
    finally:state.route_assist.close()


@pytest.mark.parametrize('field,delta', [('tx',3.1),('yaw',math.radians(2.1))])
def test_alignment_change_discards_inflight_old_frame_result(monkeypatch,field,delta):
    state,now=prepared();assist=state.route_assist
    base=state.snapshot()
    def read():
        # Keep base guidance valid to isolate the auxiliary frame race. The
        # ordinary position arbiter may independently suspend a large jump.
        with state.lock:
            return assist.snapshot(state,base['guidance'],base['position'])
    entered=threading.Event();release_old=threading.Event()
    new_entered=threading.Event();release_new=threading.Event()
    calls=[]
    def slow(request,runtime):
        calls.append(request)
        if len(calls)==1:
            entered.set();assert release_old.wait(3.)
            message='obsolete frame'
        else:
            new_entered.set();assert release_new.wait(3.)
            message='current frame'
        return dict(enabled=True,available=True,state='continue',message=message,
                    t=request['stamp']),None
    monkeypatch.setattr(assist,'_evaluate',slow)
    state.set_route_assistance(True)
    try:
        read();assert entered.wait(2.)
        old_generation=assist._generation
        state.alignment=dict(state.alignment,**{field:state.alignment[field]+delta})
        # No new odometry is required to invalidate an old geographic frame.
        result=read()
        assert assist._generation==old_generation+1
        assert not result['available'] and assist._result is None
        release_old.set();assert new_entered.wait(2.)
        # Old work finished, but cannot publish while current work is pending.
        assert assist._result is None
        assert not read()['available']
        release_new.set()
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            result=read()
            if result['available']:break
            time.sleep(.01)
        assert result['message']=='current frame'
        assert calls[0]['generation']!=calls[1]['generation']
    finally:
        release_old.set();release_new.set();assist.close()
        if assist._thread:assist._thread.join(timeout=2.)


@pytest.mark.parametrize('field,step', [('tx',1.1),('yaw',math.radians(.75))])
def test_small_alignment_changes_accumulate_from_window_initial_frame(field,step):
    state,now=prepared();assist=state.route_assist
    state.set_route_assistance(True)
    try:
        assert ready_result(state)['route_assist']['available']
        generation=assist._generation;initial=state.alignment[field]
        for index in (1,2):
            state.alignment=dict(state.alignment,**{field:initial+index*step})
            state.snapshot()
            assert assist._generation==generation
            assert assist._context['alignment'][field]==initial
        state.alignment=dict(state.alignment,**{field:initial+3*step})
        result=state.snapshot()['route_assist']
        assert assist._generation==generation+1
        assert not result['available']
        assert assist._context['alignment'][field]==initial+3*step
    finally:assist.close()

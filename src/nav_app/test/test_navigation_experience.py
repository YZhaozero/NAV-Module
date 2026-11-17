"""Backend truth/async/HTTP regressions; synthetic inputs never enter production."""
import http.client
import json
import math
import threading
import time
from types import SimpleNamespace

import pytest
from nav_app.navigation_experience import experience_for
from nav_app.navigation_trial import LocalMapProjection, transform, inverse
from nav_app.alignment_view import preview_fit
from nav_app.outdoor_service import MapServices
from nav_app.phone_remote import RemoteState, RemoteServer
from test_navigation_trial import make_state, aligned_trial, feed, projection


def prepared():
    s,now,world=aligned_trial()
    s.trial.projection=projection()
    position=s._position_now()
    p=s.trial.projection.to_bd09(transform(s.alignment,[39,0]))
    s.update_conversion(position,p)
    service=SimpleNamespace(plan_waypoints=lambda points,manual=False:dict(
        points=points,waypoints=points,source='manual' if manual else 'baidu_walking',
        coordinate_system='BD09',title='步行路线'),browser_key='')
    e=experience_for(s,service)
    return s,now,world,e


def wait_plan(e):
    deadline=time.monotonic()+3
    while e.busy and time.monotonic()<deadline:time.sleep(.01)
    assert not e.busy


def test_stale_phone_page_cannot_change_new_journey():
    s, now, world, e = prepared()
    old_trial = s.trial.identifier
    s.trial.identifier = old_trial + '-new'
    original_policy = s.trial.quality_policy
    for action in ('quality', 'manual_anchor', 'begin', 'save', 'select', 'stop'):
        with pytest.raises(ValueError, match='行程已变化'):
            e.command(dict(action=action, trial_id=old_trial, value='fixed',
                           point=[116., 40.], direction=[116.001, 40.]))
    assert s.trial.quality_policy == original_policy
    assert not e.locating and not e.waypoints
    assert e.command(dict(action='quality', trial_id=s.trial.identifier,
                          value='fixed')) == {'ok': True}


def test_unaligned_or_ended_trial_can_save_actual_capture_and_open_preview():
    s, now, world = make_state()
    e = experience_for(s, None)
    assert not e.ready()
    e.runtime = dict(capture_active=True, can_save=True, can_restart=True,
                     can_discard=True, saving=False)
    s.trial.finish(s)
    for action in ('save', 'camera', 'back', 'restart', 'discard'):
        assert e.command({'action': action}) == {'ok': True}
        assert s.requests[-1]['command'] == action
    e.runtime.update(saving=True, can_save=False, can_restart=False, can_discard=False)
    for action in ('save', 'restart', 'discard'):
        with pytest.raises(ValueError):
            e.command({'action': action})
    # Inspection and return remain usable while disk saving is in progress.
    assert e.command({'action': 'camera'}) == {'ok': True}
    assert e.command({'action': 'back'}) == {'ok': True}


def test_first_eligible_fix_anchors_preview_but_is_not_a_rigid_fit_constraint():
    s,now,world=make_state()
    for i in range(5):feed(s,now,world,i,quality=2)
    assert s.origin is None and s._position_now() is None
    assert len(s.track)==5 and len(s.local_track)==5
    feed(s,now,world,5,quality=4)
    assert s.origin and s.trial.display_anchor
    assert s.trial.visual_snapshot(s)['unoriented']
    assert s.alignment is None
    a=dict(yaw=.8,tx=11.,ty=-4.)
    points=[[i, max(0,i-10)] for i in range(20)]
    pairs=[(p,transform(a,p)) for p in points]
    pairs[0][1][0]+=2
    fit=preview_fit(pairs)
    assert math.dist(transform(fit,points[0]),pairs[0][1])>1.5
    for p in points:assert math.dist(inverse(a,transform(a,p)),p)<1e-12


def test_weighted_fit_reduces_float_bias_without_changing_handedness():
    points=[[i, max(0,i-10)] for i in range(30)]
    a=dict(yaw=-1.2,tx=11.,ty=4.)
    pairs=[(p,transform(a,p)) for p in points]
    weights=[1. if i%2==0 else .2 for i in range(len(points))]
    for i,(_,q) in enumerate(pairs):
        if i%2:q[0]+=.8
    weighted=preview_fit(pairs,weights);plain=preview_fit(pairs)
    assert weighted['tx']-a['tx'] < plain['tx']-a['tx']
    assert abs(weighted['yaw']-a['yaw'])<.01
    # East/north reversal from a swapped converter result must not pass validation.
    with pytest.raises(ValueError):
        LocalMapProjection([116.,40.,0.],[[116.,40.],[116.,40.002],[116.,39.998],[116.002,40.],[115.998,40.]])


def test_shared_plan_guide_coast_replan_keeps_capture_active_and_actual_position():
    s,now,world,e=prepared()
    assert experience_for(s,e.services) is e
    e.command({'action':'edit'});start=e.waypoints[0]
    target=s.trial.projection.to_bd09(transform(s.alignment,[70,10]))
    e.command({'action':'select','point':target})
    e.command({'action':'plan'});wait_plan(e)
    assert e.snapshot()['can_guide']
    e.command({'action':'guide'});assert s.guiding
    e.command({'action':'rtk','value':False})
    assert s.trial.active and s.guiding and not s.trial.rtk_enabled
    for i in range(40,44):feed(s,now,world,i,quality=2)
    moved=s.trial.projection.to_bd09(transform(s.alignment,[43,0]))
    s.update_conversion(s._position_now(),moved)
    e.command({'action':'replan'})
    assert not s.guiding and s.trial.active
    assert e.waypoints==[moved,target] and e.waypoints[0]!=start
    assert e.snapshot()['route'] is None
    assert all(r['action'] in ('road_tracking_start','road_tracking_stop') for r in s.requests)


@pytest.mark.parametrize('delay', [.8, 5.])
def test_route_editing_survives_delayed_odometry_but_guidance_still_needs_live_position(delay):
    s, now, world, e = prepared()
    s.odom_instance = 'same-running-mapper'
    e.command({'action': 'edit'})
    target = s.trial.projection.to_bd09([30, 60])
    now[0] += delay
    assert not e.ready()
    assert e.snapshot()['phase'] == 'plan'
    e.command({'action': 'select', 'point': target})
    e.command({'action': 'manual', 'value': True})
    e.command({'action': 'plan'}); wait_plan(e)
    result = e.snapshot()
    assert result['waypoints'][-1] == target and result['route']
    assert result['can_plan'] and not result['can_guide']
    with pytest.raises(ValueError, match='运动跟踪'):
        e.command({'action': 'guide'})
    # Recovery uses fresh current data and preserves the confirmed route.
    feed(s, now, world, 45, quality=2)
    s.update_conversion(s._position_now(), s.trial.projection.to_bd09([45, 0]))
    assert e.snapshot()['can_guide']


@pytest.mark.parametrize('change', ['reset', 'finish', 'saving', 'restarting'])
def test_route_editing_does_not_cross_invalidated_reference_or_capture_lifecycle(change):
    s, now, world, e = prepared()
    e.command({'action': 'edit'})
    original = list(e.waypoints)
    if change == 'reset': s.reset_odometry()
    elif change == 'finish': s.trial.finish(s)
    else: e.runtime[change] = True
    with pytest.raises(ValueError):
        e.command({'action': 'select', 'point': [116., 40.]})
    assert e.waypoints == original


@pytest.mark.parametrize('change',['edit','quality','restart','finish'])
def test_slow_network_result_cannot_commit_after_session_changes(change):
    s,now,world,e=prepared();entered=threading.Event();release=threading.Event()
    original=e.services.plan_waypoints
    def slow(*args,**kwargs):
        entered.set();assert release.wait(2);return original(*args,**kwargs)
    e.services.plan_waypoints=slow
    e.command({'action':'edit'})
    e.command({'action':'select','point':s.trial.projection.to_bd09([30,60])})
    e.command({'action':'plan'});assert entered.wait(1)
    # Can acquire the state lock and feed odometry while the request is waiting.
    feed(s,now,world,40)
    if change=='edit':e.command({'action':'undo'})
    elif change=='quality':e.command({'action':'quality','value':'fixed'})
    elif change=='restart':s.reset_odometry()
    else:s.trial.finish(s)
    release.set();wait_plan(e)
    assert s.route is None and e.route_token is None
    assert not e.snapshot()['can_guide']


def test_manual_path_and_walking_api_never_use_driving_or_commit_partial_legs():
    calls=[]
    services=MapServices()
    def call(action,data):
        calls.append((action,data))
        if len(calls)==2:raise ValueError('第二段无步行道路')
        return {'points':[data['start'],data['end']]}
    services.call=call
    points=[[116.,40.],[116.0002,40.],[116.0002,40.0002]]
    with pytest.raises(ValueError):services.plan_waypoints(points)
    assert [c[0] for c in calls]==['walking','walking']
    route=services.plan_waypoints(points,manual=True)
    assert route['source']=='manual' and route['points']==points and len(calls)==2


def test_route_replaced_elsewhere_is_not_started_as_the_screen_draft():
    s,now,world,e=prepared()
    e.command({'action':'edit'})
    e.command({'action':'select','point':s.trial.projection.to_bd09([0,70])})
    e.command({'action':'plan'});wait_plan(e)
    assert e.snapshot()['can_guide']
    s.set_route(dict(s.route,points=[s.trial.projection.to_bd09([0,0]),s.trial.projection.to_bd09([30,0])]))
    assert e.snapshot()['route'] is None
    with pytest.raises(ValueError):e.command({'action':'guide'})


def test_new_capture_does_not_show_previous_selection_or_queue_duplicate_start():
    s,now,world,e=prepared()
    e.command({'action':'edit'})
    e.command({'action':'select','point':s.trial.projection.to_bd09([0,70])})
    s.trial.finish(s)
    e.command({'action':'begin'});e.command({'action':'begin'})
    assert sum(r.get('command')=='begin' for r in s.requests)==1
    s.trial.start(s)
    value=e.snapshot()
    assert value['waypoints']==[] and value['route'] is None and not e.editing


def test_closed_walking_loop_does_not_arrive_at_start_or_jump_to_final_leg():
    s,now,world,e=prepared()
    p=s.trial.projection.to_bd09
    route=[p(x) for x in ([0,0],[30,0],[30,30],[0,30],[0,0])]
    s.set_route(dict(points=route,source='manual',coordinate_system='BD09',title='环线'))
    assert not s._guidance(p([0,0]))['near_destination']
    for point in ([10,0],[20,0],[30,0],[30,10],[30,20],[30,30],[20,30],[10,30],[0,30],[0,20],[0,10],[0,2]):
        now[0]+=3
        result=s._guidance(p(point))
    assert result['near_destination']


def test_downgraded_gnss_is_reported_and_gnss_only_guidance_has_readable_instruction():
    s,now,world,e=prepared()
    feed(s,now,world,40,quality=2)
    assert not s.trial.snapshot(s)['satellite_live']
    assert '雷达接续' in s.trial.snapshot(s)['phase']
    # Exercise the legacy GNSS-only walking path, where no body heading exists.
    s.trial.finish(s)
    feed(s,now,world,41,quality=4)
    p=s.position
    start=[116.35,39.99]
    s.set_route(dict(points=[start,[116.3505,39.99]],source='manual',coordinate_system='BD09',title='步行'))
    s.update_conversion(p,start);s.guiding=True
    assert s.snapshot()['guidance']['instruction']=='沿步行路线前行'


def test_http_experience_preserves_pairing_host_and_origin_controls():
    s,now,world,e=prepared();remote=RemoteState()
    remote.publish(b'image','view',800,480)
    server=RemoteServer(remote,'127.0.0.1',0,[],outdoor=s,map_services=e.services)
    def request(path,method='GET',data=None,headers=None):
        c=http.client.HTTPConnection('127.0.0.1',server.port)
        c.request(method,path,json.dumps(data) if data is not None else None,headers or {})
        r=c.getresponse();value=r.read();status=r.status;c.close();return status,value
    try:
        assert request('/navigation')[0]==200
        assert request('/navigation/experience')[0]==401
        token=remote.pair(remote.code)
        headers={'Cookie':'nav_remote='+token,'Content-Type':'application/json'}
        status,raw=request('/navigation/experience',headers=headers)
        assert status==200 and json.loads(raw)['trial']['aligned']
        assert request('/navigation/experience/control','POST',{'action':'edit'},headers)[0]==200
        bad=dict(headers,Origin='http://untrusted.invalid')
        assert request('/navigation/experience/control','POST',{'action':'rtk','value':False},bad)[0]==403
        assert s.trial.rtk_enabled
        assert request('/navigation/state',headers=headers)[0]==200  # Legacy API remains usable.
    finally:server.close()

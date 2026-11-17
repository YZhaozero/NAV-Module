"""Controlled observation replay; no claim of outdoor absolute accuracy."""
import math
from nav_app.outdoor import OutdoorState, GeoOrigin
from test_outdoor import fix, route
from test_geo_maps import CAL


def aligned():
    now=[100.]
    state=OutdoorState(clock=lambda:now[0],calibration=CAL)
    state.origin=GeoOrigin(116.3,39.9,50.)
    for step in range(25):
        t=100+step;now[0]=t+.05
        for dt in (-.04,.04):
            state.feed_odom(t+dt,[step+dt,0,0],[0,0,0,1])
        p=state.origin.wgs84(step,0)
        state.feed_fix(fix(t,lon=p[0],lat=p[1]))
    assert state.alignment and state.selected_map is None
    state.set_route(route());state.update_conversion(state._position_now(),[116.3,39.9])
    state.control('start')
    return state,now


def advance(state,now,end):
    while now[0]+.1 < end:
        now[0]+=.1
        state.feed_odom(now[0],[now[0]-100,0,0],[0,0,0,1])
        state.sample_actual_track()  # Match the application's continuous poll.
    return state._position_now()


def test_two_minutes_without_satellite_keep_route_and_report_estimated_position():
    state,now=aligned();session=state.session_id
    pose=advance(state,now,245.)
    assert pose['source']=='gnss_lio' and pose['coasting']
    assert pose['accuracy_status']=='estimated' and pose['correction_age_s']>120
    assert pose['distance_since_correction_m']>120
    state.update_conversion(pose,[116.3005,39.9])
    assert state.snapshot()['guidance'] and state.guiding and state.session_id==session
    assert '精度可能下降' in state.navigation_snapshot()['reason']


def test_stale_or_interrupted_odometry_does_not_resume_old_anchor():
    state,now=aligned();advance(state,now,140.)
    now[0]+=2
    assert state._position_now() is None
    state.feed_odom(now[0],[now[0]-100,0,0],[0,0,0,1])
    assert state._position_now() is None and state.guiding
    assert state.route  # Retain the destination for recovery.


def test_valid_gnss_revalidates_anchor_after_gap():
    state,now=aligned();advance(state,now,140.)
    now[0]+=2
    state.feed_odom(now[0]-.04,[now[0]-100-.04,0,0],[0,0,0,1])
    state.feed_odom(now[0]+.04,[now[0]-100+.04,0,0],[0,0,0,1])
    assert state._position_now() is None
    p=state.origin.wgs84(now[0]-100,0)
    state.feed_fix(fix(now[0],lon=p[0],lat=p[1]))
    assert state._position_now()['source']=='gnss_lio'
    assert not state._position_now()['coasting']


def test_recovered_satellite_conflict_requires_confirmation_not_old_lio():
    state,now=aligned();advance(state,now,160.)
    state.feed_odom(now[0]+.04,[now[0]-100+.04,0,0],[0,0,0,1])
    p=state.origin.wgs84(now[0]-100+20,0)
    state.feed_fix(fix(now[0],lon=p[0],lat=p[1]))
    assert state.alignment_conflict
    assert state._position_now() is None and state.position_conflict
    state.confirm_current_position()
    assert state._position_now()['source']=='gnss'
    now[0]+=2;state.feed_odom(now[0],[now[0]-100,0,0],[0,0,0,1])
    assert state._position_now() is None  # No return to the conflicting anchor.


def test_restart_and_bad_orientation_remove_geographic_coasting():
    state,now=aligned();advance(state,now,140.)
    state.feed_odom_instance('first');state.feed_odom_instance('second')
    assert state.alignment is None and state._position_now() is None
    state,now=aligned();advance(state,now,140.)
    now[0]+=.1;state.feed_odom(now[0],[40,0,0],[0,0,0,0])
    now[0]+=.1;state.feed_odom(now[0],[40.2,0,0],[0,0,0,1])
    assert state._position_now() is None


def test_phone_and_screen_share_tracking_start_stop_generation():
    state,now=aligned()
    start=state.requests.pop();assert start['action']=='road_tracking_start'
    state.control('stop');stop=state.requests.pop()
    assert stop['action']=='road_tracking_stop' and stop['generation']>start['generation']
    state.simulate('enter');state.control('start')
    assert not state.requests


def test_local_trajectory_works_without_any_satellite_fix():
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.odom_frame='camera_init'
    for i in range(30):
        now[0]=100+i*.1
        state.feed_odom(now[0],[i*.1,0,0],[0,0,math.sin(.3),math.cos(.3)])
        state.sample_actual_track()
    track=state.local_tracking_snapshot()
    assert track['fresh'] and len(track['points'])>=3
    assert track['position']==[2.9,0] or abs(track['position'][0]-2.9)<1e-9
    assert abs(track['heading_rad']-.6)<1e-9
    assert state._position_now() is None
    now[0]+=1
    assert state.local_tracking_snapshot()['position'] is None
    assert state.local_tracking_snapshot()['points']

"""Synthetic trajectory/route tests, not field accuracy validation."""
import copy
import json
import math
import random
from unittest.mock import Mock
import pytest
from nav_app.outdoor import OutdoorState, GeoOrigin
from nav_app.navigation_trial import trajectory_alignment,transform,inverse,LocalMapProjection
from nav_app.outdoor_service import MapServices,PositionConverter
from test_geo_maps import CAL


def trajectory(count=50):
    rng=random.Random(19)
    a=dict(yaw=1.13,tx=17.,ty=-31.)
    pairs=[]
    for i in range(count):
        p=[i*.8, max(0,i-30)*.6,0.]
        q=transform(a,p)
        q=[v+rng.gauss(0,.08) for v in q]+[0.]
        pairs.append((p,q))
    return a,pairs


def test_robust_unscaled_fit_rejects_outliers_and_validates_newest_samples():
    a,pairs=trajectory()
    for i in (3,12,20,31):pairs[i][1][0]+=20.
    result=trajectory_alignment(pairs)
    assert result and result['inliers']==43
    assert abs(result['yaw']-a['yaw'])<.01
    assert math.dist(transform(result,pairs[-1][0]),transform(a,pairs[-1][0]))<.15
    assert result['experimental'] and result['precision_source']=='trajectory_agreement'
    pairs[-1][1][1]+=5
    assert trajectory_alignment(pairs) is None


@pytest.mark.parametrize('kind',['stationary','scale','reverse_time','too_short'])
def test_bad_alignment_cannot_initialize(kind):
    a,pairs=trajectory()
    if kind=='stationary':pairs=[([0,0,0],[0,0,0])]*50
    elif kind=='scale':pairs=[(p,[q[0]*1.4,q[1]*1.4,0]) for p,q in pairs]
    elif kind=='reverse_time':
        # Different shapes/traversal order cannot be explained by a rigid fit.
        pairs=[(p,q) for (p,_),(_,q) in zip(pairs,reversed(pairs))]
    else:pairs=pairs[:12]
    assert trajectory_alignment(pairs) is None


def make_state(tmp_path=None):
    now=[100.]
    state=OutdoorState(tmp_path,clock=lambda:now[0],calibration=CAL,monotonic_clock=lambda:now[0])
    state.odom_frame='camera_init';state.trial.start(state)
    world=GeoOrigin(116.35,39.99,50.)
    return state,now,world


def test_position_converter_worker_starts_and_stops(tmp_path):
    state = OutdoorState(tmp_path, calibration=CAL)
    converter = PositionConverter(state, Mock())
    converter.close()
    assert not converter.thread.is_alive()


def feed(state,now,world,i,*,error=(0,0),sigma=None,quality=4):
    t=100+i;now[0]=t+.05
    # A sideways-facing handheld device. No assumption about travel/body yaw.
    for dt in (-.04,.04):state.feed_odom(t+dt,[i+dt,0,0],[0,0,math.sin(.7),math.cos(.7)])
    p=world.wgs84(15+error[0],25+i+error[1])
    state.feed_fix(dict(stamp=t,longitude=p[0],latitude=p[1],altitude_ellipsoid=50.,
        fix_quality=quality,timestamp_source='receiver_utc',horizontal_std_m=sigma))
    state.sample_actual_track()


def aligned_trial(tmp_path=None):
    state,now,world=make_state(tmp_path)
    for i in range(40):feed(state,now,world,i)
    assert state.trial.snapshot(state)['aligned']
    return state,now,world


def test_unstable_fixes_do_not_stop_trial_and_good_motion_recovers():
    state, now, world = make_state()
    for i in range(40):
        feed(state, now, world, i, error=(2.*math.sin(i), 0.))
    assert state.trial.active
    assert not state.guiding
    assert state.trial.stable_samples.rejected > 0
    assert not state.requests
    for i in range(40, 100):
        feed(state, now, world, i)
    assert state.trial.snapshot(state)['aligned']
    assert state.trial.snapshot(state)['stable_pairs'] >= 15
    assert state.trial.active


@pytest.mark.parametrize('trial', [True, False])
def test_gnss_arriving_before_odometry_is_paired_when_odometry_catches_up(trial):
    state, now, world = make_state()
    if not trial:
        state.trial.finish(state)
    for i in range(400):
        t = 100 + i * .1
        now[0] = t + .12
        p = world.wgs84(15, 25 + i * .1)
        state.feed_fix(dict(stamp=t, longitude=p[0], latitude=p[1], altitude_ellipsoid=50.,
            fix_quality=4, timestamp_source='receiver_utc', received_at=now[0],
            horizontal_std_m=.1, differential_age_s=1.))
        state.feed_odom(t - .1, [i * .1 - .1, 0, 0], [0, 0, 0, 1])
    assert state.alignment
    assert abs(state.alignment['yaw'] - math.pi / 2) < 1e-4
    assert len(state.trial.pairs if trial else state.pairs) >= 15


def test_waiting_alignment_observations_expire_and_do_not_cross_mapper_restart():
    state, now, world = make_state()
    def fix(t):
        now[0] = t + .1
        p = world.wgs84(15, 25)
        state.feed_fix(dict(stamp=t, longitude=p[0], latitude=p[1],
            fix_quality=2, timestamp_source='receiver_utc', received_at=now[0]))
    fix(100.)
    now[0] = 111.
    state.feed_odom(99.95, [0, 0, 0], [0, 0, 0, 1])
    state.feed_odom(100.05, [0, 0, 0], [0, 0, 0, 1])
    assert not state.trial.pairs
    fix(102.)
    state.reset_odometry()
    now[0] = 102.2
    state.feed_odom(101.95, [0, 0, 0], [0, 0, 0, 1])
    state.feed_odom(102.05, [0, 0, 0], [0, 0, 0, 1])
    assert not state.trial.pairs


def test_processing_backlog_preserves_continuity_and_pairs_without_showing_stale_pose():
    state, now, world = make_state()
    state.feed_odom_instance('same-estimator')
    deferred = []
    for i in range(650):
        t = 100 + i * .1
        now[0] = t + .1
        p = world.wgs84(15, 25 + i * .1)
        state.feed_fix(dict(stamp=t, longitude=p[0], latitude=p[1], altitude_ellipsoid=50.,
            fix_quality=4, timestamp_source='receiver_utc', received_at=now[0],
            horizontal_std_m=.1, differential_age_s=1.))
        deferred.append((t-.05, [i*.1-.05, 0, 0], [0, 0, 0, 1]))
        # A real, continuous LIO stream progressively falls 3 s behind.
        lag = min(30, max(0, (i-100)//3)) if i < 500 else 0
        while len(deferred) > lag:
            state.feed_odom(*deferred.pop(0))
        state.sample_actual_track()
        if i == 450:
            assert state.alignment
            assert not state.trial.snapshot(state)['odometry_live']
            assert state.trial.snapshot(state)['odometry_delayed']
            assert state.trial.snapshot(state)['phase'] == '运动跟踪延迟'
            assert state._position_now()['source'] == 'gnss'
    assert state.odom_epoch == 0
    assert state.odom_continuity == 0
    assert not any(e['kind']=='odometry_reset' for e in state.trial.events)
    assert state.trial.snapshot(state)['aligned']
    assert state.trial.snapshot(state)['odometry_live']
    assert state._position_now()['source'] == 'gnss_lio'
    assert len(state._alignment_pending) <= 256
    now[0] += 1.
    assert state.trial.snapshot(state)['odometry_delayed']
    assert not state.trial.snapshot(state)['odometry_live']
    now[0] += 1.1
    assert not state.trial.snapshot(state)['odometry_delayed']
    assert state.trial.snapshot(state)['phase'] == '运动跟踪中断'


def test_delayed_covariance_replaces_waiting_fix_without_double_pairing():
    state, now, world = make_state()
    state.trial.finish(state)
    now[0] = 100.1
    p = world.wgs84(15, 25)
    original = dict(stamp=100., longitude=p[0], latitude=p[1], altitude_ellipsoid=50.,
        fix_quality=4, timestamp_source='receiver_utc', received_at=100.1,
        differential_age_s=1., horizontal_std_m=None, covariance=None,
        covariance_source='unknown')
    state.feed_fix(original)
    state.feed_fix(dict(original, covariance_update=True, covariance_source='receiver_gst',
        horizontal_std_m=.1, covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .04]))
    state.feed_odom(99.96, [0, 0, 0], [0, 0, 0, 1])
    state.feed_odom(100.04, [.08, 0, 0], [0, 0, 0, 1])
    assert len(state.pairs) == 1
    assert len(state.track) == 1
    assert state.position_id == 1
    assert not state._alignment_pending


def test_waiting_fixes_cannot_align_across_rtk_participation_change():
    state, now, world = make_state()
    now[0] = 100.1
    p = world.wgs84(15, 25)
    state.feed_fix(dict(stamp=100., longitude=p[0], latitude=p[1], fix_quality=2,
        timestamp_source='receiver_utc', received_at=100.1))
    state.trial.set_rtk(state, False)
    state.trial.set_rtk(state, True)
    state.feed_odom(99.96, [0, 0, 0], [0, 0, 0, 1])
    state.feed_odom(100.04, [.08, 0, 0], [0, 0, 0, 1])
    assert not state.trial.pairs


def test_replay_cannot_drain_waiting_live_satellite_observations():
    state, now, world = make_state()
    now[0] = 100.1
    p = world.wgs84(15, 25)
    state.feed_fix(dict(stamp=100., longitude=p[0], latitude=p[1], fix_quality=2,
        timestamp_source='receiver_utc', received_at=100.1))
    state.sensor_replay = True
    state.feed_odom(99.96, [0, 0, 0], [0, 0, 0, 1])
    state.feed_odom(100.04, [.08, 0, 0], [0, 0, 0, 1])
    assert not state.trial.pairs
    assert not state._alignment_pending


def test_missing_gst_trial_aligns_without_upgrading_normal_reference(tmp_path):
    state,now,world=aligned_trial(tmp_path)
    assert abs(state.alignment['yaw']-math.pi/2)<1e-4
    assert state._position_now()['source']=='gnss_lio'
    assert state._position_now()['accuracy_status']=='experimental'
    reference = state.capture_reference('camera_init')
    assert reference['reference_source'] == 'trajectory_alignment'
    assert reference['alignment_evidence']['experimental']
    assert not reference['field_validated']
    assert state.initial_map_pose() is None
    state.trial.finish(state);state.save_session()
    assert state.alignment is None and not state.trial.active
    assert state.assets['navigation_trial']['alignment']['experimental']
    record=json.loads((tmp_path/'active.json').read_text(encoding='utf-8'))
    assert record['assets']['navigation_trial']['events'][-1]['kind']=='finish'
    restarted=OutdoorState(tmp_path,clock=lambda:now[0],calibration=CAL)
    assert not restarted.trial.active and restarted.alignment is None


def test_rtk_off_measures_drift_without_changing_alignment_or_receiver_data():
    state,now,world=aligned_trial();original=copy.deepcopy(state.alignment)
    state.trial.set_rtk(state,False)
    for i in range(40,65):feed(state,now,world,i,error=(2,0))
    assert state.alignment==original
    p=state._position_now()
    assert p['source']=='gnss_lio' and p['coasting']
    assert len(state.track)==65 and state.observation['stamp']==164
    report=state.trial.snapshot(state)
    assert report['comparison_count']==25
    assert 1.99 < report['comparison_rms_m'] < 2.01
    assert not report['road_constraint_applied']
    state.trial.set_rtk(state,True)
    before=transform(state.alignment,[65,0])
    for i in range(65,105):feed(state,now,world,i,error=(2,0))
    assert state.trial.last_correction_m<=.3+1e-9
    assert abs(state.trial.last_correction_deg)<=.5+1e-9
    assert math.dist(before,transform(state.alignment,[65,0]))>1.


def test_satellite_loss_keeps_mapping_motion_and_reports_coasting():
    state,now,world=aligned_trial()
    for i in range(1,101):
        now[0]=139.05+i*.1
        state.feed_odom(now[0],[39.05+i*.1,0,0],[0,0,0,1])
        state.sample_actual_track()
    assert state._position_now()['source']=='gnss_lio'
    assert state._position_now()['coasting']
    assert '雷达接续' in state.trial.snapshot(state)['phase']
    assert not state.trial.snapshot(state)['satellite_live']
    assert not state.trial.snapshot(state)['satellite_live']


def test_new_trial_archives_previous_walk_before_resetting_display(tmp_path):
    state=OutdoorState(tmp_path,clock=lambda:100.,calibration=CAL)
    state.feed_odom(100,[1,0,0],[0,0,0,1]);state.sample_actual_track()
    old_id=state.session_id
    state.trial.start(state)
    assert state.session_id!=old_id and not state.local_track
    saved=json.loads((tmp_path/f'session-{old_id}.json').read_text(encoding='utf-8'))
    assert len(saved['local_track'])==1


def test_recorded_accuracy_is_unknown_even_for_a_perfect_trajectory_fit():
    state,now,world=aligned_trial()
    assert not state.trial.snapshot(state)['accuracy_verified']
    assert not state.alignment['receiver_precision_available']
    state.trial.finish(state)
    # Ordinary strict alignment still requires fixed solution + receiver GST.
    for i in range(40,80):feed(state,now,world,i)
    assert state.alignment is None


def test_horizontal_alignment_does_not_require_ellipsoid_height():
    state,now,world=make_state()
    for i in range(40):
        t=100+i;now[0]=t+.05
        for dt in (-.04,.04):state.feed_odom(t+dt,[i+dt,0,0],[0,0,0,1])
        p=world.wgs84(0,i)
        state.feed_fix(dict(stamp=t,longitude=p[0],latitude=p[1],fix_quality=4,timestamp_source='receiver_utc'))
    assert state.trial.snapshot(state)['aligned']
    assert not state.alignment['vertical_alignment_valid']


def test_old_saved_map_cannot_secretly_supply_global_position_with_rtk_off(tmp_path):
    from test_geo_maps import restarted,match_twice
    now=[100.];state,geo=restarted(tmp_path,now);match_twice(state,now)
    assert state.navigation_snapshot()['map_navigation_ready']
    state.trial.start(state);state.trial.set_rtk(state,False)
    assert state.selected_map is not None
    assert not state.navigation_snapshot()['map_navigation_ready']
    assert not state.navigation_snapshot()['valid']
    assert state._position_now() is None


def test_off_before_alignment_has_local_track_but_no_fake_global_pose():
    state,now,world=make_state();state.trial.set_rtk(state,False)
    for i in range(30):feed(state,now,world,i)
    assert state.local_tracking_snapshot()['fresh'] and state.local_tracking_snapshot()['points']
    assert state._position_now() is None and not state.trial.snapshot(state)['aligned']


def test_tracking_gap_requires_new_correspondences_and_never_resumes_old_frame():
    state,now,world=aligned_trial();state.trial.set_rtk(state,False)
    feed(state,now,world,43)
    assert state.alignment is None and state._position_now() is None
    state.trial.set_rtk(state,True)
    feed(state,now,world,44)
    assert not state.trial.snapshot(state)['aligned']
    assert len(state.trial.pairs)==1


def test_trials_cannot_enter_from_simulation_or_active_guidance():
    state=OutdoorState()
    for key in ('sensor_replay','guiding','local_navigation_active'):
        setattr(state,key,True)
        with pytest.raises(ValueError):state.trial.start(state)
        setattr(state,key,False)


def projection():
    # A deterministic local affine map response stands in for Baidu's service.
    center=[116.356,39.996]
    convert=lambda x,y:[center[0]+x/85000+y/1e9,center[1]+y/111000-x/1e9]
    return LocalMapProjection([116.35,39.99,50.],[convert(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))])


def test_baidu_local_projection_roundtrip_and_invalid_extent():
    p=projection()
    for local in ((0,0),(100,-80),(-300,400),(700,400)):
        assert math.dist(p.to_enu(p.to_bd09(local)),local)<1e-6
    with pytest.raises(ValueError):p.to_bd09([1001,0])
    with pytest.raises(ValueError):LocalMapProjection([116.35,39.99,50.],[[116.35,39.99]]*5)


def test_converter_caches_official_samples_and_rejects_stale_origin():
    state,now,world=aligned_trial()
    converted=projection()
    service=Mock()
    service.call.return_value={'points':[converted.to_bd09([x,y]) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))]}
    converter=PositionConverter.__new__(PositionConverter);converter.state=state;converter.services=service
    converter.convert(state._position_now());feed(state,now,world,40)
    converter.convert(state._position_now())
    assert service.call.call_count==1
    state.trial.projection=None
    def changed(*args):
        state.origin=GeoOrigin(116,39,50)
        return service.call.return_value
    service.call.side_effect=changed
    with pytest.raises(ValueError):converter.convert(state._position_now())
    assert state.trial.projection is None


def test_route_in_map_frame_preserves_rtk_off_and_actual_tracks():
    state,now,world=aligned_trial();state.trial.projection=projection()
    local=[[39.,0.],[60.,0.],[60.,20.]]
    bd=[state.trial.projection.to_bd09(transform(state.alignment,p)) for p in local]
    count=len(state.local_track)
    state.set_route(dict(points=bd,source='manual',coordinate_system='BD09',title='多点路线',waypoints=bd))
    assert len(state.local_track)==count
    assert all(math.dist(a,b)<1e-6 for a,b in zip(state.trial.route_in_odom(state),local))
    state.update_conversion(state._position_now(),bd[0]);state.control('start')
    state.trial.set_rtk(state,False)
    feed(state,now,world,40)
    state.update_conversion(state._position_now(),state.trial.projection.to_bd09(transform(state.alignment,[40,0])))
    assert state.snapshot()['guidance']['remaining_m']>0
    assert state.guiding and state.route['waypoints']==bd


def test_multi_point_walking_segments_order_manual_and_failure_atomicity():
    svc=MapServices(url='',token='')
    points=[[116.35,39.99],[116.3502,39.99],[116.3502,39.9902],[116.35,39.99]]
    svc.plan=Mock(side_effect=lambda a,b:dict(points=[a,b]))
    route=svc.plan_waypoints(points)
    assert route['points']==points and route['waypoints']==points
    assert [c.args for c in svc.plan.call_args_list]==list(zip(points,points[1:]))
    svc.plan.reset_mock();manual=svc.plan_waypoints(points,manual=True)
    assert manual['source']=='manual' and not svc.plan.called
    svc.plan.side_effect=[dict(points=points[:2]),ValueError('测试联网失败')]
    with pytest.raises(ValueError,match='第2段'):svc.plan_waypoints(points)

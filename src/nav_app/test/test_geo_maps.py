"""Synthetic coordinates/fault injection, not outdoor accuracy evidence."""
import copy
import json
import math
from pathlib import Path
from unittest.mock import patch
import pytest
from nav_app.outdoor import OutdoorState, GeoOrigin, distance
from nav_app.geo_maps import file_hash


CAL=dict(time_verified=True,lever_arm_verified=True,antenna_in_body_m=[0,0,0],
         gnss_to_odom_offset_s=0.,odom_child_frame='base_link')


def trained(tmp_path, now):
    state=OutdoorState(tmp_path,clock=lambda:now[0],calibration=CAL)
    state.odom_frame='camera_init'
    state.origin=GeoOrigin(116.3,39.9,50.)
    for i in range(25):
        t=100+i;now[0]=t+.05
        for dt in (-.04,.04):
            state.feed_odom(t+dt,[i+dt,0,0],[0,0,0,1])
        p=state.origin.wgs84(20,30+i)
        state.feed_fix(dict(stamp=t,longitude=p[0],latitude=p[1],altitude_ellipsoid=50.,
                           fix_quality=4,timestamp_source='receiver_utc',horizontal_std_m=.1,differential_age_s=1.))
        state.sample_actual_track()
    assert state.alignment
    cloud=tmp_path/'map.pcd';cloud.write_bytes(b'synthetic pointcloud identity\n')
    result=state.register_capture(dict(map=str(cloud)),state.capture_reference('camera_init'))
    assert result
    return state,result


def quality(state, now, sequence=1, **extra):
    data=dict(valid=True,published_stamp_ns=round(now[0]*1e9),last_success_stamp_ns=round(now[0]*1e9),
              match_sequence=sequence,instance_id='localizer-1',map_sha256=state.selected_map['sha256'],
              map_frame='map',odom_frame=state.odom_frame,data_age_s=.01,fitness=.08,
              map_from_odom=dict(translation_m=[0,0,0],rotation_xyzw=[0,0,0,1]))
    return dict(data,**extra)


def restarted(tmp_path, now):
    state,geo=trained(tmp_path,now)
    state.select_geo_map(geo['id'])
    now[0]=200.
    state=OutdoorState(tmp_path,clock=lambda:now[0],calibration=CAL)
    state.odom_frame='camera_init'
    state.feed_odom(200,[24,0,0],[0,0,0,1])
    return state,geo


def match_twice(state,now,**extra):
    assert not state.feed_map_match(quality(state,now,sequence=1,**extra))
    assert state.feed_map_match(quality(state,now,sequence=2,**extra))


def test_first_walk_save_restart_map_match_without_satellites(tmp_path):
    now=[100.];state,geo=restarted(tmp_path,now)
    assert state.alignment is None
    assert state.snapshot()['position'] is None
    assert not state.guiding
    match_twice(state,now)
    pose=state.snapshot()['position']
    expected=GeoOrigin(116.3,39.9,50).wgs84(20,54)
    assert pose['source']=='map_lio'
    assert distance(pose['wgs84'],expected)<.002
    assert abs(pose['heading_enu_rad']-math.pi/2)<.001
    assert geo['field_validated'] is False
    assert geo['local_track'] and geo['assets']['map']
    now[0]+=1.1
    state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert state.snapshot()['position'] is None  # frozen quality can't keep live pose


@pytest.mark.parametrize('bad',[
    dict(valid=False),dict(map_sha256='a'*64),dict(odom_frame='other'),dict(map_frame='earth'),
    dict(published_stamp_ns=100_000_000_000),dict(last_success_stamp_ns=198_000_000_000),
    dict(data_age_s=3.),dict(fitness=float('nan')),dict(match_sequence=-1),
    dict(map_from_odom=None),dict(map_from_odom=dict(translation_m=[0,0,0],rotation_xyzw=[0,0,0,0]))])
def test_wrong_stale_or_invalid_map_never_guides(tmp_path,bad):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now)
    assert not state.feed_map_match(quality(state,now,sequence=3,**bad))
    assert state.snapshot()['position'] is None


def test_duplicate_quality_not_two_matches_and_process_restart_needs_recheck(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    for _ in range(8):assert not state.feed_map_match(quality(state,now))
    assert state.snapshot()['position'] is None
    assert state.feed_map_match(quality(state,now,2))
    assert not state.feed_map_match(quality(state,now,1,instance_id='new-process'))
    assert state.snapshot()['position'] is None


@pytest.mark.parametrize('age,available',[(.6,True),(1.,True),(1.001,False)])
def test_map_pose_uses_localizer_one_second_odom_budget(tmp_path,age,available):
    now=[100.];state,_=restarted(tmp_path,now)
    now[0]+=age
    match_twice(state,now,tracking_timeout_s=5.)
    assert (state.local_map_pose() is not None) is available


def test_brief_input_delay_pauses_pose_without_repeating_initial_confirmation(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now,tracking_timeout_s=5.)
    success=round(now[0]*1e9)
    now[0]+=.2
    delayed=quality(state,now,2,valid=False,data_age_s=1.1,
                    last_success_stamp_ns=success,tracking_timeout_s=5.,
                    operation='tracking',failures=0,match_age_s=.2)
    assert not state.feed_map_match(delayed)
    assert state.local_map_pose() is None
    assert '延迟' in state.map_reason
    now[0]+=.1
    state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert state.feed_map_match(quality(state,now,2,last_success_stamp_ns=success,
                                      tracking_timeout_s=5.))
    assert state.local_map_pose() is not None


@pytest.mark.parametrize('bad',[
    dict(failures=3),dict(input_error='invalid_cloud_layout'),
    dict(verifying_initial_pose=True),dict(initial_pose_required=True),
    dict(operation='manual'),dict(operation='global'),
    dict(instance_id='restarted'),dict(map_sha256='a'*64),
    dict(match_sequence=1),dict(map_from_odom=None),
])
def test_nontransient_invalidity_requires_two_new_matches(tmp_path,bad):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now,tracking_timeout_s=5.)
    now[0]+=.1
    invalid=dict(quality(state,now,2,valid=False,data_age_s=1.1,
                        tracking_timeout_s=5.,operation='tracking',
                        failures=0,match_age_s=.1),**bad)
    assert not state.feed_map_match(invalid)
    assert state.local_map_pose() is None
    assert not state.feed_map_match(quality(state,now,3,tracking_timeout_s=5.))
    assert state.feed_map_match(quality(state,now,4,tracking_timeout_s=5.))


def test_delayed_heartbeats_do_not_extend_confirmation_lifetime(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now,tracking_timeout_s=5.)
    success=round(now[0]*1e9)
    for elapsed in (1.,4.9,5.01):
        now[0]=200.+elapsed
        assert not state.feed_map_match(quality(state,now,2,valid=False,
            data_age_s=1.1,last_success_stamp_ns=success,
            tracking_timeout_s=5.,operation='tracking',failures=0,match_age_s=elapsed))
        assert state.local_map_pose() is None
    state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert not state.feed_map_match(quality(state,now,3,tracking_timeout_s=5.))
    assert state.feed_map_match(quality(state,now,4,tracking_timeout_s=5.))


def test_odometry_reset_during_delay_discards_old_confirmation(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now,tracking_timeout_s=5.)
    old_success=round(now[0]*1e9)
    now[0]+=.2
    assert not state.feed_map_match(quality(state,now,2,valid=False,
        data_age_s=1.1,last_success_stamp_ns=old_success,
        tracking_timeout_s=5.,operation='tracking',failures=0))
    state.reset_odometry()
    state.feed_odom(now[0],[0,0,0],[0,0,0,1])
    assert not state.feed_map_match(quality(state,now,2,
        last_success_stamp_ns=old_success,tracking_timeout_s=5.))
    assert not state.feed_map_match(quality(state,now,3,tracking_timeout_s=5.))
    assert state.feed_map_match(quality(state,now,4,tracking_timeout_s=5.))


def test_reset_rejects_transform_from_previous_odometry(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    old=quality(state,now,5)
    now[0]+=1;state.reset_odometry();state.feed_odom(now[0],[0,0,0],[0,0,0,1])
    assert not state.feed_map_match(old)
    assert state.snapshot()['position'] is None
    assert not state.local_track


def test_match_observation_deadline_shared_without_heartbeat_renewal(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    match_twice(state,now,tracking_timeout_s=5.)
    stamp=round(now[0]*1e9)
    now[0]+=3.
    state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert state.feed_map_match(quality(state,now,2,last_success_stamp_ns=stamp,tracking_timeout_s=5.))
    assert state.local_map_pose() is not None
    now[0]+=2.01
    state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert not state.feed_map_match(quality(state,now,2,last_success_stamp_ns=stamp,tracking_timeout_s=5.))
    assert state.local_map_pose() is None


@pytest.mark.parametrize('lifetime',[0,-1,5.01,float('nan'),float('inf'),True,'5'])
def test_invalid_match_lifetime_cannot_extend_pose(tmp_path,lifetime):
    now=[100.];state,_=restarted(tmp_path,now)
    assert not state.feed_map_match(quality(state,now,2,tracking_timeout_s=lifetime))
    assert state.local_map_pose() is None


def test_changed_pcd_rejected_without_overwriting_selection(tmp_path):
    now=[100.];state,geo=trained(tmp_path,now)
    Path(geo['cloud_path']).write_bytes(b'edited geometry')
    with pytest.raises(ValueError,match='内容已变化'):state.select_geo_map(geo['id'])
    assert state.selected_map is None


def test_select_atomic_failure_and_bad_identifier(tmp_path):
    now=[100.];state,geo=trained(tmp_path,now)
    with patch.object(state,'_save',side_effect=OSError('disk full')):
        with pytest.raises(OSError):state.select_geo_map(geo['id'])
    assert state.selected_map is None
    with pytest.raises(ValueError):state.select_geo_map({'id':'bad'})


def test_capture_no_fake_reference_and_final_bag_assets(tmp_path):
    now=[100.];state,geo=trained(tmp_path,now)
    assert state.capture_reference('other') is None
    state.calibration=dict(CAL,time_verified=False)
    assert state.capture_reference('camera_init') is None
    state.finish_capture_assets(dict(map=geo['cloud_path'],bag='/real/session',bag_complete=True))
    saved=json.loads((tmp_path/'maps'/(geo['id']+'.json')).read_text(encoding='utf-8'))
    assert saved['assets']['bag']=='/real/session'
    assert saved['local_track']
    assert 'local_track' not in state.maps.entries[geo['id']]
    assert saved['sha256']==file_hash(geo['cloud_path'])


def test_map_and_reliable_gnss_conflict_pauses_instead_of_snapping(tmp_path):
    now=[100.];state,geo=trained(tmp_path,now);state.select_geo_map(geo['id'])
    # Map pose 10 m away, fresh GNSS-LIO still at original coordinates.
    match_twice(state,now,map_from_odom=dict(translation_m=[10,0,0],rotation_xyzw=[0,0,0,1]))
    assert state.snapshot()['position'] is None
    assert state.snapshot()['geographic']['conflict']
    assert state.snapshot()['guidance'] is None


def test_large_source_change_requires_explicit_confirmation(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    prior=state.snapshot()['position'];assert prior
    state.update_conversion(prior,[116.306,39.906])
    now[0]+=3.
    state.position=dict(wgs84=[116.31,39.9],stamp=now[0],id=1,source='gnss',quality=1)
    assert state.snapshot()['position'] is None
    assert state.pending_pose
    state.confirm_current_position()
    assert state.snapshot()['position']['source']=='gnss'
    assert state.converted is None
    state.update_conversion(prior,[116.306,39.906])
    assert state.converted is None  # late network response from old source


def test_initial_pose_is_prior_and_preserves_body_orientation(tmp_path):
    now=[100.];state,geo=trained(tmp_path,now);state.select_geo_map(geo['id'])
    pose=state.initial_map_pose()
    assert pose['position_m']==pytest.approx([24.04,0,0],abs=.003)
    assert pose['quaternion_xyzw']==pytest.approx([0,0,0,1],abs=.001)
    assert state.map_match is None
    now[0]+=3
    assert state.initial_map_pose() is None


def test_incomplete_manifest_is_quarantined(tmp_path):
    d=tmp_path/'maps';d.mkdir();(d/'broken.json').write_text('{')
    state=OutdoorState(tmp_path)
    assert state.maps.errors==1 and not state.maps.entries


def test_localization_commands_isolated_from_robot_and_simulation(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now)
    state.mode='robot'
    with pytest.raises(ValueError):state.request_localization('start')
    state.mode='walking';state.request_localization('start')
    assert list(state.requests)==['start']
    with pytest.raises(ValueError):state.request_localization('start')


def test_sensor_replay_never_claims_live_geographic_position(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    assert state.snapshot()['position']
    state.sensor_replay=True
    assert state.snapshot()['position'] is None
    assert state.initial_map_pose() is None


def test_map_relocalization_jump_requires_confirmation(tmp_path):
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    assert state.snapshot()['position']
    now[0]+=.1;state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert state.feed_map_match(quality(state,now,3,map_from_odom=dict(translation_m=[20,0,0],rotation_xyzw=[0,0,0,1])))
    assert state.snapshot()['position'] is None
    assert state.snapshot()['geographic']['can_confirm_position']


def test_map_only_track_archived_before_route_replacement(tmp_path):
    from test_outdoor import route
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    state.set_route(route());state.sample_actual_track()
    session=state.session_id
    assert not state.track and state.estimated_track
    state.set_route(dict(route(),title='New route'))
    archived=json.loads((tmp_path/f'session-{session}.json').read_text(encoding='utf-8'))
    assert archived['route']['title']=='验证路线'
    assert archived['estimated_track'][0]['source']=='map_lio'
    assert not state.estimated_track and not state.local_track


def test_heading_gives_relative_turn_only_after_alignment(tmp_path):
    from test_outdoor import route
    now=[100.];state,_=restarted(tmp_path,now);match_twice(state,now)
    state.set_route(route());pose=state.snapshot()['position']
    state.update_conversion(pose,[116.3,39.9]);state.control('start')
    guidance=state.snapshot()['guidance']
    # Map heading points north, requested first segment points east.
    assert abs(guidance['relative_turn_deg']-90)<=1
    assert '右' in guidance['instruction']


def test_active_index_bounded_while_archive_keeps_history(tmp_path):
    now=[100.];state,_=trained(tmp_path,now)
    state.estimated_track=[dict(stamp=i,wgs84=[116.3,39.9],source='map_lio') for i in range(3600)]
    state.save_session()
    active=json.loads((tmp_path/'active.json').read_text(encoding='utf-8'))
    archive=json.loads((tmp_path/f'session-{state.session_id}.json').read_text(encoding='utf-8'))
    assert len(active['estimated_track'])==60
    assert len(archive['estimated_track'])==3600
    assert not OutdoorState(tmp_path).save_error


def test_map_without_any_satellites_is_saved_reloaded_and_located(tmp_path):
    now=[200.]
    state=OutdoorState(tmp_path,clock=lambda:now[0])
    cloud=tmp_path/'no-gnss.pcd';cloud.write_bytes(b'local geometry')
    entry=state.register_capture(dict(map=str(cloud),name='房间和门外通道'),None)
    assert entry['reference_source']=='local_only'
    assert 'origin_wgs84' not in entry
    state.select_geo_map(entry['id'])
    state=OutdoorState(tmp_path,clock=lambda:now[0])
    state.odom_frame='camera_init';state.feed_odom(now[0],[1,2,0],[0,0,0,1])
    match_twice(state,now)
    status=state.navigation_snapshot()
    assert status['valid'] and status['map_navigation_ready']
    assert not status['geographic_navigation_ready']
    assert status['local_pose']['position_m']==[1,2,0]
    assert state.snapshot()['position'] is None and state.initial_map_pose() is None
    assert not state.snapshot()['geographic']['maps'][0]['georeferenced']
    now[0]+=3;state.feed_odom(now[0],[1,2,0],[0,0,0,1])
    assert not state.navigation_snapshot()['valid']  # odom alone cannot certify global pose


def test_local_map_can_be_anchored_later_with_tilt_and_without_reset(tmp_path):
    from nav_app.geo_maps import rotate, multiply
    now=[100.];state,_=trained(tmp_path,now)
    cloud=tmp_path/'local-first.pcd';cloud.write_bytes(b'local first')
    entry=state.register_capture(dict(map=str(cloud)),None);state.select_geo_map(entry['id'])
    # Deliberately non-planar map<-odom transform: do not discard mounting/map tilt.
    q=[0,math.sin(math.radians(15)),0,math.cos(math.radians(15))]
    transform=dict(translation_m=[2,3,4],rotation_xyzw=q)
    match_twice(state,now,map_from_odom=transform)
    before=state._position_now();epoch=state.odom_epoch
    expected_local=state.local_map_pose()
    anchored=state.anchor_selected_map()
    after=state._position_now()
    assert anchored['id']==entry['id'] and state.odom_epoch==epoch
    assert state.local_map_pose()==expected_local
    assert after['source']=='map_lio' and distance(before['wgs84'],after['wgs84'])<.001
    assert state.initial_map_pose()['position_m']==pytest.approx(expected_local['position_m'],abs=.004)
    assert state.initial_map_pose()['quaternion_xyzw']==pytest.approx(expected_local['quaternion_xyzw'],abs=.001)
    assert state.snapshot()['geographic']['maps'][-1]['georeferenced']
    restored=OutdoorState(tmp_path,clock=lambda:now[0])
    assert restored.selected_map['reference_source']=='gnss_lio'
    assert restored.navigation_snapshot()['valid'] is False
    with pytest.raises(ValueError,match='已有地理参考'):state.anchor_selected_map()


def test_anchor_requires_both_observations_and_verified_calibration(tmp_path):
    now=[100.];state,_=trained(tmp_path,now)
    cloud=tmp_path/'local.pcd';cloud.write_bytes(b'local')
    entry=state.register_capture(dict(map=str(cloud)),None);state.select_geo_map(entry['id'])
    with pytest.raises(ValueError):state.anchor_selected_map()
    match_twice(state,now)
    state.calibration=dict(CAL,time_verified=False)
    with pytest.raises(ValueError):state.anchor_selected_map()
    assert state.maps.get(entry['id'])['reference_source']=='local_only'
    state.calibration=CAL;now[0]+=3;state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    state.feed_map_match(quality(state,now,3))
    with pytest.raises(ValueError):state.anchor_selected_map()


def test_same_route_survives_gnss_map_handover_and_signal_loss(tmp_path):
    from test_outdoor import route
    now=[100.];state,geo=trained(tmp_path,now);state.select_geo_map(geo['id'])
    state.set_route(route());p=state._position_now();assert p['source']=='gnss_lio'
    state.update_conversion(p,[116.3,39.9]);state.control('start')
    session,route_id=state.session_id,state.route['id']
    match_twice(state,now,tracking_timeout_s=5.)
    assert state._position_now()['source']=='map_lio'
    # After a long gap, re-confirm the map without changing the route/session.
    now[0]+=11;state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert not state.feed_map_match(quality(state,now,3,tracking_timeout_s=5.))
    assert state.feed_map_match(quality(state,now,4,tracking_timeout_s=5.))
    assert state._position_now()['source']=='map_lio'
    assert state.guiding and state.session_id==session and state.route['id']==route_id
    # Both absolute sources expire: pause progress, keep the destination/session.
    now[0]+=3;state.feed_odom(now[0],[24,0,0],[0,0,0,1])
    assert state.snapshot()['guidance'] is None and not state.navigation_snapshot()['valid']
    state.feed_map_match(quality(state,now,5,tracking_timeout_s=5.))
    assert state.navigation_snapshot()['valid'] and state.guiding
    # Fresh consistent GNSS/LIO can take over when the map observation drops.
    from test_outdoor import fix
    a=state.alignment
    p=state.origin.wgs84(math.cos(a['yaw'])*24+a['tx'],math.sin(a['yaw'])*24+a['ty'])
    now[0]+=.05;state.feed_odom(now[0]+.04,[24,0,0],[0,0,0,1])
    state.feed_fix(fix(now[0],lon=p[0],lat=p[1]))
    state.feed_map_match(dict(quality(state,now,6),valid=False))
    assert state._position_now()['source']=='gnss_lio'
    assert state.session_id==session and state.route['id']==route_id


def test_local_navigation_and_road_navigation_cannot_compete(tmp_path):
    now=[100.];state,geo=restarted(tmp_path,now);match_twice(state,now)
    state.local_navigation_active=True
    with pytest.raises(ValueError,match='当前地图导航'):state.control('start')
    with pytest.raises(ValueError):state.simulate('enter')
    with pytest.raises(ValueError):state.select_geo_map(None)
    with pytest.raises(ValueError):state.request_localization('stop')
    assert state.selected_map['id']==geo['id']

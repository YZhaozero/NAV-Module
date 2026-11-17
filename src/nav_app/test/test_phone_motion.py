"""Real consumers with synthetic faults; this does not claim field accuracy."""
import copy
import math
import pytest
from nav_app.phone_observation import motion_observation,PhoneFallback
from nav_app.outdoor import OutdoorState,GeoOrigin,distance
from test_navigation_trial import aligned_trial


def motion(stamp=140.,**updates):
    value=dict(schema_version=1,source='phone_continuity',coordinate_frame='phone_gnss_enu',
        status='observed',fresh=True,utc_ms=stamp*1000.,source_id='phone-00001',session_id='session-00001',
        sequence=int(stamp),continuity_id='arc-1',origin_revision=0,native_quality=2,
        local_enu_m=[0.,0.,0.],delta_covariance_enu_m2=[[.02,0.,0.],[0.,.02,0.],[0.,0.,.03]],delta_dt_s=1.,delta_observed=True,gap_seconds=0.,
        latitude=40.,longitude=116.,altitude_ellipsoid_m=50.,horizontal_sigma_m=.4,
        position_observed=True,raw_motion_observed=True,coarse_origin_update=False,
        native_error_reinitialized=False,absolute_accuracy_verified=False)
    value.update(updates);return value


@pytest.mark.parametrize('field,value',[
    ('status','predicted'),('position_observed',None),('raw_motion_observed',False),
    ('coarse_origin_update',True),('native_error_reinitialized',True),('quality',1),
    ('native_quality',True),('fresh',False),('horizontal_sigma_m',0),('utc_ms',110000),
    ('delta_covariance_enu_m2',[[9.,0.,0.],[0.,9.,0.],[0.,0.,9.]]),('delta_dt_s',23.),
    ('delta_observed',False),('delta_observed',None),('gap_seconds',.001),('gap_seconds',None),
    ('absolute_accuracy_verified',True)])
def test_prediction_or_origin_update_cannot_initialize_geography(field,value):
    assert motion_observation(motion(**{field:value}),140.1) is None


def test_candidate_never_promoted_to_native_fixed_and_receiver_keeps_priority():
    row=motion_observation(motion(native_quality=1),140.1)
    assert 'fix_quality' not in row and row['native_fix_quality']==4
    fallback=PhoneFallback()
    fallback.receiver(dict(stamp=140.,received_at=140.,timestamp_source='receiver_utc',
        longitude=116.,latitude=40.,fix_quality=4,horizontal_std_m=.1),140.1)
    assert fallback.motion(motion(),140.1) is None
    assert fallback.motion(motion(142.),142.1)
    assert fallback.motion(motion(142.),142.2) is None


def feed_motion(state,now,world,i,**updates):
    stamp=100.+i;now[0]=stamp+.05
    for dt in (-.04,.04):state.feed_odom(stamp+dt,[i+dt,0,0],[0,0,0,1])
    lon,lat=world.wgs84(15.,25.+i)
    row=motion(stamp,longitude=lon,latitude=lat,local_enu_m=[0.,float(i),0.],**updates)
    state.feed_phone_motion(row)
    obs=motion_observation(row,now[0])
    if obs:state.feed_fix(obs)


def test_selected_candidate_runs_actual_alignment_and_livo_survives_phone_outage():
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state)
    state.set_phone_motion(True)
    world=GeoOrigin(116.,40.,50.)
    for i in range(40):feed_motion(state,now,world,i)
    assert state.alignment and state.alignment['estimate_source']=='phone_continuity'
    assert state.alignment['experimental'] and state.position['quality'] is None
    assert state.alignment['geographic_accuracy']=='approximate'
    assert state._position_now()['precision_source']=='phone_continuity_trajectory'
    assert state.capture_reference('camera_init') is None
    original=copy.deepcopy(state.alignment)
    # Thirty metres of new LiVO motion, no phone measurements. No synthetic RTK.
    for i in range(40,70):
        now[0]=100+i+.05
        for dt in (-.04,.04):state.feed_odom(100+i+dt,[i+dt,0,0],[0,0,0,1])
        state.sample_actual_track()
    position=state._position_now()
    assert position and position['coasting'] and position['correction_age_s']>30
    assert state.alignment==original
    assert distance(position['wgs84'],world.wgs84(15.,94.04))<.05
    state.feed_odom_instance('livo-one');state.feed_odom_instance('livo-two')
    assert state._position_now() is None and state.alignment is None


def test_origin_revision_cannot_refresh_old_anchor_or_fake_movement():
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    world=GeoOrigin(116.,40.,50.)
    for i in range(40):feed_motion(state,now,world,i)
    anchor=copy.deepcopy(state.alignment)
    feed_motion(state,now,world,40,coarse_origin_update=True,origin_revision=1)
    assert state.alignment==anchor
    assert not state.trial.pairs
    assert state._position_now() and state._position_now()['source']=='gnss_lio'
    feed_motion(state,now,world,41,origin_revision=1)
    assert len(state.trial.pairs)==1
    assert state.alignment['stamp']==anchor['stamp']
    assert not state.phone_motion_snapshot()['absolute_accuracy_verified']


def test_bad_livo_and_source_switch_never_reuse_unverified_transform():
    state,now,world=aligned_trial()
    anchor=copy.deepcopy(state.alignment)
    state.set_phone_motion(True)
    feed_motion(state,now,world,40,native_quality=1)
    assert state.alignment==anchor  # New source needs a complete fit, not one agreeing fix.
    now[0]=141.05
    state.feed_odom(141.,[200,200,0],[0,0,0,1])
    assert state.alignment is None
    assert state._position_now() is None  # A fused estimate cannot stand in for native RTK.


def test_motion_setting_persists_without_persisting_live_transform(tmp_path):
    state=OutdoorState(tmp_path,clock=lambda:100.)
    assert not state.phone_motion_enabled
    state.set_phone_motion(True)
    restored=OutdoorState(tmp_path,clock=lambda:100.)
    assert restored.phone_motion_enabled and restored.alignment is None
    restored.guiding=True
    with pytest.raises(ValueError,match='先结束引导'):restored.set_phone_motion(False)


def test_relative_motion_does_not_require_absolute_float_at_every_epoch():
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    world=GeoOrigin(116.,40.,50.)
    for i in range(10):feed_motion(state,now,world,i,native_quality=4,position_observed=False)
    assert state.origin is None and state.alignment is None
    feed_motion(state,now,world,10,native_quality=2,horizontal_sigma_m=25.)
    for i in range(11,60):feed_motion(state,now,world,i,native_quality=4,position_observed=False,horizontal_sigma_m=25.)
    assert state.alignment and state.alignment['geographic_accuracy']=='approximate'
    assert state.alignment['absolute_anchor_stamp']==110.
    assert state.alignment['absolute_horizontal_sigma_m']==25.
    pose=state._position_now()
    assert pose and pose['coasting'] and pose['correction_age_s']>49.


@pytest.mark.parametrize('quality',[1,2,5])
def test_weak_receiver_cannot_interleave_and_reset_selected_phone_fit(quality):
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    fallback=PhoneFallback();world=GeoOrigin(116.,40.,50.)
    for i in range(40):
        stamp=100.+i;now[0]=stamp+.05
        for dt in (-.04,.04):state.feed_odom(stamp+dt,[i+dt,0,0],[0,0,0,1])
        lon,lat=world.wgs84(15.,25.+i)
        row=motion(stamp,longitude=lon,latitude=lat,local_enu_m=[0.,float(i),0.])
        state.feed_phone_motion(row)
        data=fallback.motion(row,now[0])
        if data:state.feed_fix(data)
        weak=dict(schema=1,source='receiver',fix_quality=quality,longitude=lon,latitude=lat,
            stamp=stamp,received_at=now[0],timestamp_source='receiver_utc',horizontal_std_m=8.)
        assert not fallback.receiver(weak,now[0])
        if fallback.receiver(weak,now[0]):state.feed_fix(weak)
    assert fallback.phone_stamp is None and fallback.motion_stamp==139.
    assert state.alignment and state.gnss_source[0]=='phone_continuity'
    assert state.alignment['estimate_source']=='phone_continuity'
    assert fallback.receiver(weak,150.)  # A disappeared phone does not hold priority forever.


def test_observed_endpoint_after_unobserved_edge_cannot_join_old_fit():
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    world=GeoOrigin(116.,40.,50.)
    for i in range(40):feed_motion(state,now,world,i)
    anchor=copy.deepcopy(state.alignment)
    feed_motion(state,now,world,40,delta_observed=False,gap_seconds=.8)
    assert not state.trial.pairs and state.alignment==anchor
    feed_motion(state,now,world,41)
    assert len(state.trial.pairs)==1 and state.alignment==anchor

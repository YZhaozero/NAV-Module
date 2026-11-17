import copy
import math
import pytest
from nav_app.phone_heading import RawMotionHeading,constraints
from nav_app.phone_observation import motion_observation
from nav_app.phone_observation import scalar_observation
from nav_app.outdoor import OutdoorState,GeoOrigin
from test_phone_motion import motion
from nav_app.navigation_trial import LocalMapProjection,transform


def raw(stamp,yaw=.7,dt=1.,dx=.8,clock=100.,**updates):
    A=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    value=dict(schema_version=1,coordinate_frame='phone_gnss_enu',
        start_utc_ms=(stamp-dt)*1000,end_utc_ms=stamp*1000,dt_s=dt,
        phase=dict(start_utc_ms=(stamp-dt)*1000,end_utc_ms=stamp*1000,A_enu=A,
            b_m=[clock+math.cos(yaw)*dx,clock+math.sin(yaw)*dx,clock],sigma_m=[.05]*3))
    value.update(updates);return value


def test_heading_requires_future_independent_edges_and_eliminates_clock():
    fit=RawMotionHeading();answers=[]
    for i in range(1,26):
        result=fit.push(raw(100+i,clock=1000*math.sin(i)),lambda t:[.8*(t-100),0,0])
        if i<=8:assert result is None
        if result:answers.append(result)
    assert answers and answers[0]['heading_validated_at']>=114.
    assert answers[0]['yaw']==pytest.approx(.7,abs=.001)
    assert answers[0]['heading_train_rows']>=16 and answers[0]['heading_validation_rows']>=12
    assert answers[0]['validation_yaw_error_rad']<.001
    assert answers[0]['validation_start_utc_s']==108.
    assert answers[0]['validation_end_utc_s']==114.
    assert answers[0]['validation_edge_count']==6
    assert answers[0]['validation_motion_baseline_m']==pytest.approx(4.8)
    assert answers[0]['validation_peak_gap']>6
    assert 0<answers[0]['validation_yaw_sigma_rad']<math.radians(15)


def test_heading_rejects_future_disagreement_missing_livo_and_ambiguous_sky():
    fit=RawMotionHeading()
    for i in range(1,15):
        result=fit.push(raw(100+i,yaw=.7 if i<=8 else -1.),lambda t:[.8*(t-100),0,0])
        assert result is None
    assert fit.result is None
    empty=RawMotionHeading()
    for i in range(1,30):assert empty.push(raw(100+i),lambda t:None) is None
    ambiguous=RawMotionHeading()
    for i in range(1,30):
        value=raw(100+i);value['phase']['A_enu']=[[1.,0,0]]*3
        value['phase']['b_m']=[100.]*3
        assert ambiguous.push(value,lambda t:[.8*(t-100),0,0]) is None


def test_profile_rejects_precise_wrong_candidate_inside_old_ten_degree_gate_then_relearns():
    fit=RawMotionHeading();truth=.7;wrong=truth+math.radians(9.)
    fit.candidate=dict(yaw=wrong,sigma=math.radians(8),rows=100,train_end=100.)
    answers=[]
    for i in range(1,31):
        answer=fit.push(raw(100+i,yaw=truth),lambda t:[.8*(t-100),0.,0.])
        if i==6:
            evidence=fit.last_validation
            assert evidence and not evidence['accepted']
            assert evidence['profile_delta']>fit.PROFILE_LIMIT
            assert abs(evidence['candidate_yaw']-evidence['optimal_yaw'])<math.radians(10)
        if answer:answers.append(answer)
    assert answers and answers[0]['yaw']==pytest.approx(truth,abs=.001)
    assert answers[0]['heading_train_end_utc_s']>=108.
    assert answers[0]['validation_start_utc_s']>=answers[0]['heading_train_end_utc_s']
    assert answers[0]['validation_profile_delta']<=fit.PROFILE_LIMIT
    assert answers[0]['heading_validated_at']>=114.


def test_weak_short_motion_and_overlapping_phase_cannot_validate_fixed_candidate():
    fit=RawMotionHeading();fit.candidate=dict(yaw=.7,sigma=.1,rows=100,train_end=100.)
    for i in range(1,10):
        assert fit.push(raw(100+i,dx=.11),lambda t:[.11*(t-100),0.,0.]) is None
    assert fit.result is None and fit.last_validation is None
    overlap=RawMotionHeading();overlap.candidate=dict(yaw=.7,sigma=.1,rows=100,train_end=100.)
    assert overlap.push(raw(100.5),lambda t:[.8*(t-100),0.,0.]) is None
    assert overlap.check==[]


def test_phase_uses_its_own_epoch_and_bad_interval_is_not_interpolated():
    value=raw(101.,dt=.5,dx=.4)
    value['phase']['start_utc_ms']=100000.
    assert constraints(value)['phase']['start']==100.
    value['phase']['start_utc_ms']=98000.
    assert constraints(value) is None


def test_partial_raw_scalar_support_reaches_heading_without_becoming_measured_3d_motion():
    row=motion(101.,status='predicted',raw_motion_observed=False,delta_observed=False,
        raw_scalar_observed=True,motion_support='partial',raw_constraints=raw(101.),
        position_observed=False,local_enu_m=[100.,-100.,-56.])
    observation=motion_observation(row,101.2)
    assert observation and observation['covariance_source']=='raw_scalar_heading_only'
    assert observation['raw_constraints']==row['raw_constraints']
    assert 'relative_position_m' not in observation and not observation['absolute_position_observed']
    assert observation['native_fix_quality']==5
    state=OutdoorState(clock=lambda:101.2);state.feed_phone_motion(row)
    snapshot=state.phone_motion_snapshot()
    assert snapshot['status']=='predicted' and snapshot['raw_scalar_observed'] is True
    assert snapshot['motion_support']=='partial'
    assert snapshot['reason']=='手机运动观测不完整，结合雷达核对方向'
    # The same predicted integrated point is not a fallback when scalars stop.
    row.pop('raw_constraints')
    assert motion_observation(row,101.2) is None


@pytest.mark.parametrize('flag',[False,0,1,None,'true'])
def test_explicit_scalar_support_cannot_fall_back_to_legacy_scalar_semantics(flag):
    row=motion(101.,raw_scalar_observed=flag,raw_constraints=raw(101.))
    assert scalar_observation(row,101.2) is None


def test_new_scalar_support_retains_timing_frame_and_genuine_constraint_checks():
    row=motion(101.,status='predicted',raw_motion_observed=False,delta_observed=False,
        raw_scalar_observed=True,raw_constraints=raw(101.))
    assert scalar_observation(row,111.01) is None
    assert scalar_observation(dict(row,coordinate_frame='unknown'),101.2) is None
    assert scalar_observation(dict(row,raw_constraints={}),101.2) is None
    assert scalar_observation(dict(row,status='unavailable'),101.2) is None
    legacy=motion(101.,raw_constraints=raw(101.))
    assert scalar_observation(legacy,101.2) is not None
    assert scalar_observation(dict(legacy,status='predicted'),101.2) is None


def test_absent_scalar_support_does_not_discard_separately_measured_full_3d_delta():
    row=motion(101.,raw_scalar_observed=False)
    observation=motion_observation(row,101.2)
    assert observation and observation['covariance_source']=='phone_raw_motion_step'
    assert 'relative_position_m' in observation and 'raw_constraints' not in observation
    assert motion_observation(dict(row,delta_observed=False),101.2) is None


def test_mirror_heading_is_not_mistaken_for_unique_between_grid_angles():
    fit=RawMotionHeading();yaw=math.radians(47.5)
    for i in range(1,40):
        value=raw(100+i,yaw=yaw,dx=2.)
        value['phase'].update(A_enu=[[1.,0.,0.],[-1.,0.,0.]],
            b_m=[2*math.cos(yaw),-2*math.cos(yaw)],sigma_m=[.05,.05])
        assert fit.push(value,lambda t:[2*(t-100),0.,0.]) is None
    assert fit.result is None and fit.candidate is None
    value=raw(101.);value['end_utc_ms']=102000.
    assert constraints(value) is None


@pytest.mark.parametrize('partial',[False,True])
def test_actual_navigation_initializes_from_scalars_without_integrated_phone_path(partial):
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    world=GeoOrigin(116.,40.,50.)
    for i in range(35):
        stamp=100.+i;now[0]=stamp+.05
        for dt in (-.04,.04):state.feed_odom(stamp+dt,[.8*(i+dt),0,0],[0,0,0,1])
        lon,lat=world.wgs84(15.+math.cos(.7)*.8*i,25.+math.sin(.7)*.8*i)
        row=motion(stamp,longitude=lon,latitude=lat,horizontal_sigma_m=25.,
            local_enu_m=[float(i*i),float(-i*i),0.],delta_observed=False,gap_seconds=1.,
            raw_constraints=raw(stamp),native_quality=2 if i==0 else 4,position_observed=i==0)
        if partial:row.update(raw_scalar_observed=True,raw_motion_observed=False,
            status='predicted',motion_support='partial')
        state.feed_phone_motion(row)
        data=motion_observation(row,now[0]);assert data is not None
        state.feed_fix(data)
        state.trial.poll_phone_heading(state,wait=True)
    a=state.alignment
    assert a and a['heading_method']=='raw_scalar_livo_increment'
    assert a['yaw']==pytest.approx(.7,abs=.001)
    assert a['absolute_horizontal_sigma_m']==25.
    assert not a['absolute_accuracy_verified'] and state.capture_reference('camera_init') is None
    # Explicitly synthetic local map projection and preselected test route.
    # This checks guidance integration, not a real Baidu service response.
    state.trial.projection=LocalMapProjection(state.origin.values,
        [state.origin.wgs84(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))])
    path=[[.8*34,0.],[.8*34+25,0.],[.8*34+45,0.]]
    route=[state.trial.projection.to_bd09(transform(a,p)) for p in path]
    state.set_route(dict(points=route,source='manual',coordinate_system='BD09',title='离线合成测试路线'))
    pose=state._position_now();state.update_conversion(pose,route[0]);state.control('start')
    assert state.guiding
    old=copy.deepcopy(a)
    # A prediction without observed raw scalars cannot move an existing map anchor.
    now[0]=135.05
    jump_lon,jump_lat=world.wgs84(15.+math.cos(.7)*28+40,25.+math.sin(.7)*28)
    bad=motion(135.,status='predicted',raw_motion_observed=False,origin_revision=1,
        longitude=jump_lon,latitude=jump_lat)
    state.feed_phone_motion(bad)
    assert motion_observation(bad,now[0]) is None and state.alignment==old
    # No phone fix for ten seconds: actual LiVO moves 8m, while an unobserved
    # phone origin reports an artificial 40m jump. It cannot arrive for us.
    for i in range(35,45):
        now[0]=100+i+.05
        for dt in (-.04,.04):state.feed_odom(100+i+dt,[.8*(i+dt),0,0],[0,0,0,1])
        state.sample_actual_track()
        pose=state._position_now();assert pose and pose['coasting']
        state.update_conversion(pose,state.trial.projection.to_bd09(transform(old,[.8*(i+.04),0])))
    guide=state.snapshot()['guidance']
    assert guide['remaining_m']>35 and not guide['near_destination'] and state.guiding
    now[0]=145.05
    for dt in (-.04,.04):state.feed_odom(145+dt,[.8*(45+dt),0,0],[0,0,0,1])
    changed=motion(145.,source_id='different-phone',raw_constraints=raw(145.),
        longitude=jump_lon,latitude=jump_lat,horizontal_sigma_m=.1)
    state.feed_phone_motion(changed);state.feed_fix(motion_observation(changed,now[0]))
    state.trial.poll_phone_heading(state,wait=True)
    assert state.alignment==old and state.guiding
    assert state.trial.phone_heading.accepted_edges<=1
    state.feed_odom_instance('first');state.feed_odom_instance('restart')
    assert state.alignment is None and not state.trial.phone_heading.train
    state.trial.phone_heading_worker.close()


def test_late_background_heading_cannot_survive_source_generation_change():
    import threading
    from nav_app.phone_heading import HeadingWorker
    now=[100.];state=OutdoorState(clock=lambda:now[0]);state.trial.start(state);state.set_phone_motion(True)
    started=threading.Event();release=threading.Event()
    class Slow:
        def push(self,*args):
            started.set();assert release.wait(5)
            return dict(yaw=.7)
    worker=HeadingWorker();state.trial.phone_heading_worker=worker
    context=dict(tag=state.trial._heading_tag(state),data=dict(stamp=100.))
    worker.submit(Slow(),{}, {},context)
    assert started.wait(5)
    state.set_phone_motion(False)
    release.set();state.trial.poll_phone_heading(state,wait=True)
    assert state.alignment is None
    worker.close()


def test_heading_worker_bounds_queue_to_latest_request():
    import threading
    from nav_app.phone_heading import HeadingWorker
    started=threading.Event();release=threading.Event();seen=[]
    class Slow:
        def push(self,value,*args):
            seen.append(value)
            if value==1:started.set();assert release.wait(5)
            return None
    worker=HeadingWorker();estimator=Slow()
    worker.submit(estimator,1,{},{});assert started.wait(5)
    worker.submit(estimator,2,{},{});worker.submit(estimator,3,{},{})
    release.set();worker.poll(wait=True);worker.poll(wait=True)
    assert seen==[1,3]
    worker.close()

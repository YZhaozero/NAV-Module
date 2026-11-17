"""Synthetic independent absolute anchors; never field-accuracy evidence."""
import copy
import math
import pytest
from nav_app.phone_translation import anchor
from nav_app.outdoor import OutdoorState,GeoOrigin,distance
from nav_app.navigation_trial import LocalMapProjection
from test_phone_motion import motion


def fixture():
    now=[104.05];s=OutdoorState(clock=lambda:now[0]);s.trial.start(s);s.set_phone_motion(True)
    s.origin=GeoOrigin(116.,40.,50.)
    for i in range(41):s.feed_odom(100.+i*.1,[float(i)*.1,0.,0.],[0.,0.,0.,1.])
    s.gnss_source=('phone_continuity','phone-00001','session-00001','arc-1',None)
    s.alignment=dict(yaw=0.,tx=0.,ty=0.,stamp=103.,absolute_anchor_stamp=103.,
        estimate_source='phone_continuity',heading_method='raw_scalar_livo_increment',
        geographic_accuracy='approximate',absolute_horizontal_sigma_m=25.,experimental=True)
    s._anchor_odometry()
    return s,now


def row(s,stamp=104.,revision=1,shift=3.,**updates):
    p=s._antenna_at(stamp,{'antenna_in_body_m':[0.,0.,0.]})
    lon,lat=s.origin.wgs84(p[0]+shift,p[1])
    value=dict(schema_version=1,status='consistent',source='system_gps_window',revision=revision,
        utc_ms=stamp*1000.,measurement_utc_ms=(stamp-.5)*1000.,longitude=lon,latitude=lat,
        altitude_ellipsoid_m=50.,horizontal_sigma_m=4.,absolute_accuracy_verified=False)
    value.update(updates)
    return motion(stamp,raw_constraints={'unused_in_this_boundary_test':True},absolute_anchor=value)


def tick(s,now,n,position=None):
    for i in range(n):
        t=s.odom[-1][0]+.1;now[0]=t+.01
        s.feed_odom(t,list(position if position is not None else s.odom[-1][1]),[0.,0.,0.,1.])


def feed_synthetic_reference(s,r):
    # The full independent solver is exercised separately. These references
    # are constructed ground truth solely for translation lifecycle tests.
    s.feed_phone_motion(r)
    value=anchor(r,s.clock())
    p=s._antenna_at(value['stamp'],{'antenna_in_body_m':[0.,0.,0.]})
    return s.trial.phone_translation.accept_reference(s,value,p)


@pytest.mark.parametrize('updates',[
    {'status':'validated'},{'revision':True},{'revision':0},{'utc_ms':103000.},
    {'measurement_utc_ms':100000.},{'measurement_utc_ms':105000.},
    {'absolute_accuracy_verified':True},{'horizontal_sigma_m':0.},
    {'source':'native_fixed_window','native_quality':2},
    {'source':'native_fixed_window','native_quality':True},
    {'revision':2**53},{'quality':4},{'fix_quality':4},
    {'source_id':'different'},{'session_id':'different'},
    {'altitude_ellipsoid_m':100001.},{'altitude_ellipsoid_m':-15001.},
])
def test_bad_optional_anchor_is_ignored(updates):
    s,now=fixture();r=row(s,**updates)
    assert anchor(r,now[0]) is None
    s.feed_phone_motion(r)
    assert s.phone_motion is not None and s.trial.phone_translation.target is None


def test_old_wire_cannot_move_translation_and_does_not_create_anchor_metadata():
    s,now=fixture();before=copy.deepcopy(s.alignment)
    s.feed_phone_motion(motion(104.,raw_constraints={'present':True},origin_revision=55))
    tick(s,now,100)
    assert s.alignment==before


def test_revision_target_slews_on_actual_ticks_without_fake_distance_or_yaw():
    s,now=fixture();start=s.odom_distance;yaw=s.alignment['yaw']
    feed_synthetic_reference(s,row(s,shift=10.8))
    assert s.trial.phone_translation.target and s.alignment['tx']==0.
    tick(s,now,100)
    assert s.alignment['tx']==pytest.approx(3.,abs=1e-4)
    assert s.alignment['yaw']==yaw and s.odom_distance==start
    assert s.alignment['absolute_horizontal_sigma_m']==25.  # no premature accuracy upgrade
    assert s.alignment['geographic_correction_remaining_m']==pytest.approx(7.8,abs=.001)
    assert s.alignment['geographic_correction_total_m']==pytest.approx(3.,abs=.001)
    assert s._position_now()['geographic_correction_total_m']==pytest.approx(3.,abs=.001)


def test_delayed_position_uses_historical_livo_and_revisions_are_monotonic():
    s,now=fixture();r=row(s,stamp=101.,shift=3.)
    feed_synthetic_reference(s,r)
    tick(s,now,110)
    assert s.alignment['tx']==pytest.approx(3.,abs=1e-4)  # binding to current p=4 would give 0m
    assert s.alignment['absolute_anchor_stamp']==101.
    assert s.alignment['absolute_horizontal_sigma_m']==4.
    again=row(s,stamp=s.odom[-1][0],shift=30.)
    feed_synthetic_reference(s,again)
    assert s.trial.phone_translation.target['stamp']==101.


def test_source_generation_reset_discards_pending_correction():
    s,now=fixture();feed_synthetic_reference(s,row(s,shift=10.8));tick(s,now,10)
    before=s.alignment['tx'];s.phone_alignment_generation+=1
    tick(s,now,20)
    assert s.alignment['tx']==before and s.trial.phone_translation.target is None
    assert s.trial.phone_translation.total_m==0.
    assert s.trial.phone_translation.last_correction_m==0.
    assert 'geographic_correction_total_m' not in s.alignment
    s.feed_odom_instance('one');s.feed_odom_instance('two')
    assert s.alignment is None


@pytest.mark.parametrize('reset',['toggle','tracking'])
def test_correction_totals_reset_and_fresh_poses_confirm_arrival(reset):
    s,now=fixture();feed_synthetic_reference(s,row(s,shift=10.8));tick(s,now,10)
    translation=s.trial.phone_translation
    assert translation.total_m>0 and not translation.arrival_ready()
    if reset=='toggle':s.trial.set_rtk(s,False)
    else:s.trial.reset_tracking(s)
    assert translation.target is None and translation.tag is None
    assert translation.total_m==0 and translation.last_correction_m==0
    assert translation.last_revision==0 and translation.last_tick is None
    assert 'geographic_correction_total_m' not in s.alignment
    assert not translation.arrival_ready()
    translation.tick(s);translation.tick(s);translation.tick(s)
    assert translation.arrival_confirmations==1  # one timestamp, not three UI refreshes
    tick(s,now,3)
    assert translation.arrival_ready()


def test_unavailable_motion_cannot_supply_optional_anchor():
    s,now=fixture();r=row(s);r['status']='unavailable'
    assert anchor(r,now[0]) is None


def test_corrected_route_reprojects_without_fake_arrival_or_permanent_cap():
    s,now=fixture();s.trial.projection=LocalMapProjection(s.origin.values,
        [s.origin.wgs84(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))])
    projection=s.trial.projection
    points=[projection.to_bd09([4.,0.]),projection.to_bd09([16.,0.])]
    s.set_route(dict(points=points,source='manual',coordinate_system='BD09',title='合成平移边界测试'))
    pose=s._position_now();s.update_conversion(pose,points[0]);s.control('start')
    before=s.odom_distance;feed_synthetic_reference(s,row(s,shift=10.8));tick(s,now,200)
    pose=s._position_now();xy=s.origin.enu(*pose['wgs84'],50.)[:2]
    guide=s._guidance(projection.to_bd09(xy))
    assert s.route_progress>5. and not guide['near_destination'] and guide['uncertain']
    tick(s,now,180)
    pose=s._position_now();xy=s.origin.enu(*pose['wgs84'],50.)[:2]
    guide=s._guidance(projection.to_bd09(xy))
    assert guide['near_destination'] and s.route_progress>10.
    assert s.odom_distance==before  # corrected location is not a walked distance


def test_new_heading_keeps_independent_origin_and_only_rotates_about_endpoint():
    s,now=fixture();feed_synthetic_reference(s,row(s,shift=3.));tick(s,now,110)
    old=copy.deepcopy(s.alignment);p=list(s.odom[-1][1])
    context=dict(data={'stamp':s.odom[-1][0]},antenna=p,calibration={},
        anchors=[(p,[100.,100.,0.],103.)],anchor_stamp=103.,anchor_sigma=25.)
    s.trial._apply_scalar_result(s,context,dict(yaw=.01,heading_method='raw_scalar_livo_increment'))
    a=s.alignment
    def out(a):return [math.cos(a['yaw'])*p[0]-math.sin(a['yaw'])*p[1]+a['tx'],
                       math.sin(a['yaw'])*p[0]+math.cos(a['yaw'])*p[1]+a['ty']]
    assert math.dist(out(a),out(old))<1e-9
    assert a['absolute_anchor_source']=='system_gps_window' and a['absolute_horizontal_sigma_m']==4.


@pytest.mark.parametrize('refresh_each_tick',[True,False])
def test_verified_backward_origin_correction_reprojects_route_without_permanent_progress_lock(refresh_each_tick):
    s,now=fixture();s.alignment['tx']=96.
    s.trial.projection=LocalMapProjection(s.origin.values,
        [s.origin.wgs84(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))])
    projection=s.trial.projection
    s.set_route(dict(points=[projection.to_bd09([0.,0.]),projection.to_bd09([200.,0.])],
        source='manual',coordinate_system='BD09',title='合成反向位置修正'))
    def guide():
        p=s._position_now();xy=s.origin.enu(*p['wgs84'],50.)[:2]
        return s._guidance(projection.to_bd09(xy))
    guide();s.route_progress=100.
    before=s.odom_distance;feed_synthetic_reference(s,row(s,shift=66.))
    for i in range(1100):
        tick(s,now,1)
        if refresh_each_tick:guide()
    result=guide()
    assert s.route_progress==pytest.approx(70.,abs=.3)
    assert result['off_route_m']<.1 and not result.get('uncertain')
    assert s.odom_distance==before


def test_toggle_clears_correction_identity_and_new_verified_correction_survives_ui_pause():
    s,now=fixture();feed_synthetic_reference(s,row(s,shift=3));tick(s,now,110);s._position_now()
    s.trial.set_rtk(s,False);tick(s,now,3)
    assert s._position_now()['geographic_correction_identity'] is None
    s.trial.set_rtk(s,True)
    feed_synthetic_reference(s,row(s,stamp=s.odom[-1][0],revision=2,shift=33))
    tick(s,now,1100)
    assert s._position_now() is not None and not s.position_conflict


def test_closed_route_correction_does_not_resolve_ambiguous_arrival_on_repaint():
    s,now=fixture();s.alignment['tx']=26.
    s.trial.projection=LocalMapProjection(s.origin.values,
        [s.origin.wgs84(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))])
    project=s.trial.projection
    s.set_route(dict(points=[project.to_bd09(p) for p in ((0,0),(20,0),(20,20),(0,0))],
        source='manual',coordinate_system='BD09',title='合成闭环路口'))
    s.route_progress=40.;feed_synthetic_reference(s,row(s,shift=-4));tick(s,now,1100)
    point=project.to_bd09([0,0]);physical=s.odom_distance
    for _ in range(5):
        g=s._guidance(point)
        assert g['uncertain'] and not g['near_destination']
    assert s.route_progress==40 and s.odom_distance==physical
    # Real movement away from the shared start/end on a unique branch resolves it.
    for i in range(1,42):tick(s,now,1,[4+i*.1,0,0])
    g=s._guidance(project.to_bd09([4.1,0]))
    assert not g.get('uncertain') and s.route_progress<10


@pytest.mark.parametrize('source',['system_gps_window','native_fixed_window','raw_window'])
def test_wire_consistency_or_fixed_label_alone_cannot_push_position(source):
    s,now=fixture();before=copy.deepcopy(s.alignment)
    s.feed_phone_motion(row(s,source=source,native_quality=1,shift=10.8))
    tick(s,now,100)
    assert s.trial.phone_translation.target is None and s.alignment==before

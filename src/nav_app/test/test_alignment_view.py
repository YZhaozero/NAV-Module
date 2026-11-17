"""Quality policy, independent trajectories, and advisory preview regressions."""
import copy
import math
import pytest
from nav_app.outdoor import OutdoorState
from nav_app.alignment_view import preview_fit
from nav_app.navigation_trial import transform
from test_navigation_trial import make_state,feed,aligned_trial,projection


@pytest.mark.parametrize('quality',[1,2,5])
def test_fixed_only_policy_never_stops_recording_or_discards_raw_tracks(quality):
    state,now,world=make_state()
    state.trial.set_quality_policy(state,'fixed')
    for i in range(40):feed(state,now,world,i,quality=quality)
    assert state.trial.active and not state.requests
    assert not state.trial.pairs and state.alignment is None
    assert len(state.track)>30 and len(state.local_track)>30
    view=state.trial.visual_snapshot(state)
    assert view['separate_axes'] and not view['aligned']
    assert not any(r['selected'] for r in view['gnss'])
    assert len(view['lidar'])>30
    assert '等待固定解' in state.trial.reason


def test_mixed_policy_uses_float_but_preview_never_unlocks_guidance():
    state,now,world=make_state()
    state.trial.set_quality_policy(state,'fixed_float')
    for i in range(7):feed(state,now,world,i,quality=5)
    view=state.trial.visual_snapshot(state)
    assert view['preview'] and not view['aligned'] and state.alignment is None
    assert view['transform']['residual_m']<.01
    with pytest.raises(ValueError):state.control('start')
    for i in range(7,40):feed(state,now,world,i,quality=5)
    assert state.trial.snapshot(state)['aligned']
    before=copy.deepcopy(state.alignment)
    for i in range(40,45):feed(state,now,world,i,quality=2,error=(5,0))
    assert state.alignment==before
    assert state._position_now()['source']=='gnss_lio'


def test_quality_switch_preserves_tracks_and_discards_unqualified_candidate(tmp_path):
    state,now,world=make_state(tmp_path)
    state.trial.set_quality_policy(state,'fixed_float')
    for i in range(40):feed(state,now,world,i,quality=5)
    before=(len(state.track),len(state.local_track))
    state.trial.set_quality_policy(state,'fixed')
    assert state.alignment is None and state.trial.preview is None
    assert not state.trial.pairs and not state._alignment_pending
    assert (len(state.track),len(state.local_track))==before
    assert state.trial.active and not state.requests
    state.guiding=True
    with pytest.raises(ValueError):state.trial.set_quality_policy(state,'fixed_float')
    state.guiding=False
    state.trial.set_quality_policy(state,'fixed_float');state.save_session()
    restored=OutdoorState(tmp_path)
    assert restored.trial.quality_policy=='fixed_float'
    restored.trial.start(restored)
    assert restored.trial.quality_policy=='fixed_float'


def test_verified_overlay_preserves_independent_measurements_and_velocity_frames():
    state,now,world=aligned_trial();state.trial.projection=projection()
    # Allow a measured mismatch to stay visible; don't copy LIO to GNSS.
    feed(state,now,world,40,error=(2,0),quality=5)
    view=state.trial.visual_snapshot(state)
    assert view['aligned'] and not view['preview']
    assert math.dist(view['gnss'][-1]['point'],view['lidar'][-1]['point'])>1.8
    assert view['gnss'][-1]['selected']
    assert view['gnss_map'] and view['lidar_map']
    vx,vy=view['velocity_lidar']['components']
    assert abs(vx)<.01 and abs(vy-1.)<.01
    assert view['velocity_frame']=='east_north'
    assert view['velocity_gnss']['source']=='position_difference'
    now[0]+=3
    view=state.trial.visual_snapshot(state)
    assert view['velocity_lidar'] is None and view['velocity_gnss'] is None


def test_rtk_off_keeps_lidar_overlay_and_both_recorded_tracks():
    state,now,world=aligned_trial();state.trial.projection=projection()
    state.trial.set_rtk(state,False)
    for i in range(40,50):feed(state,now,world,i,quality=2)
    view=state.trial.visual_snapshot(state)
    assert view['aligned'] and view['lidar_map'] and view['gnss_map']
    assert state._position_now()['source']=='gnss_lio'
    assert state.trial.comparisons>0 and not state.requests


def test_preview_does_not_mirror_or_rescale_measurements():
    a=dict(yaw=.7,tx=15.,ty=-4.)
    points=[[0,0],[1,0],[2,0],[2,1],[2,2],[1,2]]
    fit=preview_fit([(p,transform(a,p)) for p in points])
    assert abs(fit['yaw']-.7)<1e-8 and fit['residual_m']<1e-8
    mirrored=preview_fit([(p,[p[0],-p[1]]) for p in points])
    assert mirrored['residual_m']>.5


@pytest.mark.parametrize('turn,word',[(1,'左转'),(-1,'右转')])
def test_walking_turn_cues_and_actual_offroute_position(turn,word):
    state=OutdoorState(clock=lambda:100.)
    lat=39.99;scale=111195*math.cos(math.radians(lat))
    p=lambda x,y:[116.35+x/scale,lat+y/111195]
    state.set_route(dict(points=[p(0,0),p(30,0),p(30,turn*40)],source='manual',coordinate_system='BD09',title='步行'))
    guide=state._guidance(p(15,0))
    assert word in guide['instruction'] and abs(guide['turn_distance_m']-15)<.1
    assert guide['bearing_deg']==90
    # Several normal updates move through the corner, then arrive.
    for x,y in ((25,0),(30,0),(30,turn*10),(30,turn*20),(30,turn*30),(30,turn*38)):
        guide=state._guidance(p(x,y))
    assert guide['near_destination'] and '到达' in guide['instruction']
    state.route_progress=0
    guide=state._guidance(p(15,-turn*15))
    assert guide['off_route_m']>=14.9 and '偏离' in guide['instruction']

import math
import numpy as np
import pytest
from dataclasses import dataclass, field
from nav_app.route_assistance_core import Route, RouteGuard


@dataclass
class PoseMode:
    pose: np.ndarray
    covariance: np.ndarray
    weight: float = 1.


@dataclass
class MapObservation:
    stamp: float
    available_at: float
    epoch: str
    tile_id: str
    evidence: tuple
    modes: tuple
    support_start: float
    support_end: float
    calibrated: bool = False
    map_bias_covariance: np.ndarray = field(default_factory=lambda: np.eye(3))


def route():
    return Route([[0,0],[40,0],[40,40]], map_error_m=3.)


def run(points, yaw=0., sigma=2.):
    g=RouteGuard(route(),'one',enabled=True)
    states=[g.update(i,[*p,yaw],sigma_m=sigma) for i,p in enumerate(points)]
    return g,states


def obs(t, xy, *, calibrated=True, epoch='one', evidence=None, available=None, modes=None):
    return MapObservation(t,t if available is None else available,epoch,'tile',
        tuple(evidence or [f'feature-{t}']),tuple(modes or [PoseMode(np.r_[xy,0.],np.diag([1.,1.,.01]))]),
        t-1,t,calibrated=calibrated,map_bias_covariance=np.diag([1.,1.,.01]))


def test_actual_left_turn_completes_and_approaches_before_corner():
    g,states=run([[x,0] for x in range(41)]+[[40,y] for y in range(1,41)])
    assert any(s['state']=='approaching' and s['position'][0]<35 for s in states)
    assert any(s['state']=='turn_complete' for s in states)
    assert not any(s['state'] in ('missed_turn','wrong_turn','early_turn') for s in states)
    assert g.completed==[0]
    assert states[-1]['state']=='arrived'


def test_missed_turn_is_not_hidden_by_route_projection():
    _,states=run([[x,0] for x in range(65)])
    events=[s for s in states if s['state']=='missed_turn']
    assert events and events[0]['position'][0] <= 49
    assert all(s['position'][1]==0 for s in states)
    assert states[-1]['completed_turns']==[]
    assert events[0]['capture_continues'] and events[0]['replan_available']


def test_wrong_branch_reported_after_motion_not_device_heading():
    _,states=run([[x,0] for x in range(41)]+[[40,-y] for y in range(1,15)],yaw=math.pi/2)
    assert any(s['state']=='wrong_turn' for s in states)
    assert not any(s['state']=='turn_complete' for s in states)


def test_early_left_turn_not_mistaken_for_intended_corner():
    _,states=run([[x,0] for x in range(29)]+[[28,y] for y in range(1,20)])
    assert any(s['state']=='early_turn' for s in states)
    assert not any(s['state']=='turn_complete' for s in states)


def test_stationary_device_rotation_never_completes_turn():
    g,_=run([[x,0] for x in range(38)])
    states=[g.update(38+i,[37,0,i*.2]) for i in range(25)]
    assert not any(s['state']=='turn_complete' for s in states)


def test_valid_image_changes_distance_interval_not_raw_position():
    g,_=run([[x,0] for x in range(31)],sigma=5.)
    assert g.observe(obs(30,[35,0]))=='accepted'
    s=g.update(31,[31,0,0],sigma_m=5.)
    assert s['position']==[31.,0.]
    assert s['hypotheses'][0]['xy'][0]>35
    assert s['sources']==['satellite']
    assert not s['changes_odometry']


def test_conflicting_image_modes_preserved_not_averaged():
    g,_=run([[x,0] for x in range(36)],sigma=6.)
    modes=[PoseMode(np.array([25.,0,0]),np.diag([1.,1.,.01]),.5),
           PoseMode(np.array([45.,0,0]),np.diag([1.,1.,.01]),.5)]
    assert g.observe(obs(35,[35,0],modes=modes))=='accepted'
    s=g.update(36,[36,0,0],sigma_m=6.)
    assert len(s['hypotheses'])==2 and s['state']=='junction_uncertain'


@pytest.mark.parametrize('source',['satellite','streetview'])
def test_source_contract_supports_both_map_providers(source):
    g,_=run([[x,0] for x in range(10)])
    assert g.observe(obs(9,[9,0]),source)=='accepted'
    assert g.update(10,[10,0,0])['sources']==[source]


def test_uncalibrated_real_matcher_does_not_gain_implicit_authority():
    g,_=run([[x,0] for x in range(10)])
    assert g.observe(obs(9,[14,0],calibrated=False))=='tentative'
    s=g.update(10,[10,0,0])
    assert s['sources']==['lidar','satellite_tentative']
    assert s['hypotheses'][0]['xy']==[10.,0.]
    assert s['hypotheses'][1]['sigma_m']>=5.


def test_delayed_observation_transported_from_capture_pose():
    g,_=run([[x,0] for x in range(20)],sigma=5.)
    assert g.observe(obs(16,[18,0],available=19))=='accepted'
    assert g.update(20,[20,0,0],sigma_m=5.)['hypotheses'][0]['xy'][0]>21
    # Directly treating old position 18 as current would move backwards.


def test_future_old_epoch_repeated_and_correlated_evidence_rejected():
    g,_=run([[x,0] for x in range(20)])
    assert g.observe(obs(19,[19,0],available=20))=='not_arrived'
    assert g.observe(obs(19,[19,0],epoch='other'))=='epoch_mismatch'
    assert g.observe(obs(19,[19,0],evidence=['same']))=='accepted'
    assert g.observe(obs(19,[19,0],evidence=['same']),'streetview')=='repeated_evidence'
    assert g.observe(obs(19,[19,0]))=='duplicate_or_old'


def test_old_image_expires_while_navigation_continues():
    g,_=run([[x,0] for x in range(20)])
    g.observe(obs(19,[19,0]))
    for t in range(20,30):s=g.update(t,[t,0,0])
    assert s['sources']==['lidar'] and s['source_status']['satellite']=='stale'
    assert s['capture_continues']


def test_epoch_reset_does_not_reuse_route_alignment():
    g,_=run([[x,0] for x in range(10)])
    s=g.update(10,[0,0,0],epoch='new')
    assert s['state']=='reinitialize' and s['capture_continues']
    assert g.snapshot()['state']=='reinitialize'
    assert g.observe(obs(9,[9,0]))=='reinitialize'


def test_gap_does_not_manufacture_turn_from_discontinuous_positions():
    g,_=run([[x,0] for x in range(31)])
    s=g.update(60,[40,10,1.57])
    assert s['state']!='turn_complete' and s['completed_turns']==[]


def test_parallel_return_leg_cannot_jump_progress():
    g=RouteGuard(Route([[0,0],[50,0],[50,4],[0,4]]),'one',enabled=True)
    for t in range(20):s=g.update(t,[t,3,0])
    assert s['progress_m']<30 and s['completed_turns']==[]


def test_uncertainty_widens_instruction_instead_of_false_precision():
    _,s=run([[x,0] for x in range(30)],sigma=10.)
    assert s[-1]['state']=='junction_uncertain'
    assert s[-1]['distance_interval_m'][1]-s[-1]['distance_interval_m'][0]>=40


def test_bad_input_time_and_route():
    with pytest.raises(ValueError):Route([[0,0],[0,0]])
    g,_=run([[x,0] for x in range(5)])
    with pytest.raises(ValueError):g.update(4,[4,0,0])
    with pytest.raises(ValueError):g.update(5,[math.nan,0,0])


def test_repeated_ui_reads_do_not_change_events_or_progress():
    g,states=run([[x,0] for x in range(41)]+[[40,y] for y in range(1,8)])
    before=g.snapshot()
    for _ in range(20):assert g.snapshot()==before
    assert before==states[-1]


def test_off_route_persists_long_enough_to_report():
    g=RouteGuard(Route([[0,0],[100,0]]),'one',enabled=True)
    states=[g.update(t,[t,25,0]) for t in range(10)]
    assert any(s['state']=='off_route' for s in states)


def test_multiple_turns_are_confirmed_in_order():
    g=RouteGuard(Route([[0,0],[30,0],[30,30],[60,30]]),'one',enabled=True)
    points=[[x,0] for x in range(31)]+[[30,y] for y in range(1,31)]+[[x,30] for x in range(31,61)]
    states=[g.update(t,[*p,0]) for t,p in enumerate(points)]
    assert g.completed==[0,1]
    assert not any(s['state'] in ('wrong_turn','missed_turn') for s in states)


def test_small_gradual_drift_does_not_hide_a_real_turn():
    g=RouteGuard(route(),'one',enabled=True)
    path=[[x,0] for x in range(41)]+[[40,y] for y in range(1,41)]
    states=[g.update(t,[p[0]+3*t/80,p[1],0],sigma_m=3.) for t,p in enumerate(path)]
    assert g.completed==[0]
    assert not any(s['state'] in ('wrong_turn','missed_turn') for s in states)


def test_wrong_road_warning_does_not_disappear_when_walking_straight_again():
    g,states=run([[x,0] for x in range(41)]+[[40,-y] for y in range(1,15)])
    assert states[-1]['state']=='wrong_turn'
    for t in range(55,70):s=g.update(t,[40+t-55,-14,0])
    assert s['state'] in ('wrong_turn','off_route') and s['replan_available']


def test_large_position_uncertainty_reports_near_destination_not_arrived():
    g=RouteGuard(Route([[0,0],[40,0]]),'one',enabled=True)
    states=[g.update(t,[t,0,0],sigma_m=6.) for t in range(41)]
    assert states[-1]['state']=='near_destination'
    assert not any(s['state']=='arrived' for s in states)


def test_assistance_defaults_off_and_bypasses_auxiliary_work(monkeypatch):
    g=RouteGuard(route(),'one')
    def forbidden(*args, **kwargs):
        pytest.fail('disabled assistance evaluated route or imagery')
    monkeypatch.setattr(g,'_evaluate',forbidden)
    monkeypatch.setattr(g,'positions',forbidden)
    for t in range(65):
        s=g.update(t,[t,0,0])
        assert s['state']=='disabled' and s['use_base_navigation']
        assert s['position']==[float(t),0.] and s['capture_continues']
    assert g.observe(obs(64,[64,0]))=='disabled'
    assert not g.history and not g.map_observations and not g._seen


def test_disabling_clears_warning_and_all_pending_candidates():
    g,states=run([[x,0] for x in range(60)])
    assert states[-1]['state']=='missed_turn'
    g.observe(obs(59,[59,0]))
    s=g.set_enabled(False)
    assert s['state']=='disabled' and not s['replan_available']
    assert not s['hypotheses'] and not s['source_status']
    assert g.deviation is None and not g.map_observations
    assert g.update(60,[60,0,0])['position']==[60.,0.]


def test_resume_uses_current_base_route_and_rejects_old_jobs():
    g,_=run([[x,0] for x in range(20)])
    g.set_enabled(False)
    g.update(30,[40,10,0])
    g.set_enabled(True,progress_m=50.,next_turn=1)
    assert g.update(31,[40,11,0])['completed_turns']==[0]
    assert g.observe(obs(19,[19,0],available=31))=='capture_pose_missing'
    assert g.update(32,[40,12,0])['state']=='continue'
    assert g.observe(obs(32,[40,12]))=='accepted'


def test_resume_without_base_context_does_not_replay_old_warning():
    g,_=run([[x,0] for x in range(60)])
    g.set_enabled(False)
    g.update(60,[65,0,0])
    g.set_enabled(True)
    s=g.update(61,[66,0,0])
    assert s['state']=='waiting_route_context' and s['use_base_navigation']
    assert g.observe(obs(61,[66,0]))=='waiting_route_context'
    g.set_enabled(True,progress_m=40.,next_turn=0)
    assert g.update(62,[67,0,0])['state']!='missed_turn'


def test_toggle_does_not_clear_epoch_reset_or_accept_invalid_context():
    g=RouteGuard(route(),'one')
    g.update(100,[0,0,0])
    g.update(0,[0,0,0],epoch='new')
    g.set_enabled(True,progress_m=0.,next_turn=0)
    assert g.update(1,[1,0,0],epoch='new')['state']=='reinitialize'
    with pytest.raises(ValueError):g.set_enabled(False,progress_m=4.)
    with pytest.raises(ValueError):g.set_enabled(True,progress_m=999.,next_turn=0)


def test_enabling_same_setting_preserves_active_evidence_and_motion():
    g,_=run([[x,0] for x in range(20)])
    g.observe(obs(19,[19,0]))
    before=g.snapshot()
    assert g.set_enabled(True)==before
    assert g.map_observations and len(g.history)==20


def test_projection_clips_long_segments_to_progress_window():
    r=Route([[0,0],[100,0]])
    assert r.project(np.array([40.,0.]), 10., 20.) == (20.,20.)
    assert r.project(np.array([5.,0.]), 10., 20.) == (10.,5.)
    assert r.project(np.array([5.,0.]), 110., 120.) is None


def test_starting_partway_uses_explicit_progress_without_false_deviation():
    g=RouteGuard(Route([[0,0],[100,0]]),'one')
    g.set_enabled(True,progress_m=40.,next_turn=0)
    for t in range(5):
        s=g.update(t,[40.+t,0.,0.])
        assert s['state']=='continue' and not s['replan_available']
    assert s['progress_m']==44.


def test_worker_result_budgets_are_rejected_without_disrupting_motion():
    g,_=run([[x,0] for x in range(10)])
    too_many_modes=obs(9,[9,0],modes=[PoseMode(np.zeros(3),np.eye(3))]*9)
    assert g.observe(too_many_modes)=='observation_budget'
    assert g.observe(obs(9,[9,0],evidence=[str(i) for i in range(129)]))=='observation_budget'
    assert not g.map_observations and not g._seen
    assert g.update(10,[10,0,0])['state']=='continue'


@pytest.mark.parametrize('field,value',[
    ('stamp',float('nan')), ('available_at',-1), ('calibrated','yes'),
    ('evidence',('duplicate','duplicate')), ('evidence',('x'*161,)),
    ('map_bias_covariance',np.zeros((3,3))),
])
def test_malformed_observation_is_optional_failure(field,value):
    g,_=run([[x,0] for x in range(10)])
    o=obs(9,[9,0]);setattr(o,field,value)
    assert g.observe(o)=='invalid_observation'
    assert not g.map_observations
    assert g.update(10,[10,0,0])['capture_continues']


def test_producer_mutation_and_unrelated_payload_never_enter_core():
    g,_=run([[x,0] for x in range(10)])
    o=obs(9,[9,0]);o.unused_payload=object()
    assert g.observe(o)=='accepted'
    o.modes[0].pose[0]=999.;o.map_bias_covariance[:]=999.
    stored=g.map_observations['satellite']
    assert stored.modes[0].pose[0]==9. and stored.map_bias_covariance[0,0]==1.
    assert not hasattr(stored,'unused_payload')
    assert g.update(10,[10,0,0])['sources']==['satellite']


def test_evidence_ledger_expires_only_outside_motion_window():
    g=RouteGuard(route(),'one',enabled=True,history_s=2.)
    g.update(0,[0,0,0]);assert g.observe(obs(0,[0,0],evidence=['landmark']))=='accepted'
    g.update(1,[1,0,0]);assert g.observe(obs(1,[1,0],evidence=['landmark']))=='repeated_evidence'
    for t in (2,3):g.update(t,[t,0,0])
    assert not g._seen
    assert g.observe(obs(0,[0,0],available=3))=='duplicate_or_old'
    assert g.observe(obs(3,[3,0],evidence=['landmark']))=='accepted'


def test_evidence_budget_does_not_evict_recent_identifiers():
    g=RouteGuard(route(),'one',enabled=True)
    for t in range(16):
        g.update(t,[t,0,0])
        assert g.observe(obs(t,[t,0],evidence=[f'{t}-{i}' for i in range(128)]))=='accepted'
    assert len(g._seen)==2048
    g.update(16,[16,0,0])
    assert g.observe(obs(16,[16,0]))=='evidence_budget'
    assert g.observe(obs(16,[16,0],evidence=['0-0']))=='repeated_evidence'
    assert len(g._seen)==2048


def test_disabling_and_tracking_gap_release_evidence():
    g,_=run([[x,0] for x in range(10)])
    g.observe(obs(9,[9,0]))
    g.set_enabled(False)
    assert not g._seen and not g.source_status
    g.set_enabled(True,progress_m=9.,next_turn=0)
    g.update(10,[10,0,0]);g.observe(obs(10,[10,0]))
    g.update(20,[11,0,0])
    assert not g._seen and not g.map_observations and not g.source_status

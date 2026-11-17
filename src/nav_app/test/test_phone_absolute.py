"""Mathematical and causal-boundary regressions; no sensors or networking."""
import copy
import math
import time
import numpy as np
import pytest
from scipy.sparse.linalg import lsmr
from nav_app.phone_absolute import evaluate_absolute_window
from nav_app.phone_absolute_geometry import AbsoluteModel,frame,geometric_range,prepare
from nav_app.phone_absolute_solver import phase_gauge_fixed,rotation,solve


def example(epochs=130,satellite_count=6):
    origin=np.array([6378137.,0.,0.]);E=frame(origin);yaw=.6
    sats=np.array([[2.,1.,1.],[1.,-2.,1.],[2.,0.,-1.],[1.,2.,-1.],[2.,-1.,-1.],[2.,-1.,2.]])*1e7
    if satellite_count!=6:
        angles=np.arange(satellite_count)*2.399963229728653
        sats=np.column_stack([1.5+.4*np.cos(angles*.3),1.4*np.sin(angles),1.4*np.cos(angles)])*1e7
    poses=[];raw=[]
    for t in np.arange(999.9,1000+epochs+.2,.1):
        p=[20*math.sin((t-1000)/40),.15*(t-1000),.01*(t-1000)]
        poses.append(dict(utc_ms=t*1000,position=p,epoch_id='odom1'))
    for i in range(epochs):
        t=1000+i;p=np.array([20*math.sin(i/40),.15*i,.01*i]);truth=origin+(rotation(yaw)[0]@p)@E
        g,_=geometric_range(sats,np.repeat(truth[None,:],satellite_count,axis=0));signals=[]
        for k in range(satellite_count):
            signals.append(dict(id=f'G{k+1:02d}',system=1,code=1,frequency_hz=1575420000.,satellite_ecef_m=sats[k].tolist(),code_valid=True,phase_valid=True,code_observation_m=float(g[k]+100+i*.3),phase_observation_m=float(g[k]+100+i*.3+10*k),code_sigma_m=2.,phase_sigma_m=.05,lli_rover=0,lli_base=0,troposphere_m=0.,troposphere_gradient_ecef=[0.,0.,0.]))
        raw.append(dict(schema_version=1,utc_ms=t*1000,base_utc_ms=t*1000,base_ecef_m=(origin+[0,1000,0]).tolist(),station_id=1,clock_segment=0,reference_ecef_m=(origin+E.T@np.array([30,-20,10])).tolist(),signals=signals))
    return dict(source_id='phone',session_id='session',continuity_id='run1',generation=1,odom_epoch_id='odom1',available_utc_ms=(1000+epochs+1)*1000.,observations=raw,poses=poses,lever_arm_uncertainty_m=1.),truth,yaw


def model_input(s):
    rr=[dict(utc_ms=r['utc_ms'],x0=r['reference_ecef_m'],clock_segment=r['clock_segment'],rows=r['signals']) for r in s['observations']]
    return rr,np.array([[p['utc_ms']/1000,*p['position']] for p in s['poses']]),AbsoluteModel(rr)


def test_known_truth_and_gauge_fix_preserve_position_without_native_prior():
    s,truth,yaw=example();rr,pp,model=model_input(s)
    fit=solve(rr,pp,model,lever_sigma_m=1.)
    assert np.linalg.norm(np.array(fit['final_ecef'])-truth)<.01
    assert abs(math.atan2(math.sin(math.radians(fit['yaw_deg'])-yaw),math.cos(math.radians(fit['yaw_deg'])-yaw)))<.001
    d=prepare(rr,model);b=phase_gauge_fixed(d);z=np.random.default_rng(1).normal(size=b.shape[0])
    a=d['B']@lsmr(d['B'],z,atol=1e-12,btol=1e-12)[0]
    c=b@lsmr(b,z,atol=1e-12,btol=1e-12)[0]
    assert np.max(abs(a-c))<1e-8
    assert b.shape[1]==d['B'].shape[1]-1


def test_geometry_and_yaw_jacobians():
    s,_,_=example();rr,pp,m=model_input(s);r=rr[60];t=np.array([r['utc_ms']/1000]);rs=np.array([r['rows'][0]['satellite_ecef_m']]);x=np.array([r['x0']]);_,h=m(t,rs,x)
    for k in range(3):
        d=np.zeros((1,3));d[0,k]=.05
        assert (m(t,rs,x+d)[0]-m(t,rs,x-d)[0])/.1==pytest.approx(h[:,k],abs=1e-6)
    y=.4;v=np.array([.3,.2,.1]);eps=1e-6
    assert (rotation(y+eps)[0]@v-rotation(y-eps)[0]@v)/(2*eps)==pytest.approx(rotation(y)[1]@v,abs=1e-8)


@pytest.mark.parametrize('mutation,reason',[
    (lambda s:s.update(source_id=''),'missing_source_identity'),
    (lambda s:s['observations'][0].update(utc_ms=2e9),'future_observation'),
    (lambda s:s['poses'][-1].update(utc_ms=2e9),'future_livo_pose'),
    (lambda s:s['poses'][50].update(epoch_id='other'),'mixed_livo_epochs'),
    (lambda s:s['poses'][50].update(utc_ms=s['poses'][49]['utc_ms']),'nonmonotonic_livo_time'),
    (lambda s:s['observations'][0]['signals'].append(copy.deepcopy(s['observations'][0]['signals'][0])),'duplicate_signal'),
    (lambda s:s['observations'][0]['signals'][0].update(code_sigma_m=float('nan')),'invalid_raw_scalar'),
    (lambda s:s.update(lever_arm_m=[1,0,0]),'nonzero_lever_arm_requires_orientation_samples'),
])
def test_bad_input_cannot_emit_anchor(mutation,reason):
    s,_,_=example();mutation(s);r=evaluate_absolute_window(s)
    assert not r['accepted_position'] and not r['accepted_yaw'] and r['reason']==reason


def test_deadline_does_not_publish_unvalidated_partial_solution():
    s,_,_=example();r=evaluate_absolute_window(s,deadline_monotonic=time.monotonic()-1)
    assert not r['accepted'] and 'deadline' in r['reason']


def test_tracking_hole_discards_pre_hole_absolute_window():
    s,_,_=example();s['poses']=[p for p in s['poses'] if not 1060000<p['utc_ms']<1071000]
    r=evaluate_absolute_window(s);assert not r['accepted'] and r['reason']=='insufficient_data'


def test_receiver_clock_segment_splits_carrier_arc_but_solver_generation_does_not():
    s,_,_=example();rr,pp,m=model_input(s);a=prepare(rr,m)
    for r in rr[60:]:r['clock_segment']=1
    b=prepare(rr,m);assert len(b['index'])==len(a['index'])+6


def test_static_motion_keeps_position_without_inventing_heading():
    s,_,_=example()
    for p in s['poses']:p['position']=[0.,0.,0.]
    for r in s['observations']:
        for i,q in enumerate(r['signals']):
            g,_=geometric_range(np.array([q['satellite_ecef_m']]),np.array([[6378137.,0.,0.]]))
            q['code_observation_m']=float(g[0]+100);q['phase_observation_m']=float(g[0]+100+10*i)
    result=evaluate_absolute_window(s,deadline_monotonic=time.monotonic()+30)
    assert result['accepted_position'] and result['rank']=={'position':3,'yaw':0}
    assert not result['accepted_yaw'] and not result['yaw_candidate_stable']
    assert np.linalg.norm(np.array(result['anchor_ecef_m'])-[6378137.,0.,0.])<.01


def test_full_validation_accepts_position_but_never_skips_future_yaw_check():
    s,truth,_=example();r=evaluate_absolute_window(s)
    assert r['accepted_position'] and r['accepted'] and r['yaw_candidate_stable']
    assert not r['accepted_yaw'] and r['yaw_reason']=='future_holdout_required'
    assert np.linalg.norm(np.array(r['anchor_ecef_m'])-truth)<.01
    assert min(r['position_sigma_enu_m'])>=5
    assert not r['absolute_accuracy_verified'] and not r['confidence_calibrated']
    assert r['anchor_point']=='livo_reference' and not r['installation_offset_calibrated']
    def check_json_types(value):
        assert type(value) in (dict,list,str,bool,int,float,type(None))
        if isinstance(value,dict):
            for v in value.values():check_json_types(v)
        if isinstance(value,list):
            for v in value:check_json_types(v)
    check_json_types(r)


def test_phone_swing_is_not_installed_as_body_movement():
    s,body_truth,_=example();origin=np.array([6378137.,0.,0.]);E=frame(origin)
    def exact_range(sat,x):
        # Independent receive-frame Earth rotation, not geometric_range().
        tau=np.linalg.norm(sat-x)/299792458.
        for _ in range(6):
            a=7.2921151467e-5*tau;c,z=np.cos(a),np.sin(a)
            rotated=np.array([c*sat[0]+z*sat[1],-z*sat[0]+c*sat[1],sat[2]])
            distance=np.linalg.norm(rotated-x);tau=distance/299792458.
        return distance
    for i,r in enumerate(s['observations']):
        p=np.array([20*math.sin(i/40),.15*i,.01*i])
        offset=np.array([.8*math.sin(i*.8),.4*math.cos(i*.7),.2*math.sin(i*.6)])
        antenna=origin+(rotation(.6)[0]@p+offset)@E
        for k,q in enumerate(r['signals']):
            g=exact_range(np.array(q['satellite_ecef_m']),antenna)
            q['code_observation_m']=g+100+i*.3;q['phase_observation_m']=g+100+i*.3+10*k
    rr,pp,model=model_input(s);fit=solve(rr,pp,model,lever_sigma_m=1.)
    assert np.linalg.norm(np.array(fit['final_ecef'])-body_truth)>.35
    assert np.linalg.norm(np.array(fit['final_body_ecef'])-body_truth)<.1
    assert fit['body_position_rank']==3
    assert all(math.isfinite(v) and v>0 for v in fit['body_model_sigma_enu_m'])
    expected=np.array(fit['final_ecef'])-np.array(fit['phone_offset_enu_m'][-1])@frame(np.array(rr[0]['x0']))
    assert fit['final_body_ecef']==pytest.approx(expected,abs=1e-8)
    result=evaluate_absolute_window(s)
    assert result['accepted_position'] and result['anchor_point']=='livo_reference'
    assert result['anchor_ecef_m']==pytest.approx(fit['final_body_ecef'],abs=.001)
    assert not result['installation_offset_calibrated'] and min(result['position_sigma_enu_m'])>=5.


@pytest.mark.parametrize('explicit_validity',[None,False,True])
def test_bad_atmospheric_linearization_skips_only_affected_epochs(explicit_validity):
    s,truth,_=example()
    for raw in s['observations'][50:53]:
        raw['reference_ecef_m']=[6378137.-350,0.,0.]
        if explicit_validity is not None:raw['troposphere_model_valid']=explicit_validity
        for signal in raw['signals']:signal['code_observation_m']+=5000
    before=copy.deepcopy(s)
    result=evaluate_absolute_window(s)
    assert result['rejected_model_epochs']==3 and result['epoch_count']==127
    assert result['accepted_position'] and not result['accepted_yaw']
    assert np.linalg.norm(np.array(result['anchor_ecef_m'])-truth)<.01
    assert s==before


def test_explicit_invalid_model_does_not_emit_an_anchor_or_rewrite_input():
    s,_,_=example()
    for raw in s['observations']:raw['troposphere_model_valid']=False
    result=evaluate_absolute_window(s)
    assert result['rejected_model_epochs']==130
    assert not result['accepted'] and result['reason']=='insufficient_data'


def test_model_validity_is_boolean_not_truthy():
    s,_,_=example();s['observations'][0]['troposphere_model_valid']=1
    result=evaluate_absolute_window(s)
    assert not result['accepted'] and result['reason']=='invalid_troposphere_model_validity'


def test_legacy_invalid_base_atmosphere_cannot_supply_an_anchor():
    s,_,_=example()
    for raw in s['observations']:
        raw['base_ecef_m']=[6378137.-350,0.,0.]
        raw['troposphere_model_valid']=True
    result=evaluate_absolute_window(s)
    assert not result['accepted'] and result['rejected_model_epochs']==130

"""Bounded, causal raw-GNSS/LiVO absolute candidates for a background worker.

Call only in the application's isolated computation worker. This function never
controls sensors, modifies native RTK, or publishes a current navigation pose.
The consumer must check source/session/generation and transform the returned
historical anchor through the matching LiVO epoch before displaying current pose.
"""
import math
import numbers
import time


def _finite_vector(value,n):
    return isinstance(value,(list,tuple)) and len(value)==n and all(isinstance(v,(float,int)) and math.isfinite(v) for v in value)


def _plain_json(value):
    """Keep Process/Pipe results identical to persisted JSON replay results."""
    if value is None or type(value) in (str,bool,int):return value
    if isinstance(value,numbers.Integral):return int(value)
    if isinstance(value,numbers.Real):
        out=float(value)
        if not math.isfinite(out):raise ValueError('nonfinite_result')
        return out
    if isinstance(value,dict):return {str(k):_plain_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [_plain_json(v) for v in value]
    raise TypeError('unsupported_result_type')


def evaluate_absolute_window(snapshot,*,deadline_monotonic=None):
    started=time.monotonic();deadline_monotonic=started+90 if deadline_monotonic is None else deadline_monotonic
    result={k:snapshot.get(k) for k in ('source_id','session_id','continuity_id','generation','odom_epoch_id','available_utc_ms')}
    result.update(valid=False,accepted=False,accepted_position=False,accepted_yaw=False,reason='insufficient_data',confidence_calibrated=False,absolute_accuracy_verified=False,method='raw_code_phase_livo_window_v1',rejected_model_epochs=0)
    try:
        if any(not snapshot.get(k) for k in ('source_id','session_id','continuity_id')):raise ValueError('missing_source_identity')
        available=float(snapshot['available_utc_ms'])
        if not math.isfinite(available):raise ValueError('invalid_available_time')
        observations=snapshot.get('observations',[]);poses=snapshot.get('poses',[])
        if len(observations)>300 or len(poses)>6000:raise ValueError('window_budget_exceeded')
        if not observations or len(poses)<2:return result
        if any(float(r['utc_ms'])>available+.1 for r in observations):raise ValueError('future_observation')
        if any(float(p['utc_ms'])>available+.1 for p in poses):raise ValueError('future_livo_pose')
        if any(not _finite_vector(p.get('position'),3) for p in poses):raise ValueError('invalid_livo_position')
        epochs={str(p.get('epoch_id')) for p in poses}
        if len(epochs)!=1 or 'None' in epochs:raise ValueError('mixed_livo_epochs')
        odom_epoch=next(iter(epochs));result['odom_epoch_id']=odom_epoch
        if snapshot.get('odom_epoch_id') is not None and str(snapshot['odom_epoch_id'])!=odom_epoch:raise ValueError('livo_epoch_mismatch')
        if any(float(poses[i+1]['utc_ms'])<=float(poses[i]['utc_ms']) for i in range(len(poses)-1)):raise ValueError('nonmonotonic_livo_time')
        if any(float(observations[i+1]['utc_ms'])<float(observations[i]['utc_ms']) for i in range(len(observations)-1)):raise ValueError('nonmonotonic_observation_time')
        # A long LiVO delivery/tracking hole is not a measured relative-motion
        # edge. Retain only the continuous suffix; do not bridge a missing walk.
        gaps=[i for i in range(len(poses)-1) if float(poses[i+1]['utc_ms'])-float(poses[i]['utc_ms'])>1000]
        if gaps:
            poses=poses[gaps[-1]+1:]
            observations=[r for r in observations if float(r['utc_ms'])>=float(poses[0]['utc_ms'])]
            if not observations or len(poses)<2:return result
        # A restart of the reference station changes ambiguity arcs. Start a new
        # window rather than silently joining two independently defined bases.
        last_base=observations[-1];records=[];seen={}
        for raw in observations:
            if raw.get('schema_version')!=1:raise ValueError('unsupported_raw_schema')
            t=float(raw['utc_ms'])
            if t<float(last_base['utc_ms'])-240000:continue
            if t in seen:
                if seen[t]!=raw:raise ValueError('conflicting_duplicate_observation')
                continue
            seen[t]=raw
            if not _finite_vector(raw.get('reference_ecef_m'),3) or not _finite_vector(raw.get('base_ecef_m'),3):raise ValueError('invalid_ecef_reference')
            base_same=raw.get('station_id')==last_base.get('station_id') and sum((a-b)**2 for a,b in zip(raw['base_ecef_m'],last_base['base_ecef_m']))<.01**2
            if not base_same:records=[];continue
            if abs(t-float(raw['base_utc_ms']))>2000:continue
            if 'troposphere_model_valid' in raw and type(raw['troposphere_model_valid']) is not bool:
                raise ValueError('invalid_troposphere_model_validity')
            # RTKLIB's atmosphere model returns zero outside this height domain.
            # Such a zero and its zero derivative are not valid observations of
            # the atmosphere. Retain the original archive, skip only this epoch,
            # and continue with usable epochs. Check older schema-1 recordings
            # by geometry too; a caller-supplied true flag cannot override it.
            from .phone_absolute_geometry import geodetic
            heights=[geodetic(raw[key])[2] for key in ('reference_ecef_m','base_ecef_m')]
            if raw.get('troposphere_model_valid') is False or any(not -100.<=h<=10000. for h in heights):
                result['rejected_model_epochs']+=1
                continue
            signals=raw.get('signals',[])
            if len(signals)>32:raise ValueError('signal_budget_exceeded')
            rows=[];signal_keys=set()
            for q in signals:
                key=(q.get('system'),q.get('id'),q.get('code'),q.get('frequency_hz'))
                if key in signal_keys:raise ValueError('duplicate_signal')
                signal_keys.add(key)
                if not _finite_vector(q.get('satellite_ecef_m'),3) or not _finite_vector(q.get('troposphere_gradient_ecef'),3):raise ValueError('invalid_satellite_geometry')
                if not q.get('code_valid'):continue
                for name in ('frequency_hz','code_observation_m','code_sigma_m','phase_sigma_m','troposphere_m'):
                    if not isinstance(q.get(name),(int,float)) or not math.isfinite(q[name]):raise ValueError('invalid_raw_scalar')
                if q['frequency_hz']<=0 or q['code_sigma_m']<=0 or q['phase_sigma_m']<=0:raise ValueError('invalid_signal_scale')
                if q.get('phase_valid') and (not isinstance(q.get('phase_observation_m'),(int,float)) or not math.isfinite(q['phase_observation_m'])):raise ValueError('invalid_phase_observation')
                rows.append(dict(q))
            if rows:records.append({'utc_ms':t,'x0':list(raw['reference_ecef_m']),'clock_segment':raw.get('clock_segment',0),'rows':rows})
        if len(records)<80 or records[-1]['utc_ms']-records[0]['utc_ms']<120000:return result
        if sum(q.get('phase_valid',False) for r in records for q in r['rows'])<40:return result
        if any(float(v)!=0 for v in snapshot.get('lever_arm_m',[0,0,0])):raise ValueError('nonzero_lever_arm_requires_orientation_samples')
        # Imports and numerical work happen only inside the computation worker.
        import numpy as np
        from .phone_absolute_geometry import AbsoluteModel,frame,geodetic
        from .phone_absolute_solver import solve
        model=AbsoluteModel(records);pp=np.array([[float(p['utc_ms'])/1000,*p['position']] for p in poses])
        lever_sigma=float(snapshot.get('lever_arm_uncertainty_m',1.));time_sigma=float(snapshot.get('pose_time_uncertainty_s',.05))
        if not math.isfinite(lever_sigma) or lever_sigma<0 or not math.isfinite(time_sigma) or time_sigma<0:raise ValueError('invalid_installation_uncertainty')
        common=dict(code_bias_scale=3.,edge_sigma=.05,deadline_monotonic=deadline_monotonic,lever_sigma_m=lever_sigma)
        fit=solve(records,pp,model,seed_yaw=0.,**common)
        if fit['body_position_rank']<3:raise ValueError('absolute_position_rank_deficient')
        x=np.array(fit['final_body_ecef']);ef=frame(x);yaw=math.radians(fit['yaw_deg']);maximum_seed=0.;maximum_seed_yaw=0.
        for seed in (math.pi,-math.pi/2):
            other=solve(records,pp,model,seed_yaw=seed,compute_covariance=False,**common)
            maximum_seed=max(maximum_seed,float(np.linalg.norm((ef@(np.array(other['final_body_ecef'])-x))[:2])))
            delta=math.radians(other['yaw_deg'])-yaw;maximum_seed_yaw=max(maximum_seed_yaw,abs(math.atan2(math.sin(delta),math.cos(delta))))
        deviations=[];yaw_deviations=[];leaveouts=[]
        for sat in sorted({q['id'] for r in records for q in r['rows']}):
            other=solve(records,pp,model,seed_yaw=yaw,exclude=[sat],compute_covariance=False,warm_start=fit,**common)
            deviations.append(ef@(np.array(other['final_body_ecef'])-x));delta=math.radians(other['yaw_deg'])-yaw;yaw_deviations.append(abs(math.atan2(math.sin(delta),math.cos(delta))));leaveouts.append((sat,other))
        # Recheck the most influential leave-outs from a distant position and
        # different heading. Warm starts accelerate only; they are no priors.
        cold_gap=0.;cold_yaw_gap=0.
        indices=sorted(range(len(deviations)),key=lambda i:np.linalg.norm(deviations[i][:2]),reverse=True)[:2]
        for i in indices:
            sat,warm=leaveouts[i];cold=solve(records,pp,model,seed_yaw=math.pi,seed_enu=[100.,-70.,30.],exclude=[sat],compute_covariance=False,**common)
            cold_gap=max(cold_gap,float(np.linalg.norm((ef@(np.array(cold['final_body_ecef'])-warm['final_body_ecef']))[:2])))
            d=math.radians(cold['yaw_deg']-warm['yaw_deg']);cold_yaw_gap=max(cold_yaw_gap,abs(math.atan2(math.sin(d),math.cos(d))))
        loo=max(float(np.linalg.norm(v[:2])) for v in deviations);loo_yaw=max(yaw_deviations)
        # The same physical GNSS observations underlie the native solution and
        # this candidate. Do not count the two as independent precise priors.
        speed=np.linalg.norm(np.diff(pp[:,1:4],axis=0),axis=1)/np.diff(pp[:,0]);timing_error=float(np.quantile(speed,.95))*time_sigma
        marginal=np.array(fit['body_model_sigma_enu_m']);influence=np.max(np.abs(np.asarray(deviations)),axis=0)
        sigma=np.sqrt(np.maximum(marginal,influence)**2+lever_sigma**2+timing_error**2);sigma=np.maximum(sigma,[5.,5.,5.])
        lat,lon,alt=geodetic(x);result.update(valid=True,anchor_utc_ms=fit['end_utc_ms'],anchor_point='livo_reference',anchor_ecef_m=x.tolist(),anchor_latitude=lat,anchor_longitude=lon,anchor_altitude_ellipsoid_m=alt,phone_antenna_ecef_m=fit['final_ecef'],estimated_phone_offset_enu_m=fit['phone_offset_enu_m'][-1],installation_offset_calibrated=False,yaw_rad=math.atan2(math.sin(yaw),math.cos(yaw)),yaw_sigma_rad=max(fit['yaw_model_sigma_rad'],loo_yaw,maximum_seed_yaw,math.radians(3.)),position_sigma_enu_m=sigma.tolist(),model_position_sigma_enu_m=marginal.tolist(),loo_max_horizontal_m=loo,loo_max_yaw_rad=loo_yaw,seed_max_horizontal_m=maximum_seed,seed_max_yaw_rad=maximum_seed_yaw,rank={'position':fit['body_position_rank'],'yaw':fit['yaw_rank']},epoch_count=fit['epochs'],observation_count=fit['rows'],rejected_pose_brackets=fit['rejected_pose_brackets'],code_bias_m=fit['code_bias_m'],lever_arm_uncertainty_m=lever_sigma,pose_time_uncertainty_s=time_sigma,source_correlated_with_native=True,future_holdout_verified=False,window_start_utc_ms=records[0]['utc_ms'])
        result['leaveout_cold_max_horizontal_m']=cold_gap;result['leaveout_cold_max_yaw_rad']=cold_yaw_gap
        result['accepted_position']=bool(maximum_seed<=2 and cold_gap<=2 and loo<=5 and np.linalg.norm(marginal[:2])<=15)
        result['yaw_candidate_stable']=bool(fit['yaw_rank'] and maximum_seed_yaw<=math.radians(5) and cold_yaw_gap<=math.radians(5) and loo_yaw<=math.radians(8))
        result['accepted_yaw']=False
        result['yaw_reason']='future_holdout_required' if result['yaw_candidate_stable'] else 'yaw_window_unstable'
        result['accepted']=result['accepted_position']
        result['reason']='consistent_absolute_candidate' if result['accepted_position'] else 'absolute_window_unstable'
        if time.monotonic()>deadline_monotonic:raise TimeoutError('absolute window validation deadline exceeded')
    except Exception as exc:
        result.update(accepted=False,accepted_position=False,accepted_yaw=False,reason=str(exc))
    finally:
        result['computed_wall_seconds']=time.monotonic()-started
    try:return _plain_json(result)
    except (TypeError,ValueError):
        return dict(valid=False,accepted=False,accepted_position=False,accepted_yaw=False,reason='invalid_result_serialization')

"""Bounded off-thread raw-GNSS/LiVO windows and generation-checked results."""
from collections import deque
import copy
import math
import multiprocessing
import os
import time


def number(value):
    return type(value) in (int,float) and math.isfinite(value)


def _evaluate(snapshot):
    # Only the new worker process is reprioritized; never the ROS/UI process.
    for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
        os.environ[key]='1'
    if hasattr(os,'nice'):
        try:os.nice(5)
        except OSError:pass
    from .phone_absolute import evaluate_absolute_window
    return evaluate_absolute_window(snapshot,deadline_monotonic=time.monotonic()+90.)


def _evaluate_child(snapshot,connection):
    try:connection.send(_evaluate(snapshot))
    except BaseException as error:
        try:connection.send(dict(accepted=False,valid=False,reason=type(error).__name__))
        except (BrokenPipeError,EOFError,OSError):pass
    finally:connection.close()


class AbsoluteQueue:
    """One running process and one replaceable newest request, no blocking ROS."""
    def __init__(self,child_target=_evaluate_child,deadline_seconds=95.):
        self.future=None;self.pending=None;self.child_target=child_target
        self.deadline_seconds=deadline_seconds

    @staticmethod
    def _stop(job):
        process=job.get('process');connection=job.get('connection')
        if connection:
            try:connection.close()
            except OSError:pass
        if process:
            if process.is_alive():process.terminate()
            process.join(timeout=.2)
            if process.is_alive():process.kill();process.join(timeout=.2)
            if not process.is_alive():process.close()

    def submit(self,snapshot,context):
        if self.future is not None:self.pending=(snapshot,context);return
        receive=send=None
        try:
            factory=multiprocessing.get_context('spawn');receive,send=factory.Pipe(duplex=False)
            process=factory.Process(target=self.child_target,args=(snapshot,send),daemon=True)
            process.start();send.close()
            self.future=dict(process=process,connection=receive,context=context,started=time.monotonic())
        except (RuntimeError,OSError) as error:
            for connection in (receive,send):
                if connection:connection.close()
            self.future=dict(context=context,result=dict(accepted=False,valid=False,reason='worker_unavailable:'+type(error).__name__))

    def poll(self,wait=False):
        if self.future is None:return None
        job=self.future;context=job['context'];result=job.get('result')
        if result is None:
            remaining=max(0.,self.deadline_seconds-(time.monotonic()-job['started']))
            connection=job['connection'];process=job['process']
            try:
                if connection.poll(remaining if wait else 0):result=connection.recv()
                elif time.monotonic()-job['started']>=self.deadline_seconds:result=dict(accepted=False,valid=False,reason='worker_deadline_exceeded')
                elif not process.is_alive():result=dict(accepted=False,valid=False,reason='worker_exited')
                else:return None
            except (EOFError,BrokenPipeError,OSError):result=dict(accepted=False,valid=False,reason='worker_exited')
        self._stop(job)
        self.future=None
        if self.pending:
            snapshot,following=self.pending;self.pending=None;self.submit(snapshot,following)
        return context,result

    def clear_pending(self):self.pending=None

    def cancel(self):
        self.pending=None
        if self.future:self._stop(self.future);self.future=None

    def close(self):
        self.cancel()


class JointYawCandidate:
    def __init__(self,result,reference_yaw=None):
        from .phone_heading import RawMotionHeading
        self.estimator=RawMotionHeading()
        self.estimator.candidate=dict(yaw=result['yaw_rad'],sigma=result['yaw_sigma_rad'],
            rows=result['observation_count'],train_end=result.get('anchor_utc_ms',0)/1000,
            candidate_source='joint_window')
        self.seed=dict(self.estimator.candidate)
        if reference_yaw is not None:self.estimator.reference_yaw=reference_yaw

    def push(self,values,antenna_at):
        for value in values:
            result=self.estimator.push(value,antenna_at)
            if result:
                # Failed holdout windows become past training. The refined
                # direction needs another disjoint future window, and is never
                # labelled as validation of the old joint position/yaw fit.
                return result
        return None


class PhoneAbsoluteRuntime:
    def __init__(self,worker_factory=AbsoluteQueue):
        self.worker_factory=worker_factory;self.worker=None
        self.observations=deque(maxlen=300);self.poses=deque(maxlen=5000)
        self.source=None;self.base_tag=None;self.generation=0
        self.last_submit=None;self.last_sequence=None;self.revision=0
        self.completed=deque(maxlen=4);self.last_result=None
        self.raw_motion=deque(maxlen=300);self.yaw_candidate=None;self.yaw_worker=None
        self.pending_position=None;self.pending_validated_yaw=None;self.pending_waiting_pose=False
        self.reason='等待原始卫星观测';self.accepted=0;self.rejected=0

    @staticmethod
    def base_identity(state):
        return (state.trial.identifier,state.odom_epoch,state.odom_continuity,
                state.phone_motion_enabled,state.trial.active,state.trial.rtk_enabled,
                bool(state.gnss_source and state.gnss_source[0] not in ('phone_continuity','phone_software_rtk','phone_absolute')))

    def owns_alignment(self,state,source=None):
        return bool((state.alignment or {}).get('heading_method')=='raw_absolute_livo_window'
            and self.base_tag==self.base_identity(state) and self.source
            and state.alignment.get('absolute_generation')==self.generation
            and tuple(state.alignment.get('anchor_source',[])[1:3])==self.source
            and (source is None or tuple(source[:2])==self.source))

    def identity(self,state):return (self.base_identity(state),self.source,self.generation)

    def sync(self,state):
        tag=self.base_identity(state)
        if tag!=self.base_tag:
            self.base_tag=tag;self.generation+=1
            self.observations.clear();self.poses.clear();self.source=None
            self.raw_motion.clear();self.yaw_candidate=None
            self.clear_pending_position()
            self.last_submit=None;self.last_sequence=None
            if self.worker:self.worker.cancel() if hasattr(self.worker,'cancel') else self.worker.clear_pending()
        return state.phone_motion_enabled and state.trial.active and state.trial.rtk_enabled and not tag[-1]

    def feed_pose(self,state):
        if not self.sync(state) or not state.odom:return
        t,p,_=state.odom[-1]
        if self.poses and t*1000-self.poses[-1]['utc_ms']<80:return
        self.poses.append(dict(utc_ms=t*1000,position=list(p),distance_m=state.odom_distance,
            epoch_id=str(state.odom_epoch)+':'+str(state.odom_continuity)))
        while self.poses and t*1000-self.poses[0]['utc_ms']>370000:self.poses.popleft()
        self.submit_due(state)

    def feed_motion(self,state,row):
        if not self.sync(state) or not isinstance(row,dict):return
        if row.get('source')!='phone_continuity' or row.get('fresh') is not True:return
        raw=row.get('raw_constraints')
        if not isinstance(raw,dict) or not number(raw.get('end_utc_ms')):return
        item=dict(source_id=row.get('source_id'),session_id=row.get('session_id'),raw=copy.deepcopy(raw))
        if self.raw_motion and raw['end_utc_ms']<=self.raw_motion[-1]['raw']['end_utc_ms']:return
        self.raw_motion.append(item)
        self.submit_yaw(state)

    def submit_yaw(self,state):
        candidate=self.yaw_candidate
        if not candidate:return
        from .phone_heading import constraints,HeadingWorker
        points={};values=[];poses=list(self.poses)
        for entry in self.raw_motion:
            if (entry['source_id'],entry['session_id'])!=self.source:continue
            raw=entry['raw'];parsed=constraints(raw)
            if parsed is None:continue
            # Every held-out factor must start after the absolute fit window.
            groups=[v for k,v in parsed.items() if k in ('phase','doppler')]
            if not groups or any(v['start']*1000<candidate['anchor_utc_ms'] for v in groups):continue
            if any(v['end']*1000>state.clock()*1000 for v in groups):continue
            for value in groups:
                for stamp in (value['start'],value['end']):
                    if stamp not in points:points[stamp]=self.historical_pose(poses,stamp*1000)
            values.append(raw)
        if not values:return
        if self.yaw_worker is None:self.yaw_worker=HeadingWorker()
        context=dict(tag=self.identity(state),candidate_id=id(candidate),candidate=candidate)
        self.yaw_worker.submit(candidate['estimator'],values[-60:],points,context)

    def poll_yaw(self,state,wait=False):
        if self.yaw_worker is None:return
        output=self.yaw_worker.poll(wait=wait)
        if output is None:return
        context,result,_=output
        if (not result or not context or context['tag']!=self.identity(state)
                or context['candidate_id']!=id(self.yaw_candidate)):return
        candidate=context['candidate'];old=state.alignment
        if (result.get('heading_candidate_source')=='raw_scalar_refined'
                or abs(math.atan2(math.sin(result['yaw']-candidate['yaw_rad']),math.cos(result['yaw']-candidate['yaw_rad'])))>1e-8):
            self.completed.append(dict(consumer_accepted=False,consumer_received_utc_ms=state.clock()*1000,
                reason='refined_heading_is_not_joint_endpoint_validation',
                consumer_heading_validation=dict(result)))
            self.reason='方向已重新估计，等待独立地理对准'
            self.yaw_candidate=None
            return
        if old is None:
            initial=candidate.get('initial_context')
            if initial and self.apply(state,initial,candidate['initial_result'],validated_yaw=result):
                self.completed.append(dict(candidate['initial_result'],consumer_accepted=True,
                    consumer_received_utc_ms=state.clock()*1000,consumer_yaw_future_validated=True,
                    consumer_heading_validation={k:v for k,v in result.items()
                        if k.startswith('validation_') or k.startswith('heading_')}))
            self.yaw_candidate=None;return
        if not state.trial.phone_translation.applies(state):return
        shift=abs(math.atan2(math.sin(candidate['yaw_rad']-old['yaw']),math.cos(candidate['yaw_rad']-old['yaw'])))
        if (result.get('validation_reference_loss_advantage',0.)<=9
                or shift<=2*result.get('validation_yaw_sigma_rad',math.pi)):
            old['raw_yaw_update_reason']='后续运动未证明新方向优于当前方向'
            self.yaw_candidate=None;return
        old_sigma=old.get('yaw_sigma_rad',math.radians(15))
        if not number(old_sigma) or old_sigma<=0:old_sigma=math.radians(15)
        if candidate['yaw_sigma_rad']>max(old_sigma,math.radians(3))*2:return
        state.trial.phone_translation.target['yaw_target_rad']=candidate['yaw_rad']
        old.update(raw_yaw_future_validated_at=result['heading_validated_at'],
            raw_yaw_future_error_rad=result['validation_yaw_error_rad'],
            raw_yaw_candidate_sigma_rad=candidate['yaw_sigma_rad'])
        self.yaw_candidate=None

    def feed(self,state,row):
        if not self.sync(state) or not isinstance(row,dict):return False
        if (type(row.get('schema_version')) is not int or row['schema_version']!=1
                or not number(row.get('utc_ms')) or not 0<=state.clock()-row['utc_ms']/1000<=10
                or row.get('fresh') is not True or not isinstance(row.get('signals'),list)
                or not 1<=len(row['signals'])<=32
                or not all(isinstance(row.get(k),str) and 1<=len(row[k])<=160 for k in ('source_id','session_id'))):return False
        source=(row['source_id'],row['session_id'])
        if self.source!=source:
            self.source=source;self.generation+=1;self.observations.clear();self.last_submit=None;self.last_sequence=None
            self.yaw_candidate=None
            self.clear_pending_position()
            if self.worker:self.worker.cancel() if hasattr(self.worker,'cancel') else self.worker.clear_pending()
        if self.observations and row['utc_ms']<=self.observations[-1]['utc_ms']:return False
        self.observations.append(copy.deepcopy(row));self.last_sequence=row.get('sequence')
        while self.observations and row['utc_ms']-self.observations[0]['utc_ms']>240000:self.observations.popleft()
        self.reason='正在积累原始卫星与雷达观测'
        self.submit_due(state)
        return True

    def submit_due(self,state):
        if len(self.observations)<20 or not self.poses:return
        last=self.observations[-1]['utc_ms']
        if (last-self.observations[0]['utc_ms']<20000 or self.poses[-1]['utc_ms']<last
                or (self.last_submit is not None and last-self.last_submit<30000)):return
        start=max(self.observations[0]['utc_ms'],self.poses[0]['utc_ms'])
        observations=[r for r in self.observations if r['utc_ms']>=start]
        if len(observations)<20:return
        start=observations[0]['utc_ms']
        poses=[p for p in self.poses if p['utc_ms']>=start-250 and p['utc_ms']<=last+250]
        if not poses or poses[0]['utc_ms']>start:return
        snapshot=dict(source_id=self.source[0],session_id=self.source[1],
            continuity_id=str(self.base_tag[:3]),generation=self.generation,
            available_utc_ms=state.clock()*1000,observations=observations,poses=poses,
            max_window_seconds=240,lever_arm_m=[0.,0.,0.],lever_arm_uncertainty_m=1.)
        context=dict(tag=self.identity(state),poses=poses,submitted_utc_ms=state.clock()*1000,
                     input_end_utc_ms=last,snapshot_identity={k:snapshot[k] for k in ('source_id','session_id','continuity_id','generation')})
        if self.worker is None:self.worker=self.worker_factory()
        self.worker.submit(snapshot,context);self.last_submit=last
        self.reason='正在核对原始卫星与雷达的地理位置'

    @staticmethod
    def historical_pose(poses,utc_ms):
        import bisect
        i=bisect.bisect_left([p['utc_ms'] for p in poses],utc_ms)
        if i<len(poses) and abs(poses[i]['utc_ms']-utc_ms)<.01:return poses[i]['position']
        if i==0 or i==len(poses):return None
        a,b=poses[i-1],poses[i]
        if a['epoch_id']!=b['epoch_id'] or not 0<b['utc_ms']-a['utc_ms']<=250:return None
        f=(utc_ms-a['utc_ms'])/(b['utc_ms']-a['utc_ms'])
        return [x+f*(y-x) for x,y in zip(a['position'],b['position'])]

    @staticmethod
    def historical_distance(poses,utc_ms):
        """Measured cumulative LiVO distance, independent of map corrections."""
        import bisect
        i=bisect.bisect_left([p['utc_ms'] for p in poses],utc_ms)
        if i<len(poses) and abs(poses[i]['utc_ms']-utc_ms)<.01:
            value=poses[i].get('distance_m')
            return value if number(value) and value>=0 else None
        if i==0 or i==len(poses):return None
        a,b=poses[i-1],poses[i];x,y=a.get('distance_m'),b.get('distance_m')
        if (a['epoch_id']!=b['epoch_id'] or not 0<b['utc_ms']-a['utc_ms']<=250
                or not number(x) or not number(y) or not 0<=x<=y):return None
        return x+(y-x)*(utc_ms-a['utc_ms'])/(b['utc_ms']-a['utc_ms'])

    def clear_pending_position(self):
        self.pending_position=None;self.pending_validated_yaw=None;self.pending_waiting_pose=False

    def defer_position(self,context,result,validated_yaw=None,waiting_pose=False):
        # Retain only the newest qualified endpoint, never a backlog of updates.
        # Repeated polls keep its original measurement time and 120-second limit.
        if (self.pending_position and self.pending_position[1]['anchor_utc_ms']>result['anchor_utc_ms']):return False
        self.pending_position=(context,result)
        self.pending_validated_yaw=validated_yaw
        self.pending_waiting_pose=waiting_pose
        return True

    def poll(self,state,wait=False):
        self.sync(state)
        self.poll_yaw(state,wait=wait)
        if self.pending_position:
            context,result=self.pending_position
            if context['tag']!=self.identity(state) or state.clock()-result['anchor_utc_ms']/1000>120:
                self.clear_pending_position()
            elif (state.odom and -.05<=state.clock()-state.odom[-1][0]<=.5
                    and (state.alignment or self.pending_waiting_pose or self.pending_validated_yaw is not None)):
                validation=self.pending_validated_yaw
                # Clear before retry: only a still-valid, temporarily blocked
                # result may put itself back. Permanent rejection is not retried.
                self.clear_pending_position()
                if self.apply(state,context,result,validated_yaw=validation):
                    record=dict(result,consumer_accepted=True,consumer_received_utc_ms=state.clock()*1000,
                        consumer_retried=True)
                    if validation is not None:
                        record.update(consumer_yaw_future_validated=True,consumer_heading_validation={
                            k:v for k,v in validation.items() if k.startswith('validation_') or k.startswith('heading_')})
                    self.completed.append(record);self.last_result=record;self.accepted+=1
                    self.reason='原始卫星与雷达联合位置已更新'
        if self.worker is None:return
        output=self.worker.poll(wait=wait)
        if output is None:return
        context,result=output
        accepted=self.apply(state,context,result)
        pending=bool(self.pending_position and self.pending_position[0] is context and self.pending_position[1] is result)
        record=dict(result,consumer_accepted=accepted,consumer_received_utc_ms=state.clock()*1000,
            consumer_pending=pending)
        self.completed.append(record);self.last_result=record
        if accepted:self.accepted+=1;self.reason='原始卫星与雷达联合位置已更新'
        elif pending:self.reason='联合位置已保留，等待运动跟踪与方向核验'
        else:self.rejected+=1;self.reason='联合位置仍在核验，雷达继续跟踪'

    def apply(self,state,context,result,validated_yaw=None):
        if (not isinstance(result,dict) or not context or context.get('tag')!=self.identity(state)
                or not self.sync(state) or result.get('accepted_position',result.get('accepted')) is not True or result.get('valid') is not True
                or any(result.get(k)!=v for k,v in context['snapshot_identity'].items())
                or result.get('anchor_point')!='livo_reference'
                or result.get('absolute_accuracy_verified') is not False
                or not state.odom):return False
        t=result.get('anchor_utc_ms');sigmas=result.get('position_sigma_enu_m')
        if (not number(t) or not 0<=state.clock()-t/1000<=120 or t>context['input_end_utc_ms']+1
                or not isinstance(sigmas,list) or len(sigmas)!=3 or any(not number(s) or s<=0 for s in sigmas)
                or not all(number(result.get(k)) for k in ('anchor_latitude','anchor_longitude','anchor_altitude_ellipsoid_m'))):return False
        if (abs(result['anchor_latitude'])>=85 or abs(result['anchor_longitude'])>180
                or not -15000<=result['anchor_altitude_ellipsoid_m']<=100000
                or max(sigmas)>100000 or result.get('method')!='raw_code_phase_livo_window_v1'
                or result.get('confidence_calibrated') is not False
                or not isinstance(result.get('rank'),dict) or result['rank'].get('position')!=3
                or not number(result.get('epoch_count')) or result['epoch_count']<80
                or not number(result.get('observation_count')) or result['observation_count']<80):return False
        for key,limit in (('loo_max_horizontal_m',5.),('seed_max_horizontal_m',2.),('leaveout_cold_max_horizontal_m',2.)):
            if not number(result.get(key)) or not 0<=result[key]<=limit:return False
        model=result.get('model_position_sigma_enu_m')
        if (not isinstance(model,list) or len(model)!=3 or any(not number(v) or v<=0 for v in model)
                or math.hypot(*model[:2])>15):return False
        p=self.historical_pose(context['poses'],t)
        if p is None:return False
        phone=state.phone_motion or {}
        if phone and (phone.get('source_id'),phone.get('session_id'))!=self.source:return False
        if state.gnss_source and state.gnss_source[0] not in ('phone_continuity','phone_software_rtk','phone_absolute'):return False
        stable_yaw=(result.get('yaw_candidate_stable') is True and result['rank'].get('yaw')
            and number(result.get('yaw_rad')) and number(result.get('yaw_sigma_rad'))
            and 0<result['yaw_sigma_rad']<=math.radians(15)
            and all(number(result.get(k)) and 0<=result[k]<=limit for k,limit in
                (('loo_max_yaw_rad',math.radians(8)),('seed_max_yaw_rad',math.radians(5)))))
        if stable_yaw:
            stable_yaw=(number(result.get('leaveout_cold_max_yaw_rad'))
                and 0<=result['leaveout_cold_max_yaw_rad']<=math.radians(5))
        if not state.alignment and validated_yaw is not None:
            if not stable_yaw:return False
            if (not isinstance(validated_yaw,dict) or not number(validated_yaw.get('yaw'))
                    or abs(math.atan2(math.sin(validated_yaw['yaw']-result['yaw_rad']),math.cos(validated_yaw['yaw']-result['yaw_rad'])))>1e-8
                    or not number(validated_yaw.get('heading_validated_at'))
                    or not t/1000<validated_yaw['heading_validated_at']<=state.clock()
                    or not number(validated_yaw.get('validation_normalized_loss'))
                    or not 0<=validated_yaw['validation_normalized_loss']<5):return False
        pose_age=state.clock()-state.odom[-1][0]
        if pose_age>.5:
            self.defer_position(context,result,validated_yaw=validated_yaw,waiting_pose=True)
            return False
        if pose_age<-.05:return False
        if not state.alignment and validated_yaw is None:
            retained=self.defer_position(context,result)
            if retained and stable_yaw:
                self.yaw_candidate=dict(result,estimator=JointYawCandidate(result),
                    initial_context=context,initial_result=result)
                self.submit_yaw(state)
            return False
        from .outdoor import GeoOrigin
        original_origin=state.origin;original_source=state.gnss_source;original_conflict=state.alignment_conflict
        origin=state.origin or GeoOrigin(result['anchor_longitude'],result['anchor_latitude'],result['anchor_altitude_ellipsoid_m'])
        q=origin.enu(result['anchor_longitude'],result['anchor_latitude'],result['anchor_altitude_ellipsoid_m'])
        old=state.alignment
        yaw=old['yaw'] if old else result['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
        direction_sigma=(old or {}).get('yaw_sigma_rad') if old else result.get('yaw_sigma_rad')
        if not number(direction_sigma) or direction_sigma<=0:direction_sigma=math.radians(15)
        sigma=math.hypot(sigmas[0],sigmas[1])
        if old:
            # Keep an established direction until the joint estimator's direction
            # agrees; translation alone must never rotate the scene at a turn.
            yaw=old['yaw'];c,s=math.cos(yaw),math.sin(yaw)
        state.origin=origin
        state.gnss_source=('phone_continuity',*self.source,phone.get('continuity_id','raw_absolute'),None)
        candidate=dict(old or {},yaw=yaw,tx=old['tx'] if old else q[0]-c*p[0]+s*p[1],
            ty=old['ty'] if old else q[1]-s*p[0]-c*p[1],tz=0.,stamp=t/1000,
            heading_method='raw_absolute_livo_window',estimate_source='phone_continuity',
            precision_source='raw_absolute_livo_window',experimental=True,absolute_accuracy_verified=False,
            geographic_accuracy='approximate',absolute_horizontal_sigma_m=max(sigma,(old or {}).get('absolute_horizontal_sigma_m',sigma)),
            absolute_anchor_source='raw_window',absolute_anchor_stamp=t/1000,
            yaw_sigma_rad=direction_sigma,absolute_generation=self.generation,
            vertical_alignment_valid=False,receiver_precision_available=False)
        if old is None:
            candidate['raw_yaw_future_validated_at']=validated_yaw['heading_validated_at']
            if number(validated_yaw.get('validation_yaw_error_rad')):
                candidate['raw_yaw_future_error_rad']=validated_yaw['validation_yaw_error_rad']
        state.alignment=candidate;state.alignment_conflict=False;state._anchor_odometry()
        self.revision+=1
        value=dict(source='raw_window',revision=self.revision,stamp=t/1000,
            longitude=result['anchor_longitude'],latitude=result['anchor_latitude'],
            altitude_ellipsoid_m=result['anchor_altitude_ellipsoid_m'],horizontal_sigma_m=sigma)
        if not state.trial.phone_translation.accept_reference(state,value,p):
            state.alignment=old;state.origin=original_origin;state.gnss_source=original_source
            state.alignment_conflict=original_conflict;return False
        if old is None:state.previous_output=None;state.converted=None;state.pose_epoch+=1
        elif stable_yaw:
            self.yaw_candidate=dict(result,estimator=JointYawCandidate(result,old['yaw']))
            self.submit_yaw(state)
        else:self.yaw_candidate=None
        state.trial.reason='原始卫星与雷达联合定位已核验，仍为近似位置'
        state.alignment_reason=state.trial.reason
        state.trial.event(state,'raw_absolute_window',measurement_stamp=t/1000,position_sigma_m=sigma,
            loo_max_horizontal_m=result.get('loo_max_horizontal_m'),rank=result.get('rank'))
        if self.pending_position and self.pending_position[1]['anchor_utc_ms']<=t:self.clear_pending_position()
        return True

    def snapshot(self):
        return dict(reason=self.reason,observations=len(self.observations),accepted=self.accepted,
            rejected=self.rejected,running=bool(self.worker and self.worker.future),generation=self.generation)

    def close(self):
        if self.worker:self.worker.close()
        if self.yaw_worker:self.yaw_worker.close()

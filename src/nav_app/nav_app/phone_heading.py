"""Causal heading from raw range/range-rate changes and independent LiVO.

No integrated phone trajectory or absolute position is a heading observation.
Each epoch has its own unknown common receiver clock term. A candidate uses
past edges, then waits for disjoint future edges before it can guide anyone.
"""
from collections import deque
import math
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import time


def finite(v):
    return type(v) in (int,float) and math.isfinite(v)


def constraints(value):
    if (not isinstance(value,dict) or value.get('schema_version')!=1
            or value.get('coordinate_frame')!='phone_gnss_enu'):
        return None
    for k in ('start_utc_ms','end_utc_ms','dt_s'):
        if not finite(value.get(k)):return None
    if not .1<value['dt_s']<1.5:return None
    if abs((value['end_utc_ms']-value['start_utc_ms'])/1000-value['dt_s'])>.002:return None
    result={k:value[k] for k in ('start_utc_ms','end_utc_ms','dt_s')}
    for kind,bkey,skey in (('phase','b_m','sigma_m'),('doppler','b_mps','sigma_mps')):
        f=value.get(kind)
        if not isinstance(f,dict):continue
        A,b,sigma=(f.get(k) for k in ('A_enu',bkey,skey))
        if (not isinstance(A,list) or not isinstance(b,list) or not isinstance(sigma,list)
                or not 2<=len(A)<=96 or len(A)!=len(b) or len(A)!=len(sigma)):
            continue
        if any(not isinstance(a,list) or len(a)!=3 or not all(finite(x) for x in a)
               or not .2<math.sqrt(sum(x*x for x in a))<1.5 for a in A):continue
        if not all(finite(x) and abs(x)<1e7 for x in b):continue
        if not all(finite(x) and 1e-5<x<1e4 for x in sigma):continue
        if kind=='phase':
            start,end=f.get('start_utc_ms'),f.get('end_utc_ms')
            if (not finite(start) or not finite(end) or not .1<(end-start)/1000<1.5
                    or abs(end-value['end_utc_ms'])>1.):continue
        else:
            end=f.get('utc_ms')
            if not finite(end) or abs(end-value['end_utc_ms'])>1.:continue
            start=value['start_utc_ms']
        result[kind]=dict(A=A,b=b,sigma=sigma,start=start/1000,end=end/1000)
    return result if 'phase' in result or 'doppler' in result else None


def angle(x):return math.atan2(math.sin(x),math.cos(x))


class RawMotionHeading:
    # One angle, 99% chi-square reference. This is a model consistency gate,
    # not a calibrated field probability: dispersion, unique-peak, motion and
    # disjoint-future checks remain mandatory and are recorded separately.
    PROFILE_LIMIT=6.634896601

    def __init__(self):
        self.train=deque(maxlen=60)
        self.check=[]
        self.candidate=None
        self.result=None
        self.last_stamp=None
        self.reason='等待手机原始运动与雷达共同观测'
        self.accepted_edges=0
        self.last_validation=None

    @staticmethod
    def _pack(edges):
        arrays=[];group=0
        for edge in edges:
            for A,b,sigma,v in edge['factors']:
                n=len(b)
                arrays.append((A,np.asarray(b),np.asarray(sigma),np.tile(v,(n,1)),np.full(n,group,int)))
                group+=1
        return tuple(np.concatenate([a[i] for a in arrays]) for i in range(5))

    @staticmethod
    def _cost(theta,packed,info=False):
        A,b,sigma,v,groups=packed
        c,s=math.cos(theta),math.sin(theta)
        x=c*v[:,0]-s*v[:,1];y=s*v[:,0]+c*v[:,1]
        pred=A[:,0]*x+A[:,1]*y+A[:,2]*v[:,2]
        w=1/sigma**2
        for _ in range(4):
            mass=np.bincount(groups,weights=w)
            clock=np.bincount(groups,weights=w*(b-pred))/mass
            residual=(pred+clock[groups]-b)/sigma
            w=1/(1+(residual/3)**2)/sigma**2
        loss=float(np.sum(9*np.log1p((residual/3)**2)))
        rows=len(b)-len(mass)
        if not info:return loss
        derivative=-A[:,0]*y+A[:,1]*x
        mean=np.bincount(groups,weights=w*derivative)/np.bincount(groups,weights=w)
        information=float(np.sum(w*(derivative-mean[groups])**2))
        return loss,rows,information

    @classmethod
    def _fit(cls,edges):
        packed=cls._pack(edges)
        angles=np.linspace(-math.pi,math.pi,72,endpoint=False)
        costs=[cls._cost(a,packed) for a in angles]
        step=2*math.pi/72
        if max(costs)-min(costs)<1e-9:
            return dict(yaw=0.,sigma=float('inf'),normalized_loss=min(costs),peak_gap=0.,rows=0)
        def refine(index):
            lo,hi=angles[index]-step,angles[index]+step
            ratio=(math.sqrt(5)-1)/2
            a=hi-ratio*(hi-lo);b=lo+ratio*(hi-lo)
            fa,fb=cls._cost(a,packed),cls._cost(b,packed)
            for _ in range(20):
                if fa<fb:hi,b,fb=b,a,fa;a=hi-ratio*(hi-lo);fa=cls._cost(a,packed)
                else:lo,a,fa=a,b,fb;b=lo+ratio*(hi-lo);fb=cls._cost(b,packed)
            theta=angle((lo+hi)/2)
            return theta,cls._cost(theta,packed)
        # Refine every circular local minimum. A coarse grid can hide an
        # equally good mirror solution between samples; it is not evidence of
        # unique heading, even if another time window repeats that ambiguity.
        minima=[refine(i) for i in range(len(angles))
                if costs[i]<=costs[(i-1)%len(costs)] and costs[i]<=costs[(i+1)%len(costs)]]
        yaw,_=min(minima,key=lambda item:item[1])
        loss,rows,info=cls._cost(yaw,packed,True)
        distant=[cost for a,cost in minima if abs(angle(a-yaw))>math.radians(30)]
        distant.extend(cls._cost(yaw+sign*math.radians(30),packed) for sign in (-1,1))
        return dict(yaw=yaw,sigma=math.sqrt(max(1,loss/max(1,rows-1))/max(info,1e-12)),
                    normalized_loss=loss/max(1,rows),peak_gap=min(distant)-loss,rows=rows)

    def push(self,value,antenna_at):
        parsed=constraints(value)
        if not parsed:return None
        stamp=parsed['end_utc_ms']/1000
        if self.last_stamp is not None and stamp<=self.last_stamp:return None
        self.last_stamp=stamp
        factors=[];distance=0.;start=stamp
        for kind in ('phase','doppler'):
            row=parsed.get(kind)
            if not row:continue
            p,q=antenna_at(row['start']),antenna_at(row['end'])
            if p is None or q is None:continue
            delta=np.asarray(q)-np.asarray(p);dt=row['end']-row['start']
            length=float(np.linalg.norm(delta[:2]))
            if length<.10 or np.linalg.norm(delta)>6*dt:continue
            distance=max(distance,length)
            start=min(start,row['start'])
            factors.append((np.asarray(row['A']),row['b'],row['sigma'],delta if kind=='phase' else delta/dt))
        if not factors:return None
        edge=dict(start=start,stamp=stamp,distance=distance,factors=factors)
        self.accepted_edges+=1
        # Only finite, independently observed short edges enter. A time gap
        # never contributes an interpolated phone displacement.
        if self.candidate is not None:
            # A phase edge may cover a different interval from the outer row.
            # Validation must not reuse any part of a training interval.
            if start<self.candidate.get('train_end',float('-inf')):
                return None
            self.check.append(edge)
            if len(self.check)<6 or sum(e['distance'] for e in self.check)<3.:
                self.reason='方向候选已形成，正在用后续行走独立验证';return None
            check=self._fit(self.check)
            packed=self._pack(self.check)
            fixed_loss,rows,_=self._cost(self.candidate['yaw'],packed,True)
            reference_loss=self._cost(self.reference_yaw,packed) if hasattr(self,'reference_yaw') else None
            separation=abs(angle(check['yaw']-self.candidate['yaw']))
            optimal_loss=self._cost(check['yaw'],packed)
            dispersion=max(1.,optimal_loss/max(1,rows-1))
            profile_delta=max(0.,fixed_loss-optimal_loss)/dispersion
            valid=(check['sigma']<math.radians(15) and check['peak_gap']>6
                and check['normalized_loss']<5 and fixed_loss/max(1,rows)<5
                and separation<math.radians(10) and profile_delta<=self.PROFILE_LIMIT)
            check_start=min(e.get('start',e['stamp']) for e in self.check)
            check_distance=sum(e['distance'] for e in self.check)
            check_edges=len(self.check)
            self.last_validation=dict(accepted=valid,candidate_yaw=self.candidate['yaw'],
                optimal_yaw=check['yaw'],profile_delta=profile_delta,profile_limit=self.PROFILE_LIMIT,
                dispersion=dispersion,rows=rows,start=check_start,end=stamp,
                candidate_train_end=self.candidate.get('train_end'),motion_baseline_m=check_distance)
            self.train.extend(self.check);self.check=[]
            old=self.candidate;self.candidate=None
            if valid:
                self.result=dict(yaw=old['yaw'],yaw_sigma_rad=old['sigma'],
                    validation_yaw_error_rad=separation,validation_normalized_loss=fixed_loss/max(1,rows),
                    validation_optimal_yaw=check['yaw'],validation_yaw_sigma_rad=check['sigma'],
                    validation_peak_gap=check['peak_gap'],validation_edge_count=check_edges,
                    validation_profile_delta=profile_delta,validation_profile_limit=self.PROFILE_LIMIT,
                    validation_dispersion=dispersion,
                    validation_start_utc_s=check_start,validation_end_utc_s=stamp,
                    validation_motion_baseline_m=check_distance,
                    heading_train_rows=old['rows'],heading_validation_rows=rows,
                    heading_validated_at=stamp,heading_method='raw_scalar_livo_increment')
                self.result['heading_train_end_utc_s']=old.get('train_end')
                self.result['heading_candidate_source']=old.get('candidate_source','raw_scalar_refined')
                if hasattr(self,'reference_yaw'):
                    # The check subset is disjoint from both fitted candidates.
                    # Keep explicit evidence before accepting a direction change.
                    self.result.update(validation_optimal_yaw=check['yaw'],validation_yaw_sigma_rad=check['sigma'],
                        validation_reference_loss_advantage=reference_loss-fixed_loss)
                self.reason='原始卫星运动与雷达方向通过后续行走验证'
                return dict(self.result)
            self.reason='后续运动与方向候选不一致，继续核对'
        else:self.train.append(edge)
        if len(self.train)>=8 and sum(e['distance'] for e in self.train)>=5.:
            candidate=self._fit(self.train)
            if (candidate['sigma']<math.radians(10) and candidate['peak_gap']>9
                    and candidate['normalized_loss']<5):
                self.candidate=candidate
                self.candidate['train_end']=stamp
                self.reason='方向候选已形成，等待后续行走验证'
        return None


class HeadingWorker:
    """One running solve plus one latest pending request; never waits in UI/ROS."""
    def __init__(self):
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='phone-heading')
        self.future=None
        self.pending=None
        self.durations=[]

    @staticmethod
    def _run(estimator,value,points,context):
        begin=time.perf_counter()
        result=estimator.push(value,lambda stamp:points.get(stamp))
        return context,result,(time.perf_counter()-begin)*1000

    def submit(self,estimator,value,points,context):
        request=(estimator,value,points,context)
        if self.future is None:self.future=self.executor.submit(self._run,*request)
        else:self.pending=request

    def poll(self,wait=False):
        if self.future is None or (not wait and not self.future.done()):return None
        try:
            context,result,duration=self.future.result()
            self.durations.append(duration)
            if len(self.durations)>1000:del self.durations[:-1000]
        except Exception:
            context,result,duration=None,None,None
        self.future=None
        if self.pending:
            request=self.pending;self.pending=None
            self.future=self.executor.submit(self._run,*request)
        return context,result,duration

    def close(self):
        self.pending=None
        self.executor.shutdown(wait=False,cancel_futures=True)

"""Independent, explicitly evidenced geographic translation corrections.

No revision in the wire means no new correction path. Native RTK quality and
the established heading estimator remain independent of these coarse anchors.
"""
import math


def number(value):
    return type(value) in (int,float) and math.isfinite(value)


def anchor(row, now):
    value=row.get('absolute_anchor')
    if (not isinstance(value,dict) or type(value.get('schema_version')) is not int or value['schema_version']!=1
            or value.get('status')!='consistent'
            or value.get('source') not in ('system_gps_window','native_fixed_window','raw_window')
            or value.get('absolute_accuracy_verified') is not False
            or type(value.get('revision')) is not int or not 1<=value['revision']<2**53
            or row.get('status')=='unavailable'
            or any(k in value for k in ('quality','fix_quality','source_id','session_id'))):return None
    keys=('utc_ms','measurement_utc_ms','latitude','longitude','altitude_ellipsoid_m','horizontal_sigma_m')
    if any(not number(value.get(k)) for k in keys):return None
    t=value['utc_ms']/1000
    if (not number(row.get('utc_ms')) or abs(value['utc_ms']-row['utc_ms'])>1.
            or not 1<=value['utc_ms']<2**53 or not 1<=value['measurement_utc_ms']<2**53
            or not 0<=now-t<=10 or not 0<=value['utc_ms']-value['measurement_utc_ms']<=2000
            or abs(value['latitude'])>=85 or abs(value['longitude'])>180
            or not -15000<=value['altitude_ellipsoid_m']<=100000
            or not 0<value['horizontal_sigma_m']<=100000):return None
    if value['source']=='native_fixed_window' and (type(value.get('native_quality')) is not int or value['native_quality']!=1):return None
    return dict(value,stamp=t)


class PhoneTranslation:
    def __init__(self):
        self.target=None;self.last_tick=None;self.last_revision=0
        self.tag=None;self.total_m=0.;self.last_correction_m=0.
        self.arrival_confirmations=3
        self.last_arrival_stamp=None
        self.started_stamp=None

    @staticmethod
    def identity(state):
        if (state.alignment or {}).get('heading_method')=='raw_absolute_livo_window':
            runtime=state.phone_absolute
            return (state.trial.identifier,state.odom_epoch,state.odom_continuity,
                ('phone_absolute',runtime.source),runtime.generation)
        return (state.trial.identifier,state.odom_epoch,state.odom_continuity,
            state.gnss_source,state.phone_alignment_generation)

    def clear(self,state=None):
        # Corrections belong to one source/odometry generation. A retained map
        # transform is not evidence that the next session walked that distance.
        had_correction=self.total_m>0 or self.arrival_confirmations<3
        self.target=None;self.last_tick=None;self.last_revision=0;self.tag=None
        self.total_m=0.;self.last_correction_m=0.
        self.last_arrival_stamp=None
        self.started_stamp=None
        self.arrival_confirmations=0 if had_correction else 3
        if state is not None and state.alignment:
            state.alignment.pop('geographic_correction_total_m',None)
            state.alignment.pop('geographic_correction_remaining_m',None)

    def applies(self,state):
        return bool(self.target and self.tag==self.identity(state))

    def feed(self,state,row):
        # A finite low-weight GPS posterior or a FIX label alone has not
        # established a better absolute reference. Keep these wire candidates
        # recorded; only the independent window validator can install a target.
        return False

    def accept_reference(self,state,value,p):
        """Install an internally verified reference at its original LiVO pose.

        Called after the independent solver's full window/LOO/identity checks,
        never merely because an optional phone payload declares consistency.
        """
        a=state.alignment
        if (not state.trial.active or not state.trial.rtk_enabled or not state.phone_motion_enabled
                or not a or state.alignment_conflict or not state.origin
                or a.get('heading_method') not in ('raw_scalar_livo_increment','raw_absolute_livo_window')
                or a.get('odom_continuity')!=state.odom_continuity):return False
        if (not isinstance(value,dict) or not isinstance(p,(list,tuple)) or len(p)!=3
                or any(not number(v) for v in p) or not number(value.get('stamp'))
                or not 0<=state.clock()-value['stamp']<=120):return False
        if (value.get('source') not in ('system_gps_window','native_fixed_window','raw_window')
                or type(value.get('revision')) is not int or not 1<=value['revision']<2**53
                or any(not number(value.get(k)) for k in ('latitude','longitude','altitude_ellipsoid_m','horizontal_sigma_m'))
                or abs(value['latitude'])>=85 or abs(value['longitude'])>180
                or not -15000<=value['altitude_ellipsoid_m']<=100000
                or not 0<value['horizontal_sigma_m']<=100000
                or not state.odom or not -.05<=state.clock()-state.odom[-1][0]<=.5):return False
        tag=self.identity(state)
        if tag==self.tag and value['revision']<=self.last_revision:return False
        q=state.origin.enu(value['longitude'],value['latitude'],value['altitude_ellipsoid_m'])
        if self.target and value['stamp']<=self.target['stamp']:return False
        self.target=dict(value,odom=list(p),enu=list(q),
            previous_sigma=a.get('absolute_anchor_horizontal_sigma_m',a.get('absolute_horizontal_sigma_m',value['horizontal_sigma_m'])))
        if self.tag!=tag or self.started_stamp is None:self.started_stamp=state.odom[-1][0]
        self.tag=tag;self.last_revision=value['revision']
        if self.last_tick is None:self.last_tick=state.odom[-1][0]
        return True

    def tick(self,state):
        if not self.target:
            if (state.odom and -.05<=state.clock()-state.odom[-1][0]<=.5
                    and (self.last_arrival_stamp is None or state.odom[-1][0]>self.last_arrival_stamp)):
                self.arrival_confirmations=min(3,self.arrival_confirmations+1)
                self.last_arrival_stamp=state.odom[-1][0]
            return
        if (not self.applies(state) or not state.trial.active or not state.trial.rtk_enabled
                or not state.phone_motion_enabled or not state.alignment or state.alignment_conflict):
            self.clear(state);return
        stamp=state.odom[-1][0];previous=self.last_tick;self.last_tick=stamp
        if previous is None or not 0<stamp-previous<=.5 or not -.05<=state.clock()-stamp<=.5:return
        self.last_arrival_stamp=stamp
        requested=self.target.get('yaw_target_rad')
        if number(requested):
            a=state.alignment;p=state.odom[-1][1];c,s=math.cos(a['yaw']),math.sin(a['yaw'])
            current=(c*p[0]-s*p[1]+a['tx'],s*p[0]+c*p[1]+a['ty'])
            change=(requested-a['yaw']+math.pi)%(2*math.pi)-math.pi
            limit=math.radians(.5)*(stamp-previous)
            a['yaw']+=max(-limit,min(limit,change));c,s=math.cos(a['yaw']),math.sin(a['yaw'])
            a['tx']=current[0]-c*p[0]+s*p[1];a['ty']=current[1]-s*p[0]-c*p[1]
            if abs(change)<=limit and number(a.get('raw_yaw_candidate_sigma_rad')):
                a['yaw_sigma_rad']=a['raw_yaw_candidate_sigma_rad']
        a=state.alignment;t=self.target;c,s=math.cos(a['yaw']),math.sin(a['yaw']);p=t['odom']
        target=(t['enu'][0]-c*p[0]+s*p[1],t['enu'][1]-s*p[0]-c*p[1])
        delta=(target[0]-a['tx'],target[1]-a['ty']);size=math.hypot(*delta)
        rate=min(1.,.3*(stamp-previous)/max(size,1e-12))
        self.last_correction_m=size*rate
        self.arrival_confirmations=(0 if size>.25 or self.last_correction_m>.02
            else min(3,self.arrival_confirmations+1))
        if self.last_correction_m>1e-9:
            a['tx']+=delta[0]*rate;a['ty']+=delta[1]*rate;self.total_m+=self.last_correction_m
            state.trial.last_correction_m=self.last_correction_m
        remaining=max(0.,size-self.last_correction_m)
        sigma=t['horizontal_sigma_m'] if remaining<=.25 else max(t['horizontal_sigma_m'],t['previous_sigma'])
        a.update(absolute_anchor_source=t['source'],absolute_anchor_revision=t['revision'],
            absolute_anchor_stamp=t['stamp'],absolute_horizontal_sigma_m=sigma,
            absolute_anchor_horizontal_sigma_m=t['horizontal_sigma_m'],
            absolute_accuracy_verified=False,geographic_accuracy='approximate',
            geographic_correction_total_m=self.total_m,geographic_correction_remaining_m=remaining)

    def arrival_ready(self):
        return self.arrival_confirmations>=3

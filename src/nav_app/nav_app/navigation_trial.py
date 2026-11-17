"""Live navigation/mapping experiment. No synthetic sensor inputs or route snapping."""
from collections import deque
import copy
import math
import statistics
import uuid
from .alignment_samples import StableAlignmentSamples


def transform(a, p):
    c, s = math.cos(a['yaw']), math.sin(a['yaw'])
    return [c*p[0]-s*p[1]+a['tx'], s*p[0]+c*p[1]+a['ty']]


def inverse(a, p):
    c, s = math.cos(a['yaw']), math.sin(a['yaw'])
    x, y = p[0]-a['tx'], p[1]-a['ty']
    return [c*x+s*y, -s*x+c*y]


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def trajectory_alignment(pairs, weights=None):
    """Robust planar rigid fit with the newest three pairs held out.

    Thresholds describe this experiment's trajectory agreement, not receiver
    covariance or an absolute accuracy certificate. Scale is never adjusted.
    """
    from .outdoor import fit_alignment
    if len(pairs) < 15:
        return None
    weights=list(weights) if weights is not None else [1.]*len(pairs)
    if len(weights)!=len(pairs) or any(not math.isfinite(w) or w<=0 for w in weights):return None
    train, check = list(pairs)[:-3], list(pairs)[-3:]
    stride = max(1, len(train)//10)
    candidates = train[::stride]
    best = [];best_mass=0.
    for i, (p, q) in enumerate(candidates):
        for r, t in candidates[i+1:]:
            lp, gp = math.dist(p[:2], r[:2]), math.dist(q[:2], t[:2])
            if lp < 8 or not .8 < gp/lp < 1.2:
                continue
            yaw = angle(math.atan2(t[1]-q[1], t[0]-q[0])-math.atan2(r[1]-p[1], r[0]-p[0]))
            a = dict(yaw=yaw, tx=0., ty=0.)
            x, y = transform(a, p)
            a.update(tx=q[0]-x, ty=q[1]-y)
            inliers = [j for j,(u,v) in enumerate(train) if math.dist(transform(a,u),v[:2]) <= 1.5]
            mass=sum(weights[j] for j in inliers)
            if mass > best_mass:
                best = inliers;best_mass=mass
    if len(best) < max(10, math.ceil(.75*len(train))):
        return None
    result = fit_alignment([train[j] for j in best], minimum_spread=5., maximum_residual=1., maximum_relative=.1,
                           weights=[weights[j] for j in best])
    if not result:
        return None
    held_out = [math.dist(transform(result,p),q[:2]) for p,q in check]
    if max(held_out) > 2.:
        return None
    return dict(result, validation_error_m=max(held_out), inliers=len(best),
                observations=len(pairs), precision_source='trajectory_agreement', experimental=True)


class LocalMapProjection:
    """A bounded local BD09/ENU approximation fitted to official conversions."""
    radius_m = 1000.

    def __init__(self, origin, converted):
        from .outdoor import coordinate
        if len(converted) != 5 or not all(coordinate(p) for p in converted):
            raise ValueError('地图坐标转换结果无效')
        self.origin = tuple(origin)
        self.center = list(converted[0])
        self.east = [(converted[1][i]-converted[2][i])/400 for i in (0,1)]
        self.north = [(converted[3][i]-converted[4][i])/400 for i in (0,1)]
        self.det = self.east[0]*self.north[1]-self.north[0]*self.east[1]
        if not 1e-12 < self.det < 1e-8:
            raise ValueError('地图局部坐标转换异常')
        expected = [(0,0),(200,0),(-200,0),(0,200),(0,-200)]
        self.validation_error_m = max(math.dist(self.to_enu(p), e) for p,e in zip(converted,expected))
        if self.validation_error_m > 1.:
            raise ValueError('地图局部转换误差过大，请重新获取')

    @staticmethod
    def samples(origin):
        return [origin.wgs84(x,y) for x,y in ((0,0),(200,0),(-200,0),(0,200),(0,-200))]

    def to_enu(self, p):
        x,y = p[0]-self.center[0], p[1]-self.center[1]
        result = [(x*self.north[1]-y*self.north[0])/self.det,
                  (self.east[0]*y-self.east[1]*x)/self.det]
        if math.hypot(*result) > self.radius_m:
            raise ValueError('路线超出本次局部对准范围，请在1公里内测试')
        return result

    def to_bd09(self, p):
        if math.hypot(*p[:2]) > self.radius_m:
            raise ValueError('已超出本次地图对准范围，请重新初始对准')
        return [self.center[i]+self.east[i]*p[0]+self.north[i]*p[1] for i in (0,1)]


class NavigationTrial:
    def __init__(self):
        previous_worker=getattr(self,'phone_heading_worker',None)
        if previous_worker is not None:previous_worker.close()
        self.phone_heading_worker=None
        self.active = False
        self.identifier = uuid.uuid4().hex
        self.rtk_enabled = True
        self.quality_policy = 'fixed_float'
        self.preview = None
        self.display_anchor = None
        self.quality_rejected = 0
        self.last_quality_stamp = None
        self.pairs = deque(maxlen=100)
        self.pair_weights = deque(maxlen=100)
        self.stable_samples = StableAlignmentSamples()
        self.events = deque(maxlen=512)
        self.projection = None
        self.reason = '启动后缓慢走一段，自动对准卫星与雷达轨迹'
        self.last_pair = 0.
        self.last_compare = 0.
        self.last_candidate = None
        self.stable = 0
        self.comparisons = 0
        self.error_sum2 = 0.
        self.error_max = 0.
        self.satellite_difference_m = None
        self.last_correction_m = 0.
        self.last_correction_deg = 0.
        self.last_logged = 0.
        self.phone_anchor = None
        self.phone_offsets = deque(maxlen=100)
        self.phone_anchor_stamp = None
        self.phone_anchor_sigma = None
        from .phone_heading import RawMotionHeading
        self.phone_heading=RawMotionHeading()
        self.phone_scalar_anchors=deque(maxlen=100)
        from .phone_translation import PhoneTranslation
        self.phone_translation=PhoneTranslation()

    def event(self, state, kind, **values):
        self.events.append(dict(stamp=state.clock(), kind=kind, **values))

    def start(self, state, *, preserve_capture=False):
        if state.simulation or state.sensor_replay or state.mode != 'walking':
            raise ValueError('请退出模拟和回放，使用手持实时数据')
        if state.guiding or state.local_navigation_active or state.journey.active:
            raise ValueError('请先结束当前引导，再启动导航与建图')
        if self.active:
            return
        if not preserve_capture and (state.track or state.local_track or state.estimated_track or state.track_bd09):
            state.save_session()
        policy = self.quality_policy
        self.__init__()
        self.quality_policy = policy
        state._alignment_pending.clear()
        self.active = True
        if not preserve_capture:
            state.session_id=self.identifier
            state.assets={}
        state.alignment = None
        state.alignment_conflict = False
        state.pairs.clear()
        if not preserve_capture:
            state.origin=None
        state.position=None;state.last_accepted=None
        state.previous_output=None;state.converted=None;state.pose_epoch+=1
        state._pending_covariance_fix=None;state.position_conflict=False;state.pending_pose=None
        if not preserve_capture:
            state.local_track=[];state.estimated_track=[];state.track.clear();state.track_bd09.clear()
        self.event(state, 'begin', joined_capture=bool(preserve_capture))

    def finish(self, state):
        if not self.active:
            return
        self.event(state, 'finish')
        state._alignment_pending.clear()
        # Persist evidence before removing the experimental transform. It must
        # never become a trusted alignment in ordinary map localization.
        state.assets['navigation_trial']=dict(self.snapshot(state),
            events=list(self.events), origin_wgs84=state.origin.values if state.origin else None,
            route_odom=self.route_in_odom(state))
        state.guiding=False
        state.guidance_generation+=1
        self.active=False
        state.alignment=None;state.alignment_conflict=False;state.pairs.clear()
        state.previous_output=None;state.converted=None;state.pose_epoch+=1
        state.alignment_reason='本次导航与建图已结束，地理对准记录已保留'

    def reset_tracking(self, state, reason='运动跟踪已重启，请重新行走完成初始对准',preserve_absolute=False,
                       event_kind='odometry_reset'):
        if not preserve_absolute:self.phone_translation.clear(state)
        self.pairs.clear(); self.last_pair=0.; self.last_candidate=None; self.stable=0
        self.preview = None
        self.display_anchor = None;self.pair_weights.clear()
        self.stable_samples = StableAlignmentSamples()
        self.phone_anchor=None;self.phone_offsets.clear()
        self.phone_anchor_stamp=None;self.phone_anchor_sigma=None
        from .phone_heading import RawMotionHeading
        self.phone_heading=RawMotionHeading();self.phone_scalar_anchors.clear()
        self.reason = reason
        self.event(state, event_kind, reason=reason)

    def set_rtk(self, state, enabled):
        if not self.active:
            raise ValueError('请先启动导航与建图')
        enabled = bool(enabled)
        if self.rtk_enabled == enabled:
            return
        self.rtk_enabled = enabled
        self.phone_translation.clear(state)
        if enabled and state.alignment and state.alignment.get('manual'):
            state.alignment = None
            state.previous_output = None
            state.converted = None
            state.pose_epoch += 1
        state._alignment_pending.clear()
        self.pairs.clear(); self.last_candidate=None; self.stable=0; self.last_pair=0.
        self.preview = None
        self.display_anchor = None;self.pair_weights.clear()
        self.stable_samples = StableAlignmentSamples()
        self.phone_anchor=None;self.phone_offsets.clear()
        from .phone_heading import RawMotionHeading
        self.phone_heading=RawMotionHeading();self.phone_scalar_anchors.clear()
        self.phone_anchor_stamp=None;self.phone_anchor_sigma=None
        self.reason = ('已恢复RTK校正，行走后自动更新对准' if enabled else
                       'RTK已退出定位；卫星仍录制用于比较，雷达继续跟踪')
        # Toggling corrections does not change an already aligned LIO pose.
        # Keep its map coordinate until the next normal position update.
        if not state.converted or state.converted.get('source') != 'gnss_lio':
            state.converted = None
        self.event(state, 'rtk_enabled', enabled=enabled)

    def accepts_quality(self, quality):
        return quality == 4 or (self.quality_policy == 'fixed_float' and quality == 5)

    def set_quality_policy(self, state, policy):
        if policy not in ('fixed', 'fixed_float'):
            raise ValueError('请选择仅固定解或固定解＋浮点解')
        if policy == self.quality_policy:
            return
        if state.guiding:
            raise ValueError('请先结束引导，再更换对齐用的卫星解')
        self.quality_policy = policy
        self.phone_translation.clear(state)
        state._alignment_pending.clear()
        self.pairs.clear(); self.last_pair=0.; self.last_candidate=None; self.stable=0
        self.preview=None; self.quality_rejected=0; self.last_quality_stamp=None
        self.display_anchor=None;self.pair_weights.clear()
        self.stable_samples=StableAlignmentSamples()
        self.phone_anchor=None;self.phone_offsets.clear()
        from .phone_heading import RawMotionHeading
        self.phone_heading=RawMotionHeading();self.phone_scalar_anchors.clear()
        self.phone_anchor_stamp=None;self.phone_anchor_sigma=None
        if self.active:
            state.alignment=None; state.alignment_conflict=False
            state.previous_output=None; state.converted=None; state.pose_epoch+=1
        self.reason='已更换对齐条件，雷达轨迹与录制继续'
        self.event(state, 'alignment_quality', policy=policy)

    def visual_snapshot(self, state):
        from .alignment_view import alignment_view
        return alignment_view(state, self)

    def feed(self, state, data, p, altitude):
        from .outdoor import finite
        from .phone_observation import alignment_quality
        quality=alignment_quality(data)
        if not state.origin:
            self.reason = ('等待固定解，雷达轨迹与录制继续' if self.quality_policy == 'fixed'
                           else '等待固定解或浮点解，雷达轨迹与录制继续')
            return
        calibration = state._observation_calibration(data)
        if calibration is None:
            self.reason = '等待天线安装偏移与时间对齐确认'; return
        antenna = state._antenna_at(data['stamp']+calibration.get('gnss_to_odom_offset_s',0.), calibration)
        if antenna is None:
            self.reason = '等待时间同步的雷达位姿'; return
        world = state.origin.enu(*p, altitude if finite(altitude) else state.origin.values[2])
        fusion=data.get('source')=='phone_continuity'
        if fusion and data.get('raw_constraints'):
            self._feed_scalar(state,data,antenna,world,calibration)
            return
        if fusion:
            # Estimate orientation from independent continuous motion. Absolute
            # location enters only as a coarse translation, never as movement.
            relative=data.get('relative_position_m')
            if not isinstance(relative,list) or len(relative)!=3 or not all(finite(v) for v in relative):return
            offset=[world[i]-relative[i] for i in range(3)]
            absolute_ok=self.accepts_quality(quality) and data.get('absolute_position_observed') is True
            if self.phone_anchor is None:
                if not absolute_ok:
                    self.reason='等待所选手机差分解建立近似位置，运动轨迹继续记录';return
                self.phone_anchor=offset
            if absolute_ok:
                self.phone_offsets.append(offset)
                self.phone_anchor_stamp=data['stamp']
                self.phone_anchor_sigma=data.get('absolute_horizontal_std_m')
            world=tuple(relative[i]+self.phone_anchor[i] for i in range(3))
        if state.alignment and data['stamp']-self.last_compare >= 1.:
            self.last_compare=data['stamp']
            error=math.dist(transform(state.alignment,antenna),world[:2])
            self.satellite_difference_m=error
            if not self.rtk_enabled:
                self.comparisons+=1;self.error_sum2+=error*error;self.error_max=max(self.error_max,error)
        if not self.rtk_enabled:
            return
        if not fusion and not self.accepts_quality(quality):
            if self.last_quality_stamp != data['stamp']:
                self.quality_rejected += 1
                self.last_quality_stamp = data['stamp']
            self.reason=('等待固定解，雷达轨迹与录制继续' if self.quality_policy == 'fixed'
                         else '等待固定解或浮点解，雷达轨迹与录制继续')
            return
        sigma=data.get('horizontal_std_m')
        if finite(sigma) and (sigma <= 0 or sigma > 2.):
            self.reason='卫星精度波动较大，保留运动跟踪，等待稳定';return
        if data['stamp']-self.last_pair < .5:
            return
        self.last_pair=data['stamp']
        weight=1. if quality==4 else .2
        if self.display_anchor is None:
            self.display_anchor=dict(yaw=0.,tx=world[0]-antenna[0],ty=world[1]-antenna[1],
                                     preview_only=True,unoriented=True)
        self.pairs.append((antenna,world))
        self.pair_weights.append(weight)
        from .alignment_view import preview_fit
        self.preview = preview_fit(list(self.pairs),list(self.pair_weights))
        added = self.stable_samples.add(data['stamp'], antenna, world,weight)
        candidate = trajectory_alignment(self.stable_samples.pairs,self.stable_samples.weights) if added else None
        installation = dict(
            installation_policy=calibration.get('installation_policy', 'measured_lever_arm'),
            timing_policy=calibration.get('timing_policy', 'measured_offset'),
            lever_arm_verified=calibration.get('lever_arm_verified') is True,
            time_verified=calibration.get('time_verified') is True,
            antenna_uncertainty_m=calibration.get('antenna_uncertainty_m'))
        if not candidate:
            if (not fusion and state.alignment and state.alignment.get('odom_continuity')==state.odom_continuity
                    and state.alignment.get('anchor_source')==(list(state.gnss_source) if state.gnss_source else None)
                    and state.alignment.get('phone_generation',0)==state.phone_alignment_generation
                    and math.dist(transform(state.alignment,antenna),world[:2])<=.8):
                # Pausing after initial alignment should not discard a measured
                # heading. A stationary fix only renews agreement, not yaw.
                state.alignment.update(installation, stamp=data['stamp']);state._anchor_odometry()
                self.reason='卫星与雷达位置一致，保留已对准方向';return
            self.reason = self.stable_samples.reason
            if added:
                self.stable = 0
                self.last_candidate = None
                self.reason = (f'已有{len(self.stable_samples.keyframes)}个稳定轨迹点，方向尚未可靠对齐；'
                               '建图和录包继续')
            return
        old_candidate=self.last_candidate
        if fusion:
            # Robust coarse origin, without shrinking its uncertainty by
            # treating correlated phone estimates as independent fixes.
            coarse=[statistics.median(v[i] for v in self.phone_offsets) for i in range(2)]
            candidate=dict(candidate,tx=candidate['tx']+coarse[0]-self.phone_anchor[0],
                           ty=candidate['ty']+coarse[1]-self.phone_anchor[1])
        self.last_candidate=candidate
        if (old_candidate and abs(angle(candidate['yaw']-old_candidate['yaw'])) < math.radians(2)
                and math.dist(transform(candidate,antenna),transform(old_candidate,antenna)) < 1.):
            self.stable+=1
        else:
            self.stable=1
        if self.stable < 3:
            self.reason='正在复核初始对准，请继续平稳行走';return
        old=state.alignment
        adopted=copy.deepcopy(candidate)
        if old:
            dt=max(0.,min(1.,data['stamp']-old['stamp']))
            delta=angle(candidate['yaw']-old['yaw'])
            yaw=old['yaw']+max(-math.radians(.5)*dt,min(math.radians(.5)*dt,delta))
            start,end=transform(old,antenna),transform(candidate,antenna)
            difference=math.dist(start,end)
            rate=min(1.,.3*dt/max(difference,1e-9))
            point=[start[i]+rate*(end[i]-start[i]) for i in (0,1)]
            adopted.update(yaw=yaw,tx=0.,ty=0.)
            rotated=transform(adopted,antenna)
            adopted.update(tx=point[0]-rotated[0],ty=point[1]-rotated[1])
            self.last_correction_m=difference*rate
            self.last_correction_deg=math.degrees(angle(yaw-old['yaw']))
        adopted.update(stamp=data['stamp'],tz=0.,vertical_alignment_valid=False,
                       experimental=True, precision_source='trajectory_agreement',
                       estimate_source=data.get('source','receiver'),
                       receiver_precision_available=not fusion and finite(sigma) and sigma > 0)
        if fusion:
            adopted.update(absolute_horizontal_sigma_m=self.phone_anchor_sigma,
                           absolute_anchor_stamp=self.phone_anchor_stamp,
                           absolute_accuracy_verified=False,geographic_accuracy='approximate',
                           precision_source='phone_relative_motion_agreement')
        adopted.update(installation)
        state.alignment=adopted;state._anchor_odometry()
        if old is None:
            # A lever-arm/body offset and the new rotation may change the
            # position source. Compare subsequent positions within this epoch.
            state.previous_output=None;state.converted=None;state.pose_epoch+=1
        self.reason=('手机运动方向已对准，地理位置为近似估计' if fusion else
                     '初始对准已完成，行走中持续微调') if self.rtk_enabled else '雷达独立跟踪'
        state.alignment_reason=self.reason
        if old is None:self.event(state,'initial_alignment',alignment=copy.deepcopy(adopted))

    def _feed_scalar(self,state,data,antenna,world,calibration):
        from .phone_observation import alignment_quality
        if not self.rtk_enabled:return
        if self.accepts_quality(alignment_quality(data)) and data.get('absolute_position_observed') is True:
            self.phone_scalar_anchors.append((list(antenna),list(world),data['stamp']))
            self.phone_anchor_stamp=data['stamp'];self.phone_anchor_sigma=data.get('absolute_horizontal_std_m')
        self.poll_phone_heading(state)
        from .phone_heading import constraints,HeadingWorker
        raw=data['raw_constraints'];parsed=constraints(raw)
        if parsed is None:return
        points={}
        for kind in ('phase','doppler'):
            if kind not in parsed:continue
            for stamp in (parsed[kind]['start'],parsed[kind]['end']):
                point=state._antenna_at(stamp+calibration.get('gnss_to_odom_offset_s',0.),calibration)
                points[stamp]=None if point is None else list(point)
        if self.phone_heading_worker is None:self.phone_heading_worker=HeadingWorker()
        context=dict(tag=self._heading_tag(state),data=copy.deepcopy(data),antenna=list(antenna),
            calibration=dict(calibration),anchors=list(self.phone_scalar_anchors),
            anchor_stamp=self.phone_anchor_stamp,anchor_sigma=self.phone_anchor_sigma)
        self.phone_heading_worker.submit(self.phone_heading,copy.deepcopy(raw),points,context)
        self.reason=self.phone_heading.reason
        return

    def _heading_tag(self,state):
        return (self.identifier,state.phone_alignment_generation,state.odom_continuity,
                state.odom_epoch,state.gnss_source,id(self.phone_heading),self.rtk_enabled)

    def poll_phone_heading(self,state,wait=False):
        # Production never waits; wait=True is solely for deterministic offline
        # replay so host speed cannot decide which recorded epochs are dropped.
        if self.phone_heading_worker is None:return
        completed=self.phone_heading_worker.poll(wait=wait)
        if completed is None:return
        context,result,_=completed
        if (context is None or result is None or not self.active or not state.phone_motion_enabled
                or context['tag']!=self._heading_tag(state)
                or not 0<=state.clock()-context['data']['stamp']<=10.):return
        self._apply_scalar_result(state,context,result)

    def _apply_scalar_result(self,state,context,result):
        # Once a fully checked raw absolute window owns the map transform,
        # later scalar-only estimates must not restore its old coarse origin.
        if state.phone_absolute.owns_alignment(state):
            return
        data,antenna,calibration=(context[k] for k in ('data','antenna','calibration'))
        if not context['anchors']:return
        yaw=result['yaw'];c,s=math.cos(yaw),math.sin(yaw)
        offsets=[(q[0]-c*p[0]+s*p[1],q[1]-s*p[0]-c*p[1]) for p,q,_ in context['anchors']]
        candidate=dict(result,tx=statistics.median(p[0] for p in offsets),
            ty=statistics.median(p[1] for p in offsets),stamp=data['stamp'],tz=0.,
            vertical_alignment_valid=False,experimental=True,
            estimate_source='phone_continuity',precision_source='raw_scalar_livo_heading',
            receiver_precision_available=False,geographic_accuracy='approximate',
            absolute_accuracy_verified=False,absolute_horizontal_sigma_m=context['anchor_sigma'],
            absolute_anchor_stamp=context['anchor_stamp'],
            installation_policy=calibration.get('installation_policy'),
            timing_policy=calibration.get('timing_policy'),lever_arm_verified=False,time_verified=False,
            antenna_uncertainty_m=None)
        self.last_candidate=copy.deepcopy(candidate);self.stable+=1
        old=state.alignment
        if old:
            dt=max(0.,min(1.,data['stamp']-old['stamp']))
            change=angle(yaw-old['yaw'])
            candidate['yaw']=old['yaw']+max(-math.radians(.5)*dt,min(math.radians(.5)*dt,change))
            start,end=transform(old,antenna),transform(candidate,antenna)
            difference=math.dist(start,end);rate=min(1.,.3*dt/max(difference,1e-9))
            point=[start[i]+rate*(end[i]-start[i]) for i in (0,1)]
            candidate.update(tx=0.,ty=0.)
            rotated=transform(candidate,antenna)
            candidate.update(tx=point[0]-rotated[0],ty=point[1]-rotated[1])
            self.last_correction_m=difference*rate
            self.last_correction_deg=math.degrees(angle(candidate['yaw']-old['yaw']))
            if self.phone_translation.applies(state):
                # A heading refinement must not restore the old coarse origin.
                # Rotate about this historical LiVO endpoint; the independent
                # absolute target will adjust translation on following ticks.
                point=transform(old,antenna)
                candidate.update(tx=0.,ty=0.)
                rotated=transform(candidate,antenna)
                candidate.update(tx=point[0]-rotated[0],ty=point[1]-rotated[1])
                for key in ('absolute_anchor_source','absolute_anchor_revision','absolute_anchor_stamp',
                            'absolute_horizontal_sigma_m','absolute_anchor_horizontal_sigma_m',
                            'geographic_correction_total_m','geographic_correction_remaining_m'):
                    if key in old:candidate[key]=old[key]
        state.alignment=candidate;state._anchor_odometry()
        if old is None:
            state.previous_output=None;state.converted=None;state.pose_epoch+=1
            self.event(state,'initial_alignment',alignment=copy.deepcopy(candidate))
        self.reason='手机原始运动与雷达方向已对准；地理位置为近似估计'
        state.alignment_reason=self.reason

    def route_in_odom(self,state):
        if (not self.active or not self.projection or not state.alignment or not state.route
                or state.alignment_conflict
                or state.alignment.get('odom_continuity') != state.odom_continuity):
            return []
        try:return [inverse(state.alignment,self.projection.to_enu(p)) for p in state.route['points']]
        except ValueError:return []

    def snapshot(self,state):
        a=state.alignment if self.active else None
        timing=state.odometry_timing()
        live=timing['fresh'];delayed=timing['delayed']
        aligned=bool(a and not state.alignment_conflict
                     and a.get('odom_continuity')==state.odom_continuity)
        satellite_live=bool(state.position and self.accepts_quality(state.position.get('native_quality',state.position.get('quality')))
                            and -.05<=state.clock()-state.position['stamp']<=1.5)
        phase=('未开始' if not self.active else '运动跟踪延迟' if delayed else '运动跟踪中断' if not live else
               '初始对准暂停，请开启RTK' if not aligned and not self.rtk_enabled else
               ('初始对准' if satellite_live or (state.phone_motion_enabled and self.phone_heading.accepted_edges) else '等待卫星位置') if not aligned else
               ('手动定位引导' if state.guiding else '手动定位已确认') if a and a.get('manual') else
               ('雷达独立导航' if state.guiding else '雷达独立跟踪') if not self.rtk_enabled else
               '合格卫星位置暂缺，雷达接续' if not satellite_live else
               '行走导航' if state.guiding else '已对准，可规划路线')
        return dict(active=self.active,trial_id=self.identifier,phase=phase,rtk_enabled=self.rtk_enabled,
            alignment_method='manual' if a and a.get('manual') else (a.get('heading_method','trajectory') if a else 'trajectory'),
            reason=self.reason,aligned=aligned,odometry_live=live,odometry_delayed=delayed,
            odometry_timing=timing,
            satellite_live=satellite_live,pairs=len(self.pairs),
            stable_pairs=len(self.stable_samples.keyframes), rejected_windows=self.stable_samples.rejected,
            phone_heading_edges=self.phone_heading.accepted_edges,
            phone_heading_pending=self.phone_heading.candidate is not None,
            phone_heading_check_edges=len(self.phone_heading.check),
            quality_policy=self.quality_policy, quality_rejected=self.quality_rejected,
            preview=copy.deepcopy(self.preview), verification_count=self.stable,
            alignment=copy.deepcopy(a), map_projection_ready=self.projection is not None,
            satellite_difference_m=self.satellite_difference_m,
            comparison_count=self.comparisons,
            comparison_rms_m=math.sqrt(self.error_sum2/self.comparisons) if self.comparisons else None,
            comparison_max_m=self.error_max if self.comparisons else None,
            correction_m=self.last_correction_m, correction_deg=self.last_correction_deg,
            accuracy_verified=False, road_constraint_applied=False)

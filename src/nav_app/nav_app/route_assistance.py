"""Optional route advice, isolated from sensor callbacks and base navigation.

Only the newest request is retained. The worker never acquires OutdoorState's
lock, changes a pose, or calls a capture/driver service. Heavy imagery models
are deliberately not part of this device runtime.
"""
import copy
import logging
import math
import threading
import time


class RouteAssistance:
    def __init__(self, enabled=False):
        self.enabled = bool(enabled)
        self._condition = threading.Condition()
        self._thread = None
        self._generation = 0
        self._pending = None
        self._context = None
        self._last_stamp = None
        self._result = None
        self._retry_after = 0.
        self._closed = False

    def set_enabled(self, enabled):
        with self._condition:
            if self.enabled != enabled:
                self.enabled = bool(enabled)
                self._invalidate()
            self._condition.notify_all()

    def _invalidate(self):
        self._generation += 1
        self._pending = self._context = self._last_stamp = self._result = None
        self._retry_after = 0.

    def close(self):
        with self._condition:
            self._closed = True
            self._invalidate()
            self._condition.notify_all()

    def _status(self, state, message):
        return dict(enabled=self.enabled, available=False, state=state, message=message,
                    distance_interval_m=None, turn='', sources=[], replan_available=False,
                    changes_odometry=False, capture_continues=True, imagery_available=False)

    def snapshot(self, outdoor, base_guide, position):
        """Called with the state lock held; copy a bounded request, then return."""
        with self._condition:
            if not self.enabled or self._closed:
                return self._status('disabled', '路口辅助已关闭')
            trial = outdoor.trial
            missing = None
            if outdoor.mode != 'walking' or outdoor.simulation or outdoor.sensor_replay:
                missing = '路口辅助用于实时步行引导'
            elif not trial.active:
                missing = '开始导航与建图后启用路口辅助'
            elif not outdoor.guiding or not outdoor.route:
                missing = '开始引导后提醒路口与错路'
            elif base_guide and base_guide.get('uncertain'):
                missing = '路线进度待确认，基础引导继续'
            elif (not position or not base_guide or not outdoor.odom or not outdoor.alignment
                  or not trial.projection or outdoor.alignment_conflict
                  or outdoor.alignment.get('odom_continuity') != outdoor.odom_continuity
                  or not -.05 <= outdoor.clock()-outdoor.odom[-1][0] <= .5):
                missing = '等待有效运动跟踪，基础引导继续'
            if missing:
                if self._context is not None or self._pending is not None or self._result is not None:
                    self._invalidate()
                return self._status('waiting', missing)

            now = time.monotonic()
            key = (trial.identifier, outdoor.guidance_generation, outdoor.odom_epoch,
                   outdoor.odom_continuity, id(outdoor.route), id(trial.projection))
            stamp, point, quaternion = outdoor.odom[-1]
            alignment = {k:float(outdoor.alignment[k]) for k in ('yaw','tx','ty')}
            # Invalidate while holding the publish lock: a slow worker from the
            # old geographic frame must not restore an obsolete turn warning.
            # Compare against the window's initial frame, not the last update,
            # so a sequence of individually small adjustments also resets it.
            frame_changed = (self._context is not None
                             and self._frame_changed(self._context['alignment'],alignment,point))
            if self._context is None or self._context['key'] != key or frame_changed:
                self._invalidate()
                self._context = dict(key=key, alignment=alignment,
                    points=tuple(tuple(p) for p in outdoor.route['points']),
                    projection=copy.deepcopy(trial.projection),
                    route_length=float(outdoor.route['distance_m']))
            if now >= self._retry_after and (self._last_stamp is None or stamp-self._last_stamp >= .2):
                self._last_stamp = stamp
                absolute_sigma=position.get('absolute_horizontal_sigma_m')
                if type(absolute_sigma) not in (int,float) or not math.isfinite(absolute_sigma) or absolute_sigma<=0:
                    absolute_sigma=2.
                # A conservative route-association scale, not a calibrated
                # covariance. Never turn an approximate phone anchor into 2m.
                sigma=max(2.,absolute_sigma)+min(18.,max(0.,position.get('distance_since_correction_m',0.))*.01)
                yaw_sigma=position.get('yaw_displacement_sigma_m')
                if type(yaw_sigma) in (int,float) and math.isfinite(yaw_sigma) and yaw_sigma>=0:
                    # The joint endpoint/yaw cross covariance is unavailable
                    # here: sum scales conservatively rather than assuming
                    # independent errors. Rotation uses net displacement;
                    # the separate travelled-distance drift term stays above.
                    sigma+=yaw_sigma
                elif 'yaw_displacement_uncertainty' in position:
                    # Unknown is not zero. This is a broad association scale,
                    # explicitly not a statistical bound on unknown drift.
                    sigma=max(sigma,30.)
                self._pending = dict(generation=self._generation, context=self._context,
                    stamp=float(stamp), point=tuple(point), quaternion=tuple(quaternion),
                    alignment=alignment,
                    progress=float(outdoor.route_progress),
                    sigma=sigma)
                if self._thread is None or not self._thread.is_alive():
                    self._thread = threading.Thread(target=self._run, name='route-assistance', daemon=True)
                    self._thread.start()
                self._condition.notify()
            if self._result and self._result['generation'] == self._generation:
                if (now-self._result['finished_at'] <= 1.5
                        and stamp-self._result['value'].get('t',stamp) <= 1.5):
                    return copy.deepcopy(self._result['value'])
            return self._status('waiting', '正在核对前方路口，基础引导继续')

    @staticmethod
    def _transform(alignment, point):
        c, s = math.cos(alignment['yaw']), math.sin(alignment['yaw'])
        return [c*point[0]-s*point[1]+alignment['tx'],
                s*point[0]+c*point[1]+alignment['ty']]

    @classmethod
    def _frame_changed(cls, old, current, point):
        shift = math.dist(cls._transform(old,point),cls._transform(current,point))
        yaw_change = abs(math.atan2(math.sin(old['yaw']-current['yaw']),
                                    math.cos(old['yaw']-current['yaw'])))
        return shift > 3. or yaw_change > math.radians(2.)

    def _evaluate(self, request, runtime):
        # Lazy import: default-off navigation does not load even numpy here.
        from .route_assistance_core import Route, RouteGuard
        generation = request['generation']
        alignment = request['alignment']
        reset = runtime is None or runtime['generation'] != generation
        if not reset:
            reset = self._frame_changed(runtime['alignment'],alignment,request['point'])
        if reset:
            context = request['context']
            route = Route([context['projection'].to_enu(p) for p in context['points']])
            guard = RouteGuard(route, str(generation), enabled=True)
            progress = min(float(route.s[-1]), max(0.,request['progress']
                           *float(route.s[-1])/max(context['route_length'],1.)))
            guard.progress = progress
            # Seed the upcoming maneuver, retaining a nearby already-passed
            # corner for verification. This does not count past turns as observed.
            guard.next_turn = next((i for i,t in enumerate(route.turns)
                                    if t.progress >= progress-8.),len(route.turns))
            runtime = dict(generation=generation,
                           alignment=context.get('alignment',alignment), guard=guard)
        guard = runtime['guard']
        # Freeze the frame over a motion window: gradual RTK frame adjustments
        # must not appear as physical turning while the handheld is stationary.
        a = runtime['alignment']
        xy = self._transform(a,request['point'])
        x,y,z,w = request['quaternion']
        yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))+a['yaw']
        value = guard.update(request['stamp'],[*xy,yaw],sigma_m=request['sigma'])
        value.update(enabled=True, available=True, imagery_available=False,
                     sources=['route','lidar'])
        value.setdefault('distance_interval_m',None)
        value.setdefault('turn','')
        return value, runtime

    def _run(self):
        runtime = None
        while True:
            with self._condition:
                if self._closed or not self.enabled:
                    self._thread = None
                    return
                if self._pending is None:
                    self._condition.wait(timeout=2.)
                    if self._pending is None:
                        runtime = None
                        self._thread = None
                        return
                request, self._pending = self._pending, None
            if request is None:
                continue
            failed = False
            try:
                value, runtime = self._evaluate(request,runtime)
            except Exception:
                logging.getLogger(__name__).exception('route assistance failed; base navigation unchanged')
                failed = True
                runtime = None
                value = self._status('unavailable','路口辅助暂不可用，基础引导继续')
            with self._condition:
                if self.enabled and not self._closed and request['generation'] == self._generation:
                    self._result = dict(generation=self._generation,finished_at=time.monotonic(),value=value)
                    if failed:
                        self._retry_after = time.monotonic()+5.

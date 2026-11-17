"""Causal pedestrian maneuver decisions from motion, route and map observations.

This layer owns instructions, never odometry, TF, recording or motor commands.
Route intent is not a position measurement. Image hypotheses are transported
from their capture pose and kept separate; ambiguous roads are not averaged.
"""
from collections import deque
import copy
from dataclasses import dataclass
import math
import numpy as np

# This production core intentionally has no ROS, network, worker, SciPy or
# research-package dependency. The owner supplies validated metre-frame inputs.
def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def compose(a, b):
    return np.r_[a[:2] + rot(a[2]) @ b[:2], wrap(a[2] + b[2])]



def angle(vector):
    return math.atan2(vector[1], vector[0])


@dataclass(frozen=True)
class Turn:
    progress: float
    point: np.ndarray
    incoming: np.ndarray
    outgoing: np.ndarray
    radians: float

    @property
    def direction(self):
        return '掉头' if abs(self.radians) > math.radians(150) else ('左转' if self.radians > 0 else '右转')


@dataclass(frozen=True)
class _Mode:
    pose: np.ndarray
    covariance: np.ndarray
    weight: float


@dataclass(frozen=True)
class _Observation:
    stamp: float
    modes: tuple
    calibrated: bool
    map_bias_covariance: np.ndarray


class Route:
    def __init__(self, points, *, corridor_m=6., map_error_m=3.):
        p = np.asarray(points, float)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or not np.isfinite(p).all():
            raise ValueError('route requires finite ENU metre coordinates')
        p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > .01]]
        if len(p) < 2 or not math.isfinite(corridor_m + map_error_m) or min(corridor_m, map_error_m) <= 0:
            raise ValueError('invalid route or uncertainty')
        self.points = p
        self.delta = np.diff(p, axis=0)
        self.lengths = np.linalg.norm(self.delta, axis=1)
        self.s = np.r_[0., np.cumsum(self.lengths)]
        self.corridor_m, self.map_error_m = corridor_m, map_error_m
        self.turns = []
        for i in range(1, len(p)-1):
            s = self.s[i]
            if min(s, self.s[-1]-s) < 3.:
                continue
            before, after = p[i]-self.at(s-4.), self.at(s+4.)-p[i]
            if min(np.linalg.norm(before), np.linalg.norm(after)) < 1.:
                continue
            a = float(wrap(angle(after)-angle(before)))
            if abs(a) < math.radians(40):
                continue
            turn = Turn(float(s), p[i].copy(), before/np.linalg.norm(before), after/np.linalg.norm(after), a)
            if self.turns and s-self.turns[-1].progress < 5.:
                if abs(a) > abs(self.turns[-1].radians):
                    self.turns[-1] = turn
            else:
                self.turns.append(turn)

    def at(self, s):
        s = float(np.clip(s, 0, self.s[-1]))
        i = min(len(self.lengths)-1, max(0, np.searchsorted(self.s, s, side='right')-1))
        return self.points[i] + self.delta[i]*(s-self.s[i])/self.lengths[i]

    def project(self, point, low=0., high=float('inf')):
        fraction = np.clip(np.sum((point-self.points[:-1])*self.delta, axis=1)/self.lengths**2, 0., 1.)
        # Intersect each segment with the permitted progress window first.
        # Otherwise a perfectly valid point on a long segment could disappear
        # merely because its unconstrained projection lies outside the window.
        starts = np.maximum(self.s[:-1], low)
        ends = np.minimum(self.s[1:], high)
        eligible = starts <= ends
        s = np.clip(self.s[:-1]+fraction*self.lengths, starts, np.maximum(starts, ends))
        fraction = (s-self.s[:-1])/self.lengths
        q = self.points[:-1]+fraction[:, None]*self.delta
        distances = np.linalg.norm(q-point, axis=1)
        if not eligible.any():
            return None
        i = int(np.argmin(np.where(eligible, distances, np.inf)))
        return float(s[i]), float(distances[i])


class RouteGuard:
    """Bounded, non-blocking instruction monitor. One instance per route/epoch.

    Calibrated map likelihoods can narrow route location through conservative
    covariance intersection; uncalibrated outputs only widen the candidate set.
    Sources sharing a tile/evidence identity cannot vote repeatedly.
    """
    max_modes = 8
    max_evidence_per_observation = 128
    max_evidence = 2048

    def __init__(self, route, epoch, *, announce_m=18., history_s=30., enabled=False):
        self.route, self.epoch = route, epoch
        self.announce_m, self.history_s = announce_m, history_s
        self.history = deque(maxlen=600)
        self.map_observations = {}
        self.source_status = {}
        self.progress = 0.
        self.next_turn = 0
        self.completed = []
        self.latched = None
        self._event = None
        self._event_since = None
        self._event_origin = None
        self._last = None
        self._source_latest = {}
        self._seen = {}
        self.deviation = None
        self.requires_initialization = False
        self.enabled = bool(enabled)
        self._last_motion = None
        self._needs_route_context = False

    def set_enabled(self, enabled, *, progress_m=None, next_turn=None):
        """Opt-in auxiliary decisions; the caller always owns base navigation.

        After receiving motion while disabled, resume using the base navigator's
        confirmed route progress and next maneuver. Do not infer that missed
        turns were completed by snapping to the nearest route segment.
        """
        enabled = bool(enabled)
        has_context = progress_m is not None or next_turn is not None
        if has_context:
            if (progress_m is None or next_turn is None
                    or not math.isfinite(progress_m) or not 0 <= progress_m <= self.route.s[-1]
                    or not isinstance(next_turn, int) or not 0 <= next_turn <= len(self.route.turns)):
                raise ValueError('provide valid progress and next turn together')
        if enabled == self.enabled and not has_context:
            return self.snapshot()
        self.enabled = enabled
        self.history.clear()
        self.map_observations.clear()
        self.source_status.clear()
        self._seen.clear()
        self.latched = self.deviation = self._event = None
        self._event_since = self._event_origin = None
        if has_context:
            self.progress = float(progress_m)
            self.next_turn = next_turn
            self.completed = list(range(next_turn))
            self._needs_route_context = False
        else:
            self._needs_route_context = self._last_motion is not None
        self._last = None
        return self.snapshot()

    def _bypass(self):
        state = 'disabled' if not self.enabled else 'waiting_route_context'
        result = dict(state=state, enabled=self.enabled,
            message='辅助判断已关闭，基础引导继续' if not self.enabled else
                    '辅助判断等待当前路线进度，基础引导继续',
            use_base_navigation=True, capture_continues=True, changes_odometry=False,
            replan_available=False, sources=[], source_status={}, hypotheses=[], completed_turns=[])
        if self._last_motion is not None:
            t, pose, _ = self._last_motion
            result.update(t=t, position=pose[:2].tolist())
        return result

    def observe(self, observation, source='satellite'):
        if not self.enabled:
            return 'disabled'
        if self._needs_route_context:
            return 'waiting_route_context'
        if source not in ('satellite', 'streetview'):
            raise ValueError('unknown map source')
        reason = self._validate_observation(observation)
        if reason:
            self.source_status[source] = reason
            return reason
        self._prune_evidence()
        reason = 'accepted'
        if self.requires_initialization:
            reason = 'reinitialize'
        elif not self.history:
            reason = 'no_motion'
        elif observation.epoch != self.epoch:
            reason = 'epoch_mismatch'
        elif observation.available_at > self.history[-1][0] + 1e-6:
            reason = 'not_arrived'
        elif not observation.modes:
            reason = 'no_candidates'
        elif observation.stamp <= self._source_latest.get(source, -math.inf):
            reason = 'duplicate_or_old'
        elif not any(abs(t-observation.stamp) < 1e-6 for t, _, _ in self.history):
            reason = 'capture_pose_missing'
        elif any((observation.epoch, e) in self._seen for e in observation.evidence):
            reason = 'repeated_evidence'
        if reason == 'accepted' and len(self._seen)+len(observation.evidence) > self.max_evidence:
            reason = 'evidence_budget'
        if reason == 'accepted':
            # Copy only the bounded fields used here, never a producer's opaque
            # object, which may also carry images or other large payloads.
            self.map_observations[source] = _Observation(float(observation.stamp),
                tuple(_Mode(np.asarray(m.pose, float).copy(),
                            np.asarray(m.covariance, float).copy(), float(m.weight))
                      for m in observation.modes),
                bool(observation.calibrated), np.asarray(observation.map_bias_covariance, float).copy())
            self._source_latest[source] = observation.stamp
            self._seen.update({(observation.epoch, e): observation.stamp for e in observation.evidence})
            if not observation.calibrated:
                reason = 'tentative'
        self.source_status[source] = reason
        return reason

    def _prune_evidence(self):
        earliest = self.history[0][0] if self.history else math.inf
        self._seen = {key: stamp for key, stamp in self._seen.items() if stamp >= earliest}

    def _validate_observation(self, observation):
        """Reject oversized/malformed worker results before any expensive copy.

        Exact capture pose and source watermarks are checked by observe(). The
        per-window ledger refuses excess evidence instead of forgetting recent
        identities and accidentally counting the same landmarks twice.
        """
        try:
            modes, evidence = observation.modes, observation.evidence
            if (not isinstance(modes, (tuple, list)) or len(modes) > self.max_modes
                    or not isinstance(evidence, (tuple, list))
                    or not 1 <= len(evidence) <= self.max_evidence_per_observation):
                return 'observation_budget'
            if (not all(isinstance(e, str) and 0 < len(e) <= 160 for e in evidence)
                    or len(set(evidence)) != len(evidence)
                    or not isinstance(observation.epoch, str) or len(observation.epoch) > 128
                    or not isinstance(observation.calibrated, (bool, np.bool_))):
                return 'invalid_observation'
            times = [observation.stamp, observation.available_at,
                     observation.support_start, observation.support_end]
            if (not np.isfinite(times).all()
                    or not observation.support_start <= observation.support_end <= observation.stamp <= observation.available_at):
                return 'invalid_observation'
            covariances = [observation.map_bias_covariance]
            for mode in modes:
                if (np.asarray(mode.pose).shape != (3,) or not np.isfinite(mode.pose).all()
                        or not math.isfinite(mode.weight) or not 0 < mode.weight <= 1):
                    return 'invalid_observation'
                covariances.append(mode.covariance)
            for covariance in covariances:
                a = np.asarray(covariance, float)
                if (a.shape != (3, 3) or not np.isfinite(a).all()
                        or not np.allclose(a, a.T) or np.linalg.eigvalsh(a).min() <= 0):
                    return 'invalid_observation'
        except (AttributeError, TypeError, ValueError, OverflowError):
            return 'invalid_observation'
        return None

    def positions(self, t, pose, sigma):
        raw = (pose[:2].copy(), sigma, 'lidar')
        choices = []
        tentative = []
        for source, obs in self.map_observations.items():
            if t-obs.stamp > 8.:
                self.source_status[source] = 'stale'
                continue
            old = next(((p, s) for ts, p, s in self.history if abs(ts-obs.stamp) < 1e-6), None)
            if old is None:
                continue
            p, _ = old
            delta = np.r_[rot(-p[2])@(pose[:2]-p[:2]), wrap(pose[2]-p[2])]
            for mode in obs.modes:
                if mode.weight < .1:
                    continue
                mapped = compose(mode.pose, delta)[:2]
                # Include map datum uncertainty and angular transport uncertainty.
                variance = float(np.linalg.eigvalsh(mode.covariance[:2,:2]+obs.map_bias_covariance[:2,:2]).max())
                variance += np.linalg.norm(delta[:2])**2 * (mode.covariance[2,2]+obs.map_bias_covariance[2,2])
                variance += (.1*(t-obs.stamp))**2
                if np.linalg.norm(mapped-pose[:2]) > 3.*math.sqrt(sigma*sigma+variance):
                    continue
                if not obs.calibrated:
                    # Experimental imagery can reveal a conflicting junction,
                    # but never remove the raw hypothesis or narrow its interval.
                    tentative.append((mapped, max(sigma, math.sqrt(variance), 5.), source+'_tentative'))
                    continue
                # CI accounts conservatively for shared initialization/map cues.
                precision = .5/(sigma*sigma)+.5/variance
                estimate = (.5*pose[:2]/(sigma*sigma)+.5*mapped/variance)/precision
                choices.append((estimate, math.sqrt(1./precision), source))
        # Multiple sources remain alternatives, not repeated independent votes.
        return (choices or [raw]) + tentative

    def update(self, t, pose, *, sigma_m=2., epoch=None):
        pose = np.asarray(pose, float)
        if pose.shape != (3,) or not np.isfinite(pose).all() or not math.isfinite(t+sigma_m) or sigma_m <= 0:
            raise ValueError('invalid motion sample')
        if (self._last_motion is not None and t <= self._last_motion[0]
                and not self.requires_initialization and (epoch is None or epoch == self.epoch)):
            raise ValueError('motion time must increase')
        self._last_motion = (float(t), pose.copy(), float(sigma_m))
        if epoch is not None and epoch != self.epoch:
            self.requires_initialization = True
        if not self.enabled or self._needs_route_context:
            return self.snapshot()
        if self.requires_initialization or (epoch is not None and epoch != self.epoch):
            self.requires_initialization=True
            self.history.clear();self.map_observations.clear();self._seen.clear()
            self.source_status.clear()
            self._last=dict(state='reinitialize', message='运动跟踪已重启，请重新确认起点和方向', capture_continues=True)
            return self.snapshot()
        if self.history and t <= self.history[-1][0]:
            raise ValueError('motion time must increase')
        if self.history and t-self.history[-1][0] > 3.:
            self.history.clear()
            self.map_observations.clear()
            self.source_status.clear()
            self._seen.clear()
            self.latched = None
            self._event = None
        previous = self.history[-1] if self.history else None
        moved = float(np.linalg.norm(pose[:2]-previous[1][:2])) if previous else 0.
        self.history.append((float(t), pose.copy(), float(sigma_m)))
        while len(self.history) > 1 and self.history[0][0] < t-self.history_s:
            self.history.popleft()
        self._prune_evidence()
        return self._evaluate(t, pose, sigma_m, moved)

    def heading(self, pose):
        for _, old, _ in reversed(self.history):
            v = pose[:2]-old[:2]
            if np.linalg.norm(v) >= 3.:
                return v/np.linalg.norm(v)
        return None

    def snapshot(self):
        """Reading a UI frame cannot advance progress or confirm another turn."""
        if not self.enabled or self._needs_route_context:
            return self._bypass()
        return copy.deepcopy(self._last) if self._last else dict(
            state='waiting', enabled=True, message='等待运动跟踪', capture_continues=True)

    def _evaluate(self, t, pose, sigma, moved):
        if not self.history:
            return dict(state='waiting', message='等待运动跟踪', capture_continues=True)
        self._requested_event = None
        options = self.positions(t, pose, sigma)
        projections = [self.route.project(p, max(0., self.progress-8.), self.progress+max(12., moved*2.+6.))
                       for p, _, _ in options]
        valid = [p for p in projections if p is not None]
        if valid:
            self.progress = max(self.progress, min(p[0] for p in valid))
        heading = self.heading(pose)
        result = dict(state='continue', enabled=True, message='沿当前道路前行', t=float(t),
                      position=pose[:2].tolist(), progress_m=round(self.progress,2),
                      sources=sorted(set(s for _,_,s in options)), source_status=dict(self.source_status),
                      hypotheses=[dict(xy=p.tolist(),sigma_m=s,source=src) for p,s,src in options],
                      completed_turns=list(self.completed), capture_continues=True,
                      changes_odometry=False, replan_available=False)
        turn = self.route.turns[self.next_turn] if self.next_turn < len(self.route.turns) else None
        if turn is not None:
            distances = [float((turn.point-p)@turn.incoming) for p,_,_ in options]
            errors = [max(self.route.map_error_m, 2*s) for _,s,_ in options]
            lo = min(d-e for d,e in zip(distances,errors))
            hi = max(d+e for d,e in zip(distances,errors))
            result.update(turn=turn.direction, distance_interval_m=[round(lo,1),round(hi,1)],
                          turn_point=turn.point.tolist(), turn_index=self.next_turn)
            near = lo <= self.announce_m and hi >= -15.
            if near and self.latched is None and heading is not None:
                # A heading from actual movement; rotating the device is irrelevant.
                if abs(wrap(angle(heading)-angle(turn.incoming))) < math.radians(45):
                    self.latched = heading.copy()
            actual = float(wrap(angle(heading)-angle(self.latched))) if heading is not None and self.latched is not None else None
            is_outgoing = heading is not None and abs(wrap(angle(heading)-angle(turn.outgoing))) < math.radians(30)
            near_corner = [np.linalg.norm(p-turn.point) <= self.announce_m+e for (p,_,_),e in zip(options,errors)]
            signed_turn = actual is not None and abs(actual) > math.radians(35) and (actual*turn.radians > 0 or abs(turn.radians)>math.radians(150))
            outgoing_dist = [float((p-turn.point)@turn.outgoing) for p,_,_ in options]
            passed = [d < -max(5.,e) for d,e in zip(distances,errors)]
            incoming = heading is not None and abs(wrap(angle(heading)-angle(turn.incoming))) < math.radians(30)
            normal=np.array([-turn.incoming[1],turn.incoming[0]])
            on_incoming=all(abs((p-turn.point)@normal)<=self.route.corridor_m+e
                            for (p,_,_),e in zip(options,errors))
            corner_agreement = all(abs(d) <= max(6.,e) for d,e in zip(distances,errors))
            if signed_turn and is_outgoing and corner_agreement and min(outgoing_dist)>2.:
                event='turn_complete'
            elif actual is not None and abs(actual)>math.radians(45) and not is_outgoing and any(near_corner):
                event='wrong_turn'
            elif incoming and on_incoming and all(passed):
                event='missed_turn'
            elif signed_turn and is_outgoing and all(d>max(8.,e+3.) for d,e in zip(distances,errors)):
                event='early_turn'
            else:
                event=None
            confirmed=self.confirm(event,t,pose[:2])
            if confirmed == 'turn_complete':
                self.completed.append(self.next_turn);self.next_turn+=1;self.latched=None
                self.progress=max(self.progress,turn.progress)
                self._event=None
                result.update(state='turn_complete',message=f'已{turn.direction}，继续前行',completed_turns=list(self.completed))
            elif confirmed:
                text={'missed_turn':'可能已走过转弯路口，请核对路线',
                      'wrong_turn':'当前行进方向与路线不符，请核对岔路',
                      'early_turn':'可能提前转弯，请核对路口'}[confirmed]
                result.update(state=confirmed,message=text,replan_available=True)
            elif near:
                ambiguous = hi-lo > 18. or (lo < -5. and hi > 5.)
                result.update(state='junction_uncertain' if ambiguous else 'approaching',
                    message=f'接近{turn.direction}路口，请留意路口位置' if ambiguous else
                            (f'前方约 {max(0.,(lo+hi)/2):.0f} 米{turn.direction}' if lo>3. else f'到路口{turn.direction}'))
            elif hi < -5.:
                result.update(state='junction_uncertain',message='路口位置待确认，请核对路线')
        else:
            end=[np.linalg.norm(p-self.route.points[-1]) for p,_,_ in options]
            if self.progress >= self.route.s[-1]-12. and all(d <= max(8.,2*s) for d,(_,s,_) in zip(end,options)):
                result.update(state='near_destination',message='已接近目的地，请核对入口')
                if all(d<=4. and s<=3. for d,(_,s,_) in zip(end,options)):
                    if self.confirm('arrived',t,pose[:2],require_motion=False):
                        result.update(state='arrived',message='已到达目的地')
        if result['state']=='continue' and (not valid or min(p[1] for p in valid)>self.route.corridor_m+max(s for _,s,_ in options)*2):
            if self.confirm('off_route',t,pose[:2]):
                result.update(state='off_route',message='已偏离规划路线，可从当前位置重新规划',replan_available=True)
        if result['state'] in ('missed_turn','wrong_turn','early_turn','off_route'):
            self.deviation=(result['state'],result['message'])
        elif self.deviation:
            recovered=result['state']=='turn_complete'
            if turn is not None and heading is not None:
                recovered |= (all(d>5. for d in distances) and incoming
                              and bool(valid) and max(p[1] for p in valid)<self.route.corridor_m)
            elif turn is None and heading is not None and valid:
                direction=self.route.at(self.progress+3)-self.route.at(self.progress-3)
                recovered |= (max(p[1] for p in valid)<self.route.corridor_m
                              and abs(wrap(angle(heading)-angle(direction)))<math.radians(30))
            if recovered:
                self.deviation=None
            else:
                result.update(state=self.deviation[0],message=self.deviation[1],replan_available=True)
        if self._requested_event is None:
            self._event=None
        self._last=result
        return result

    def confirm(self,event,t,xy,require_motion=True):
        if event is None:
            return None
        self._requested_event=event
        if self._event != event:
            self._event,self._event_since,self._event_origin=event,t,xy.copy()
            return None
        if t-self._event_since >= 1. and (not require_motion or np.linalg.norm(xy-self._event_origin)>=1.):
            return event
        return None

"""Outdoor route/session state. WGS84 observations never become BD09 robot goals.

Thread safe, ROS/Qt independent. A saved geographic origin survives a restart;
an odometry alignment deliberately does not. No TF or motion commands are emitted.
"""
from collections import deque
from pathlib import Path
import copy
import json
import math
import os
import threading
import time
import uuid
from .geo_maps import GeographicNavigation
from .journey import JourneyCommands
from .gnss_alignment import alignment_configuration_ready


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def coordinate(p):
    return (isinstance(p, (list, tuple)) and len(p) == 2
            and all(finite(v) for v in p) and abs(p[0]) <= 180 and abs(p[1]) < 85)


def distance(a, b):
    """Short-range spherical distance; BD09 values are display guidance only."""
    dlat, dlon = map(math.radians, (b[1] - a[1], b[0] - a[0]))
    h = math.sin(dlat / 2)**2 + math.cos(math.radians(a[1])) * math.cos(math.radians(b[1])) * math.sin(dlon / 2)**2
    return 12742000 * math.asin(min(1., math.sqrt(max(0., h))))


def route_waypoint_coverage(points, waypoints):
    """Measure the provider geometry against the selections without moving either.

    Ends must match the corresponding route endpoints, not an earlier pass of a
    loop. For intermediate points this is a nearest-polyline coverage check; it
    does not claim that the provider visited them in the intended order.
    """
    rows, warnings = [], []
    for index, selected in enumerate(waypoints):
        role = 'start' if index == 0 else 'destination' if index == len(waypoints)-1 else 'via'
        method = 'route_endpoint'
        if role != 'via':
            matched = points[0] if role == 'start' else points[-1]
        else:
            method = 'nearest_route_segment'
            scale = 111195 * math.cos(math.radians(selected[1]))
            candidates = []
            for a, b in zip(points, points[1:]):
                ax, ay = (a[0]-selected[0])*scale, (a[1]-selected[1])*111195
                dx, dy = (b[0]-a[0])*scale, (b[1]-a[1])*111195
                rate = max(0., min(1., -(ax*dx+ay*dy)/max(1e-12, dx*dx+dy*dy)))
                candidates.append((math.hypot(ax+rate*dx, ay+rate*dy),
                                   [a[j]+rate*(b[j]-a[j]) for j in (0, 1)]))
            matched = min(candidates, key=lambda value: value[0])[1]
        gap = distance(selected, matched)
        rows.append(dict(waypoint_index=index, role=role, offset_m=round(gap, 1),
                         matched_point=list(matched), method=method))
        if gap > 8.:
            name = '起点' if role == 'start' else '终点' if role == 'destination' else f'途经点{index}'
            warnings.append(dict(code='waypoint_not_covered', waypoint_index=index, role=role,
                offset_m=round(gap, 1),
                message=f'百度路线{name}离所选位置约{gap:.0f}米，可调整选点或手绘'))
    return dict(source='selected_waypoints_vs_baidu_polyline', coordinate_system='BD09',
                warning_threshold_m=8., measurements=rows, warnings=warnings,
                start_offset_m=rows[0]['offset_m'], end_offset_m=rows[-1]['offset_m'])


def ecef(lon, lat, altitude=0.):
    lon, lat = math.radians(lon), math.radians(lat)
    n = 6378137. / math.sqrt(1 - 6.69437999014e-3 * math.sin(lat)**2)
    return ((n + altitude) * math.cos(lat) * math.cos(lon),
            (n + altitude) * math.cos(lat) * math.sin(lon),
            (n * (1 - 6.69437999014e-3) + altitude) * math.sin(lat))


class GeoOrigin:
    def __init__(self, lon, lat, altitude=0.):
        self.values = [lon, lat, altitude]
        self.xyz = ecef(*self.values)
        lon, lat = math.radians(lon), math.radians(lat)
        self.rotation = ((-math.sin(lon), math.cos(lon), 0.),
                         (-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)),
                         (math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)))

    def enu(self, lon, lat, altitude=0.):
        delta = [a-b for a, b in zip(ecef(lon, lat, altitude), self.xyz)]
        return tuple(sum(a*b for a, b in zip(row, delta)) for row in self.rotation)

    def wgs84(self, east, north, up=0.):
        xyz = [self.xyz[j] + sum(self.rotation[i][j]*v for i, v in enumerate((east, north, up))) for j in range(3)]
        x, y, z = xyz
        rho = math.hypot(x, y)
        lat = math.atan2(z, rho * (1 - 6.69437999014e-3))
        for _ in range(8):
            n = 6378137. / math.sqrt(1 - 6.69437999014e-3 * math.sin(lat)**2)
            lat = math.atan2(z + 6.69437999014e-3*n*math.sin(lat), rho)
        return [math.degrees(math.atan2(y, x)), math.degrees(lat)]


def fit_alignment(pairs, minimum_spread=4., maximum_residual=.8, maximum_relative=.08, weights=None):
    """Unscaled planar Procrustes: odom antenna -> geographic ENU."""
    count = len(pairs)
    if count < 8:
        return None
    local = [p[0] for p in pairs]
    world = [p[1] for p in pairs]
    weights=list(weights) if weights is not None else [1.]*count
    if len(weights)!=count or any(not finite(w) or w<=0 for w in weights):return None
    mass=sum(weights)
    a = [sum(w*p[i] for w,p in zip(weights,local))/mass for i in range(2)]
    b = [sum(w*p[i] for w,p in zip(weights,world))/mass for i in range(2)]
    spread = math.sqrt(sum(w*((p[0]-a[0])**2+(p[1]-a[1])**2) for w,p in zip(weights,local))/mass)
    if spread < minimum_spread:
        return None
    c = sum(w*((p[0]-a[0])*(q[0]-b[0])+(p[1]-a[1])*(q[1]-b[1])) for w,p,q in zip(weights,local,world))
    s = sum(w*((p[0]-a[0])*(q[1]-b[1])-(p[1]-a[1])*(q[0]-b[0])) for w,p,q in zip(weights,local,world))
    yaw = math.atan2(s, c)
    co, si = math.cos(yaw), math.sin(yaw)
    tx, ty = b[0]-co*a[0]+si*a[1], b[1]-si*a[0]-co*a[1]
    residual = math.sqrt(sum(w*((co*p[0]-si*p[1]+tx-q[0])**2+(si*p[0]+co*p[1]+ty-q[1])**2) for w,p,q in zip(weights,local,world))/mass)
    # A wrong scale, long outlier or curved mismatch must not initialize heading.
    if residual > maximum_residual or residual / spread > maximum_relative:
        return None
    return dict(yaw=yaw, tx=tx, ty=ty, residual_m=residual, spread_m=spread, pairs=count)


class OutdoorState(GeographicNavigation, JourneyCommands):
    def __init__(self, directory=None, clock=time.time, calibration=None, monotonic_clock=time.monotonic):
        self.lock = threading.RLock()
        self.clock = clock
        self.monotonic_clock = monotonic_clock
        self.directory = Path(directory).expanduser() if directory else None
        self.calibration = calibration or {}
        self.route = None
        self.route_provenance = None
        self.origin = None
        self.alignment = None
        self.odom_continuity = 0
        self.odom_distance = 0.
        self.odom_instance = None
        self.alignment_conflict = False
        self.motion_status = ""
        self.guidance_generation = 0
        self.pairs = deque(maxlen=120)
        self.odom = deque(maxlen=400)
        self.odom_received_at = None
        self.odom_stream_received_at = None
        self.odom_stream_stamp = None
        self._alignment_pending = deque(maxlen=256)
        self.track = deque(maxlen=3600)  # One sample/second, bounded on small device.
        self.track_bd09 = deque(maxlen=1800)
        self.observation = None
        self.gnss_source = None
        self.phone_motion_enabled = False
        self.phone_motion = None
        self.phone_motion_sequence = None
        self.phone_alignment_generation = 0
        self.phone_motion_support_lost_at = None
        self.observation_reason = "等待卫星定位"
        self.alignment_reason = "等待位置与里程计"
        self.position = None
        self.position_id = 0
        self.converted = None
        self.conversion_error = ""
        self.guiding = False
        self.local_navigation_active = False
        self.route_progress = 0.
        self.progress_stamp = 0.
        self.last_pair_stamp = 0.
        self.last_accepted = None
        self._pending_covariance_fix = None
        self.last_track_stamp = 0.
        self.session_id = uuid.uuid4().hex
        self.assets = {}
        self.save_error = ""
        self.mode = "walking"
        self.simulation = None  # Volatile UI rehearsal; never a sensor observation.
        self.sensor_replay = False
        from .navigation_trial import NavigationTrial
        self.trial = NavigationTrial()
        from .phone_absolute_runtime import PhoneAbsoluteRuntime
        self.phone_absolute=PhoneAbsoluteRuntime()
        from .route_assistance import RouteAssistance
        self.route_assist = RouteAssistance()
        self.init_geographic()
        self.init_journey()
        self._load()

    def _load(self):
        if not self.directory:
            return
        try:
            p = self.directory / "active.json"
            if not p.exists():
                return
            if p.stat().st_size > 2_000_000:
                raise ValueError()
            data = json.loads(p.read_text(encoding="utf-8"))
            if data.get("schema") != 1:
                raise ValueError()
            if data.get("route"):
                self.route = self.validate_route(data["route"])
                context = data.get('route_context')
                if (isinstance(context, dict) and context.get('route_id') == self.route['id']
                        and isinstance(context.get('planned_session_id'), str)
                        and finite(context.get('planned_at'))):
                    self.route_provenance = {key: context[key] for key in
                        ('route_id', 'planned_session_id', 'planned_at')}
            policy=data.get('alignment_quality_policy')
            if policy in ('fixed','fixed_float'):
                self.trial.quality_policy=policy
            self.route_assist.set_enabled(data.get('route_assistance_enabled') is True)
            self.phone_motion_enabled=data.get('phone_motion_enabled') is True
            origin = data.get("origin_wgs84")
            if origin and len(origin) == 3 and coordinate(origin[:2]) and finite(origin[2]):
                self.origin = GeoOrigin(*origin)
            identifier = data.get('selected_map_id')
            if identifier in self.maps.entries:
                self.selected_map = self.maps.get(identifier)
                self._cache_saved_map_track(self.maps.get(identifier, full=True))
                self.map_reason = '已恢复地图选择，等待本次启动后的重新定位'
            # Old odom transforms/tracks are historical, never resumed as live pose.
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            self.save_error = "上次路线无法读取，请重新选择路线"

    @staticmethod
    def validate_route(data):
        if (not isinstance(data, dict) or data.get("coordinate_system") != "BD09"
                or data.get("source") not in ("baidu_walking", "manual")):
            raise ValueError("路线坐标或来源无效")
        points = data.get("points")
        if not isinstance(points, list) or not 2 <= len(points) <= 2000 or not all(coordinate(p) for p in points):
            raise ValueError("请选择至少两个路线点，最多 2000 个")
        total = sum(distance(a, b) for a, b in zip(points, points[1:]))
        if not 1 <= total <= 50000:
            raise ValueError("路线长度应在 1 米至 50 公里之间")
        title = data.get("title", "步行路线")
        if not isinstance(title, str) or not 1 <= len(title.strip()) <= 80:
            raise ValueError("路线名称应为 1–80 个字")
        result = dict(title=title.strip(), points=points, coordinate_system="BD09", source=data["source"],
                    distance_m=round(total, 1), id=data.get("id") if isinstance(data.get("id"), str) and len(data["id"]) <= 64 else uuid.uuid4().hex)
        via=data.get('waypoints')
        if isinstance(via,list) and 2 <= len(via) <= 20 and all(coordinate(p) for p in via):
            result['waypoints']=copy.deepcopy(via)
            if result['source']=='baidu_walking':
                # Recompute on load/validation too. Provider strings or stale
                # persisted warnings cannot override the measured geometry.
                result['coverage']=route_waypoint_coverage(points,via)
        return result

    def set_route(self, data):
        route = self.validate_route(data)
        with self.lock:
            if self.guiding or self.simulation or self.journey.active:
                raise ValueError("请先结束当前引导再更换路线")
            previous = (self.route, self.route_provenance, self.session_id, self.track, self.track_bd09,
                        self.assets, self.local_track, self.estimated_track)
            if not self.trial.active and (self.track or self.local_track or self.estimated_track or self.track_bd09):
                try:
                    self._save(archive=True)
                except OSError:
                    raise ValueError("旧轨迹未保存，请检查磁盘空间") from None
            self.route = copy.deepcopy(route)
            if not self.trial.active:
                self.session_id = uuid.uuid4().hex
                self.track = deque(maxlen=3600)
                self.track_bd09 = deque(maxlen=1800)
                self.assets = {}
                self.local_track = []
                self.estimated_track = []
            self.route_provenance = dict(route_id=self.route['id'], planned_session_id=self.session_id,
                                         planned_at=self.clock())
            try:
                self._save()
            except OSError:
                (self.route, self.route_provenance, self.session_id, self.track, self.track_bd09,
                 self.assets, self.local_track, self.estimated_track) = previous
                raise ValueError("路线未保存，请检查磁盘空间") from None
            self.route_progress = 0.
            return copy.deepcopy(route)

    def begin_capture_session(self):
        """Give every ordinary capture its own async-save ownership too."""
        with self.lock:
            if self.trial.active:
                return self.session_id
            if self.assets or self.track or self.track_bd09 or self.local_track or self.estimated_track:
                self.save_session()
            previous = (self.session_id, self.assets, self.track, self.track_bd09,
                        self.local_track, self.estimated_track)
            global_fields = ('origin', 'alignment', 'alignment_conflict', 'position', 'last_accepted',
                             '_pending_covariance_fix', 'previous_output', 'converted', 'pending_pose',
                             'position_conflict', 'pose_epoch', 'conversion_error', 'last_pair_stamp',
                             'last_track_stamp', 'pairs', '_alignment_pending', 'alignment_reason')
            previous_global = {key: getattr(self, key) for key in global_fields}
            self.session_id = uuid.uuid4().hex
            self.assets = {}
            self.track = deque(maxlen=3600)
            self.track_bd09 = deque(maxlen=1800)
            self.local_track = []
            self.estimated_track = []
            # A new capture has a new odometry frame and may start kilometres
            # from the saved restart origin. Only the first accepted observation
            # of this session may establish its ENU reference. Joining a trial
            # later preserves that reference and the tracks already expressed in it.
            self.origin = self.alignment = self.position = self.last_accepted = None
            self._pending_covariance_fix = self.previous_output = self.converted = self.pending_pose = None
            self.alignment_conflict = self.position_conflict = False
            self.pose_epoch += 1
            self.conversion_error = ''
            self.last_pair_stamp = self.last_track_stamp = 0.
            self.pairs = deque(maxlen=120)
            self._alignment_pending = deque(maxlen=256)
            self.alignment_reason = '新行程等待位置与方向确认'
            try:
                self._save()
            except (OSError, ValueError):
                (self.session_id, self.assets, self.track, self.track_bd09,
                 self.local_track, self.estimated_track) = previous
                for key, value in previous_global.items():setattr(self, key, value)
                self.save_error = '采集记录未能创建，请检查磁盘空间'
                raise ValueError(self.save_error) from None
            return self.session_id

    def route_context(self):
        """A reusable route is not evidence that this capture completed alignment."""
        if not self.route:
            return None
        value = copy.deepcopy(self.route_provenance or dict(route_id=self.route['id'],
            planned_session_id=None, planned_at=None))
        value['planned_in_this_session'] = value['planned_session_id'] == self.session_id
        value['status'] = ('planned_this_session' if value['planned_in_this_session'] else
                           'retained_previous_route' if value['planned_session_id'] else 'legacy_origin_unknown')
        return value

    def _document(self):
        return dict(schema=1, session_id=self.session_id, saved_at=self.clock(), route=self.route,
                    route_context=self.route_context(),
                    route_assistance_enabled=self.route_assist.enabled,
                    alignment_quality_policy=self.trial.quality_policy,
                    phone_motion_enabled=self.phone_motion_enabled,
                    origin_wgs84=self.origin.values if self.origin else None, track=list(self.track),
                    alignment=self.alignment, calibration=self.calibration, assets=self.assets, track_bd09=list(self.track_bd09),
                    selected_map_id=self.selected_map['id'] if self.selected_map else None,
                    local_track=self.local_track, odom_epoch=self.odom_epoch,
                    odom_continuity=self.odom_continuity, odom_instance=self.odom_instance,
                    odom_frame=self.odom_frame,
                    estimated_track=self.estimated_track,
                    simulation_active=self.simulation is not None,
                    simulation_track_included=False,
                    navigation_trial=self.trial.snapshot(self), trial_events=list(self.trial.events),
                    trial_route_odom=self.trial.route_in_odom(self),
                    odom_alignment_reusable=False, coordinate_contract="track=WGS84; route=BD09; alignment=ENU_from_odom")

    def export_document(self):
        """Export current observations plus the explicitly selected map history.

        Restarted state deliberately has no live trajectory. Saved history is
        exported in its own namespace so it cannot be mistaken for new fixes.
        """
        with self.lock:
            result = copy.deepcopy(self._document())
            identifier = self.selected_map['id'] if self.selected_map else None
        # Archive I/O and historical reprojection must not hold the live pose
        # lock while a phone downloads a long recording's export.
        if identifier:
            result['saved_map_track'] = self.read_saved_map_track(identifier)
        return result

    def _save(self, archive=False):
        if not self.directory:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        target = self.directory / (f"session-{self.session_id}.json" if archive else "active.json")
        tmp = target.with_suffix(".tmp")
        document=self._document()
        if not archive:
            # Active state is a small restart index; complete history lives in
            # the session archive and must never make startup exceed its limit.
            for key in ('track','track_bd09','local_track','estimated_track'):
                document[key]=document[key][-60:]
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(document, f, ensure_ascii=False, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, target)
        self.save_error = ""

    def save_session(self):
        with self.lock:
            try:
                self._save(archive=True)
                self._save()
            except OSError:
                self.save_error = "轨迹保存失败，请检查磁盘空间"
                raise ValueError(self.save_error) from None

    def set_route_assistance(self, enabled):
        if not isinstance(enabled, bool):
            raise ValueError('路口辅助开关状态无效')
        with self.lock:
            old = self.route_assist.enabled
            if old == enabled:
                return
            self.route_assist.set_enabled(enabled)
            try:
                self._save()
            except OSError:
                self.route_assist.set_enabled(old)
                raise ValueError('开关设置未保存，请检查存储空间') from None

    def control(self, action):
        with self.lock:
            if self.simulation:
                if action == "start":
                    self.simulate("walk")
                elif action == "stop":
                    self.simulate("exit")
                elif action == "save":
                    raise ValueError("当前为模拟演示，没有可保存的实走轨迹")
                else:
                    raise ValueError("不支持此操作")
                return
            if action == "start":
                if self.local_navigation_active or self.journey.active:
                    raise ValueError("请先结束当前地图导航，再开始另一条路线")
                if self.mode != "walking":
                    raise ValueError("路网路线仅供步行引导，请先切换到手持模式")
                if not self.route:
                    raise ValueError("请先选择并保存路线")
                if self.trial.active:
                    if not self.trial.snapshot(self)['aligned']:
                        raise ValueError('请先平稳行走完成初始对准；建图和录制继续进行')
                    if not self.trial.route_in_odom(self):
                        raise ValueError('路线坐标尚未就绪，请检查地图联网并在起点1公里内选点')
                if not self._position_now() or not self.converted or self.clock()-self.converted["stamp"] > 4:
                    raise ValueError("当前位置未确认，请等待地图定位或卫星定位")
                if distance(self.converted["point"], self.route["points"][0]) > 50:
                    raise ValueError("当前位置距离起点较远，请以当前位置重新规划")
                self.route_progress = 0.
                self.progress_stamp = self.clock()
                self.guiding = True
                self.guidance_generation += 1
                self.requests.append(dict(action='road_tracking_start', generation=self.guidance_generation))
            elif action == "stop":
                self.guiding = False
                self.guidance_generation += 1
                self.requests.append(dict(action='road_tracking_stop', generation=self.guidance_generation))
                self.save_session()
            elif action == "save":
                self.save_session()
            else:
                raise ValueError("不支持此操作")

    def simulate(self, action):
        with self.lock:
            if action != 'exit' and (self.local_navigation_active or self.journey.active):
                raise ValueError('请先结束当前地图导航，再开始模拟')
            if action == "exit":
                self.simulation = None
                self.guiding = False
                return
            if self.mode != "walking":
                raise ValueError("请切换到手持模式后进行模拟")
            if action == "enter":
                if self.guiding or self.simulation:
                    raise ValueError("请先结束当前引导或退出模拟")
                if not self.route:
                    raise ValueError("请先选择并保存路线")
                self.simulation = RouteSimulation(self.route, self.clock)
            elif self.simulation:
                self.simulation.control(action)
            else:
                raise ValueError("请先进入模拟演示")

    def feed_odom(self, stamp, position, quaternion):
        if not all(finite(v) for v in [stamp, *position, *quaternion]) or len(position) != 3 or len(quaternion) != 4:
            return
        with self.lock:
            now = self.clock()
            norm = math.sqrt(sum(q*q for q in quaternion))
            # Receipt activity and pose freshness answer different questions.
            # Count only advancing, finite, plausible-time measurements; a
            # repeated timestamp must not keep a stopped estimator alive.
            if (stamp <= now+.25 and .9 < norm < 1.1 and
                    (self.odom_stream_stamp is None or stamp > self.odom_stream_stamp)):
                self.odom_stream_stamp = stamp
                self.odom_stream_received_at = self.monotonic_clock()
            # Measurement history and a live navigation pose have different
            # freshness requirements. Dropping every delayed LIO state created
            # an artificial timestamp gap, then reset the otherwise continuous
            # estimator when its queue caught up. Display/guidance still require
            # a pose within 0.5 s; this bounded history is only for interpolation.
            if not -.25 <= now-stamp <= 10.:
                return
            # Repeated/out-of-order delivery is not evidence of a new estimator.
            # The diagnostics instance identity handles actual process restarts.
            if self.odom and stamp <= self.odom[-1][0]:
                return
            if self.odom and math.dist(position, self.odom[-1][1]) > max(2., (stamp-self.odom[-1][0])*5):
                self.reset_odometry('运动跟踪位置跳变，请重新确认位置与方向')
            if not .9 < norm < 1.1:
                self._alignment_pending.clear()
                self.odom_continuity += 1
                if self.trial.active:
                    self.alignment=None;self.trial.reset_tracking(self, '运动跟踪姿态异常，等待恢复')
                return
            if self.odom:
                if stamp-self.odom[-1][0] > 1.:
                    self._alignment_pending.clear()
                    # A delivery gap is not a new coordinate frame. A known
                    # estimator identity, unchanged frames (checked by the ROS
                    # adapter), valid attitude and plausible displacement keep
                    # the geographic transform. Interpolation still rejects
                    # gaps over 150 ms in _antenna_at; no missing poses are made up.
                    if self.odom_instance is None:
                        self.odom_continuity += 1
                        self.alignment_reason = '运动跟踪曾中断，等待卫星或地图重新确认'
                        if self.trial.active:
                            self.alignment=None;self.trial.reset_tracking(self, '运动跟踪数据出现间隙，请重新确认位置与方向')
                self.odom_distance += math.dist(position, self.odom[-1][1])
            self.odom.append((stamp, list(position), [q/norm for q in quaternion]))
            self.odom_received_at = now
            # reset_odometry may have cleared receipt fields after a jump.
            self.odom_stream_stamp = stamp
            self.odom_stream_received_at = self.monotonic_clock()
            self._drain_alignment_pending()
            self.trial.phone_translation.tick(self)
            self.phone_absolute.feed_pose(self)

    def feed_odom_instance(self, instance):
        if not isinstance(instance, str) or not instance:
            return
        with self.lock:
            if self.odom_instance is not None and instance != self.odom_instance:
                self.reset_odometry()
            self.odom_instance = instance

    def reset_odometry(self, reason="里程计已重启，方向需要重新对齐"):
        with self.lock:
            self.odom.clear()
            self.odom_received_at = None
            self.odom_stream_received_at = None
            self.odom_stream_stamp = None
            self._alignment_pending.clear()
            self.pairs.clear()
            self.alignment = None
            self.odom_continuity += 1
            self.odom_distance = 0.
            self.alignment_conflict = False
            self.last_pair_stamp = 0.
            self.alignment_reason = reason
            self.odom_epoch += 1
            self.clear_map_match()
            self.local_track = []
            if self.trial.active:
                self.trial.reset_tracking(self, reason)

    def _observation_calibration(self, data):
        """Keep phone co-carried trajectory assumptions separate from the receiver."""
        if data.get('source') in ('phone_software_rtk','phone_continuity'):
            if not self.trial.active:
                return None  # No phone-specific measured installation is configured.
            return dict(antenna_in_body_m=[0., 0., 0.], gnss_to_odom_offset_s=0.,
                        installation_policy='phone_co_carried_trajectory',
                        timing_policy='phone_utc_interpolated_odom',
                        lever_arm_verified=False, time_verified=False,
                        antenna_uncertainty_m=None)
        return self.calibration if alignment_configuration_ready(self.calibration) else None

    def set_phone_motion(self, enabled):
        if type(enabled) is not bool:raise ValueError('请选择是否使用手机融合定位')
        with self.lock:
            if enabled==self.phone_motion_enabled:return
            if self.guiding:raise ValueError('请先结束引导，再更换定位来源')
            self.phone_motion_enabled=enabled
            self._alignment_pending.clear();self.pairs.clear()
            self.last_accepted=None;self.position=None
            self.phone_alignment_generation+=1
            self.trial.reset_tracking(self,'已更换手机定位来源，重新核对轨迹；运动跟踪继续',
                event_kind='alignment_source_changed')
            # The previous, verified ENU_from_odom still describes the same
            # LiVO frame. Keep it for coasting, but never renew it from one
            # sample of the newly selected source.
            self._save()

    def feed_phone_motion(self, data):
        """Keep the estimate and origin changes distinct from physical motion."""
        from .phone_observation import number,motion_observation
        if (not isinstance(data,dict) or data.get('source')!='phone_continuity'
                or data.get('schema_version')!=1 or data.get('absolute_accuracy_verified') is not False
                or not number(data.get('utc_ms')) or not isinstance(data.get('continuity_id'),str)
                or type(data.get('origin_revision')) is not int):return
        with self.lock:
            key=(data.get('source_id'),data.get('session_id'),data.get('sequence'))
            if key==self.phone_motion_sequence:return
            stamp=data['utc_ms']/1000
            if not 0<=self.clock()-stamp<=10. or data.get('fresh') is not True:return
            old=self.phone_motion
            self.phone_motion=copy.deepcopy(data);self.phone_motion_sequence=key
            # Both phone streams remain available for diagnostics/recording.
            # Only the explicitly selected fusion stream may invalidate its
            # own fit. A background continuity recovery must not erase native
            # RTK pairs or change a manually confirmed geographic reference.
            if not (self.phone_motion_enabled and self.trial.active and self.trial.rtk_enabled):
                self.phone_motion_support_lost_at=None
                return
            identity=lambda row:(row.get('source_id'),row.get('session_id'),row.get('continuity_id'),
                                 None if row.get('raw_constraints') else row.get('origin_revision'))
            changed=bool(old and (identity(old)!=identity(data)
                or (not data.get('raw_constraints') and data.get('coarse_origin_update') is True)
                or data.get('native_error_reinitialized') is True))
            if changed:self.phone_motion_support_lost_at=None
            # An observed endpoint cannot make the intervening extrapolated
            # edge a measurement. Never join that edge to an old fit window.
            changed=changed or bool(old and not data.get('raw_constraints') and (data.get('delta_observed') is False
                or (number(data.get('gap_seconds')) and data['gap_seconds']>0)))
            support=motion_observation(data,self.clock()) is not None
            if not support:
                if self.phone_motion_support_lost_at is None:self.phone_motion_support_lost_at=stamp
            else:
                changed=changed or bool(self.phone_motion_support_lost_at is not None
                    and stamp-self.phone_motion_support_lost_at>2.)
                self.phone_motion_support_lost_at=None
            self.phone_absolute.feed_motion(self,data)
            if changed:
                self.phone_alignment_generation+=1
                if self.gnss_source and self.gnss_source[0]=='phone_continuity':
                    self._alignment_pending.clear();self.pairs.clear();self.last_accepted=None
                    self.trial.reset_tracking(self,'手机定位已重新收敛，正在核对地理位置；运动跟踪继续',
                        preserve_absolute=self.phone_absolute.owns_alignment(self,(data.get('source_id'),data.get('session_id'))),
                        event_kind='phone_motion_discontinuity')
            self.trial.phone_translation.feed(self,data)

    def phone_motion_snapshot(self):
        row=self.phone_motion or {}
        stamp=row.get('utc_ms',0)/1000
        fresh=bool(row and -.05<=self.clock()-stamp<=1.5)
        status=row.get('status','unavailable') if fresh else 'unavailable'
        reason=('等待手机融合定位' if not row else '手机融合定位暂缺，运动跟踪接续' if not fresh
                else '手机位置正在重新收敛，暂不更新地理对齐' if row.get('coarse_origin_update') or row.get('native_error_reinitialized')
                else '手机运动观测不完整，结合雷达核对方向' if status=='predicted' and row.get('raw_scalar_observed') is True
                else '手机运动暂为预测，暂不更新地理对齐' if status!='observed'
                else '手机融合位置可用，仍需轨迹对齐与精度核验')
        return dict(enabled=self.phone_motion_enabled,fresh=fresh,status=status,reason=reason,
            absolute_window=self.phone_absolute.snapshot(),
            raw_scalar_observed=row.get('raw_scalar_observed'),motion_support=row.get('motion_support'),
            horizontal_sigma_m=row.get('horizontal_sigma_m'),local_horizontal_sigma_m=row.get('local_horizontal_sigma_m'),
            native_quality=row.get('native_quality'),absolute_accuracy_verified=False,
            continuity_id=row.get('continuity_id'),origin_revision=row.get('origin_revision'))

    def _antenna_at(self, stamp, calibration=None):
        # Linear position and normalized shortest quaternion interpolation; no
        # extrapolation across missing odometry. Lever arm is in odom child frame.
        calibration = self.calibration if calibration is None else calibration
        rows = list(self.odom)
        for a, b in zip(rows, rows[1:]):
            if a[0] <= stamp <= b[0] and 0 < b[0]-a[0] <= .15:
                t = (stamp-a[0])/(b[0]-a[0])
                pos = [x+(y-x)*t for x,y in zip(a[1],b[1])]
                sign = 1 if sum(x*y for x,y in zip(a[2],b[2])) >= 0 else -1
                q = [x*(1-t)+sign*y*t for x,y in zip(a[2],b[2])]
                norm = math.sqrt(sum(x*x for x in q))
                x,y,z,w = [v/norm for v in q]
                lx,ly,lz = calibration.get("antenna_in_body_m", [0,0,0])
                offset = [(1-2*y*y-2*z*z)*lx+(2*x*y-2*z*w)*ly+(2*x*z+2*y*w)*lz,
                          (2*x*y+2*z*w)*lx+(1-2*x*x-2*z*z)*ly+(2*y*z-2*x*w)*lz,
                          (2*x*z-2*y*w)*lx+(2*y*z+2*x*w)*ly+(1-2*x*x-2*y*y)*lz]
                return tuple(a+b for a,b in zip(pos,offset))
        return None

    def feed_fix(self, data):
        with self.lock:
            if self.sensor_replay:
                return
            self.observation = copy.deepcopy(data)
            now = self.clock()
            stamp = data.get("stamp")
            p = [data.get("longitude"), data.get("latitude")]
            if data.get("coordinate_system", "WGS84") != "WGS84":
                self.observation_reason = "差分坐标系尚未转换为 WGS84"
                return
            from .phone_observation import alignment_quality
            quality=alignment_quality(data)
            fusion=data.get('source')=='phone_continuity'
            if (not fusion and quality not in (1,2,4,5)) or not coordinate(p):
                self.observation_reason = "卫星无有效定位"
                return
            from nav_rtk.parser.observation_time import time_state
            from .phone_observation import historical_time_ready
            phone = data.get('source') in ('phone_software_rtk', 'phone_continuity')
            timing = time_state(data, now)
            if not (historical_time_ready(data, now) if phone else timing == 'ready'):
                self.observation_reason = {
                    'clock_mismatch': '已收到坐标，但设备与卫星时间未对齐，暂停导航定位',
                    'unavailable': '尚未取得完整卫星时间，暂停导航定位',
                    'waiting': '等待设备时间追齐卫星测量时间',
                    'stale': '卫星定位数据已过期，等待新数据',
                }.get(timing, '卫星时间未确认或数据已过期')
                return
            source = data['source'] if phone else 'receiver'
            source = (source, data.get('source_id'), data.get('session_id')) if phone else (source,)
            if data.get('source')=='phone_continuity':source+= (data.get('continuity_id'),
                None if data.get('raw_constraints') else data.get('origin_revision'))
            if self.gnss_source != source:
                self.last_accepted = None
                self._pending_covariance_fix = None
                self._alignment_pending.clear()
                if self.gnss_source is not None:
                    self.trial.reset_tracking(self, '定位来源已切换，重新核对两条轨迹',
                        preserve_absolute=phone and self.phone_absolute.owns_alignment(self,(data.get('source_id'),data.get('session_id'))),
                        event_kind='alignment_source_changed')
                    self.pairs.clear()
                self.gnss_source = source
            if data.get("covariance_update") is True:
                pending = self._pending_covariance_fix
                precision_fields = {"covariance","horizontal_std_m","covariance_source","covariance_update"}
                unchanged = lambda item: {key:value for key,value in item.items() if key not in precision_fields}
                sigma = data.get("horizontal_std_m")
                if (pending is None or unchanged(data) != unchanged(pending)
                        or data.get("covariance_source") != "receiver_gst"
                        or not finite(sigma) or sigma <= 0):
                    self.observation_reason = "卫星数据重复或乱序"
                    return
                # This is accuracy for the accepted fix, not a new position.
                self._pending_covariance_fix = None
                self.observation_reason = "固定解" if data["fix_quality"] == 4 else "卫星粗定位"
                self._align_or_wait(data, p, data.get("altitude_ellipsoid"))
                return
            if self.last_accepted:
                dt = stamp-self.last_accepted[0]
                if dt <= 0:
                    self.observation_reason = "卫星数据重复或乱序"
                    return
                if data.get('source')!='phone_continuity' and distance(p,self.last_accepted[1]) > max(10.,15.*dt):
                    self.observation_reason = "卫星位置跳变，等待稳定"
                    return
            self.last_accepted = (stamp,p)
            self._pending_covariance_fix = (copy.deepcopy(data) if data.get("covariance") is None
                                            and data.get("horizontal_std_m") is None else None)
            self.position_id += 1
            self.position = dict(wgs84=p, stamp=stamp, id=self.position_id, source="phone_continuity" if data.get('source')=='phone_continuity' else "gnss", quality=data.get("fix_quality"))
            if data.get('source')=='phone_continuity':self.position.update(native_quality=quality,absolute_accuracy_verified=False,
                segment=[data.get(k) for k in ('source_id','session_id','continuity_id','origin_revision')])
            self.observation_reason = '手机融合位置，绝对精度待核验' if data.get('source')=='phone_continuity' else "固定解" if quality == 4 else "卫星粗定位"
            altitude = data.get("altitude_ellipsoid")
            eligible = (not self.trial.active or self.trial.accepts_quality(quality)) and (not fusion or data.get('absolute_position_observed') is True)
            if not self.origin and eligible and finite(altitude):
                self.origin = GeoOrigin(*p, altitude)
            elif not self.origin and eligible and self.trial.active:
                # This explicit trial estimates only horizontal SE(2).
                # Missing ellipsoid height must not block a 2-D walking route.
                self.origin = GeoOrigin(*p, 0.)
            self._align_or_wait(data, p, altitude)
            if stamp-self.last_track_stamp >= 1:
                self.track.append(copy.deepcopy(self.position))
                self.last_track_stamp = stamp

    def _align_or_wait(self, data, p, altitude):
        # GNSS commonly arrives before the LIO state for the same epoch.
        # Retain accepted observations until interpolation is possible instead
        # of making alignment depend on callback arrival order.
        self._drain_alignment_pending()
        self._align(data, p, altitude)
        calibration = self._observation_calibration(data)
        if calibration is None:
            return
        target = data['stamp'] + calibration.get('gnss_to_odom_offset_s', 0.)
        if not self.odom or target > self.odom[-1][0]:
            item = (copy.deepcopy(data), list(p), altitude)
            for i, old in enumerate(self._alignment_pending):
                if old[0]['stamp'] == data['stamp']:
                    self._alignment_pending[i] = item  # GST refresh, same fix.
                    break
            else:
                self._alignment_pending.append(item)

    def _drain_alignment_pending(self):
        if self.sensor_replay:
            self._alignment_pending.clear()
            return
        while self._alignment_pending:
            data, p, altitude = self._alignment_pending[0]
            # These observations already passed acquisition-time validation in
            # feed_fix. Let delayed LIO join them at their measurement epochs;
            # never relabel an old correction or pose as fresh for guidance.
            now = self.clock()
            if not 0 <= now-data['stamp'] <= 10.:
                self._alignment_pending.popleft()
                continue
            calibration = self._observation_calibration(data)
            if calibration is None:
                self._alignment_pending.popleft()
                continue
            target = data['stamp'] + calibration.get('gnss_to_odom_offset_s', 0.)
            if not self.odom or target > self.odom[-1][0]:
                break
            self._alignment_pending.popleft()
            if target >= self.odom[0][0]:
                self._align(data, p, altitude)

    def _align(self, data, p, altitude):
        calibration = self._observation_calibration(data)
        if calibration is None:
            self.alignment_reason = ("手机位置可用，请进入导航与建图完成方向对准"
                                     if data.get('source') in ('phone_software_rtk','phone_continuity') else
                                     "等待天线安装偏移与时间对齐确认")
            if self.trial.active:self.trial.reason=self.alignment_reason
            return
        if self.trial.active:
            self.trial.feed(self, data, p, altitude)
            self.alignment_reason=self.trial.reason
            return
        sigma = data.get("horizontal_std_m")
        age = data.get("differential_age_s")
        if data.get("fix_quality") != 4:
            self.alignment_reason = "等待 RTK 固定解"
            return
        if not finite(sigma) or sigma <= 0:
            self.alignment_reason = "卫星位置可用于路线引导；精度信息缺失，方向尚未对齐"
            return
        if sigma > .5:
            self.alignment_reason = "水平精度不足，等待 0.5 米以内观测"
            return
        if not finite(age) or not 0 <= age <= 5:
            self.alignment_reason = "差分龄期缺失或超过 5 秒，请检查差分连接"
            return
        if not finite(altitude) or not self.origin:
            self.alignment_reason = "等待有效椭球高与地理原点"
            return
        stamp = data["stamp"] + calibration.get("gnss_to_odom_offset_s", 0.)
        antenna = self._antenna_at(stamp, calibration)
        if antenna is None:
            self.alignment_reason = "等待同步里程计"
            return
        if stamp-self.last_pair_stamp < .5:
            return
        world = self.origin.enu(*p, altitude)
        if math.hypot(*world[:2]) > 50000:
            self.alignment_reason = "超出当前地图范围，请新建地理地图"
            return
        if self.alignment:
            a = self.alignment
            co,si = math.cos(a["yaw"]),math.sin(a["yaw"])
            error = math.hypot(co*antenna[0]-si*antenna[1]+a["tx"]-world[0],si*antenna[0]+co*antenna[1]+a["ty"]-world[1])
            if error > 3:
                self.alignment_conflict = True
                self.alignment_reason = "卫星与里程计冲突，已拒绝纠偏"
                return
            # A stationary device cannot refit yaw, but agreeing, high-quality
            # observations still renew the existing alignment's freshness.
            if error <= .8:
                self.alignment['stamp'] = data['stamp']
                self._anchor_odometry()
        self.pairs.append((antenna,world))
        self.last_pair_stamp = stamp
        alignment = fit_alignment(self.pairs)
        if alignment:
            # Reject abrupt rotation/translation changes before adopting refits.
            if self.alignment:
                old=self.alignment
                angle=math.atan2(math.sin(alignment["yaw"]-old["yaw"]),math.cos(alignment["yaw"]-old["yaw"]))
                if abs(angle) > math.radians(5) or math.hypot(alignment["tx"]-old["tx"],alignment["ty"]-old["ty"]) > 1:
                    self.alignment_reason="对齐修正过大，等待重新确认"
                    return
            self.alignment={**alignment,"stamp":data["stamp"],
                            "antenna_uncertainty_m":calibration.get('antenna_uncertainty_m', 0.),
                            "timing_policy":calibration.get('timing_policy', 'measured_offset'),
                            "installation_policy":calibration.get('installation_policy', 'measured_lever_arm'),
                            "tz":sum(w[2]-p[2] for p,w in self.pairs)/len(self.pairs)}
            self._anchor_odometry()
            self.alignment_reason="方向已对齐"
        else:
            self.alignment_reason="位置已确定，请在开阔处走一段以确认方向"

    def _anchor_odometry(self):
        # A delayed geographic observation refers to its measurement epoch,
        # not to the distance counter when its computation finishes.
        stamp=self.alignment.get('absolute_anchor_stamp',self.alignment['stamp'])
        anchor_point=self.phone_absolute.historical_pose(self.phone_absolute.poses,stamp*1000)
        if anchor_point is None:
            short=[dict(utc_ms=t*1000,position=p,epoch_id=self.odom_continuity) for t,p,_ in self.odom]
            anchor_point=self.phone_absolute.historical_pose(short,stamp*1000)
        if (anchor_point is None and self.alignment.get('geographic_anchor_pose_stamp')==stamp
                and self.alignment.get('odom_continuity')==self.odom_continuity):
            anchor_point=self.alignment.get('geographic_anchor_pose_m')
        measured=self.phone_absolute.historical_distance(self.phone_absolute.poses,stamp*1000)
        lower_bound=False
        if measured is None or measured>self.odom_distance:
            history=list(self.odom);walk=0.
            for a,b in zip(history,history[1:]):
                if b[0]>stamp:
                    fraction=min(1.,(b[0]-stamp)/(b[0]-a[0]))
                    walk+=fraction*math.dist(a[1],b[1])
            measured=max(0.,self.odom_distance-walk)
            lower_bound=bool(history and stamp<history[0][0])
        self.alignment.update(odom_continuity=self.odom_continuity,
                              distance_at_anchor_m=measured,
                              correction_distance_is_lower_bound=lower_bound,
                              geographic_anchor_pose_m=None if anchor_point is None else list(anchor_point),
                              geographic_anchor_pose_stamp=stamp,
                              anchor_source=list(self.gnss_source) if self.gnss_source else None,
                              phone_generation=self.phone_alignment_generation)
        self.alignment_conflict = False

    def _position_now(self):
        if self.sensor_replay:
            return None
        now=self.clock()
        lio = None
        if (self.alignment and self.origin and self.odom and not self.alignment_conflict
                and self.alignment.get('odom_continuity', self.odom_continuity) == self.odom_continuity
                and -.05 <= now-self.odom[-1][0] <= .5 and 0 <= now-self.alignment["stamp"]):
            a=self.alignment
            correction_stamp=a.get('absolute_anchor_stamp',a['stamp'])
            stamp,p,q=self.odom[-1]
            co,si=math.cos(a["yaw"]),math.sin(a["yaw"])
            wgs=self.origin.wgs84(co*p[0]-si*p[1]+a["tx"],si*p[0]+co*p[1]+a["ty"])
            heading=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))+a["yaw"]
            lio = dict(wgs84=wgs,stamp=stamp,id=f"lio-{self.odom_epoch}-{stamp:.6f}",source="gnss_lio",heading_enu_rad=heading,
                        coasting=now-correction_stamp>2,
                        correction_age_s=now-correction_stamp,
                        distance_since_correction_m=max(0.,self.odom_distance-a.get('distance_at_anchor_m',self.odom_distance)),
                        correction_distance_is_lower_bound=a.get('correction_distance_is_lower_bound',False),
                        accuracy_status='estimated' if now-a['stamp']>2 else 'aligned')
        gnss = copy.deepcopy(self.position) if self.position and -.05 <= now-self.position["stamp"] <= 1.5 else None
        if gnss and gnss.get('source')=='phone_continuity':gnss=None
        if self.trial.active:
            if not self.trial.rtk_enabled or (gnss and not self.trial.accepts_quality(gnss.get('quality'))):
                gnss=None
            if lio:
                lio['experimental']=True
                manual=bool(self.alignment.get('manual'))
                lio['source']='manual_lio' if manual else 'gnss_lio'
                lio['precision_source']='manual_map_selection' if manual else 'trajectory_agreement'
                if self.alignment.get('estimate_source')=='phone_continuity':
                    anchor=self.alignment.get('geographic_anchor_pose_m')
                    yaw_sigma=self.alignment.get('yaw_sigma_rad')
                    yaw_uncertainty=None;uncertainty_status='unknown_anchor'
                    if anchor is not None:
                        uncertainty_status='unknown_heading'
                        if finite(yaw_sigma) and 0<yaw_sigma<=math.pi:
                            yaw_uncertainty=math.dist(p[:2],anchor[:2])*yaw_sigma
                            uncertainty_status='estimated'
                    lio.update(precision_source=(self.alignment['heading_method'] if self.alignment.get('heading_method')
                                                else 'phone_continuity_trajectory'),
                        absolute_horizontal_sigma_m=self.alignment.get('absolute_horizontal_sigma_m'),
                        yaw_displacement_sigma_m=yaw_uncertainty,
                        yaw_displacement_uncertainty=uncertainty_status,
                        absolute_accuracy_verified=False,geographic_accuracy='approximate')
                    if self.alignment.get('absolute_anchor_source'):
                        lio.update(absolute_anchor_source=self.alignment['absolute_anchor_source'],
                            geographic_correction_total_m=self.alignment.get('geographic_correction_total_m',0.),
                            geographic_correction_identity=(repr(self.trial.phone_translation.tag)
                                if self.trial.phone_translation.applies(self) else None),
                            geographic_correction_remaining_m=self.alignment.get('geographic_correction_remaining_m',0.))
                lio['coasting']=lio['coasting'] or not self.trial.rtk_enabled
                lio['accuracy_status']='experimental'
        return self.arbitrate_position(lio, gnss)

    def odometry_timing(self):
        """Report output activity without promoting delayed poses to live ones."""
        with self.lock:
            age = self.clock()-self.odom[-1][0] if self.odom else None
            receipt_gap = (self.monotonic_clock()-self.odom_stream_received_at
                           if self.odom_stream_received_at is not None else None)
            fresh = age is not None and -.05 <= age <= .5
            progressing = receipt_gap is not None and 0 <= receipt_gap <= 2.
            return dict(fresh=fresh, delayed=bool(not fresh and progressing),
                        measurement_age_s=age, receipt_gap_s=receipt_gap,
                        stream_measurement_age_s=(self.clock()-self.odom_stream_stamp
                            if self.odom_stream_stamp is not None else None),
                        progressing_stream=progressing)

    def local_tracking_snapshot(self):
        """Metric odometry display, independent of GNSS and saved-map matching."""
        with self.lock:
            points = [v['position_m'][:2] for v in self.local_track
                      if v['epoch'] == self.odom_epoch
                      and v.get('continuity', self.odom_continuity) == self.odom_continuity]
            timing = self.odometry_timing()
            fresh = timing['fresh']
            heading = None
            current = None
            if fresh:
                _, p, q = self.odom[-1]
                current = p[:2]
                heading = math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
                points = points + [current]
            return dict(points=points, position=current, heading_rad=heading,
                        **timing, frame=self.odom_frame, epoch=self.odom_epoch)

    def update_conversion(self, position, bd09):
        if not coordinate(bd09):
            return
        with self.lock:
            if self.clock()-position["stamp"] > 4 or position.get('epoch', self.pose_epoch) != self.pose_epoch:
                return
            self.converted=dict(point=bd09,stamp=position["stamp"],id=position["id"],epoch=self.pose_epoch,source=position['source'])
            if not self.track_bd09 or position['stamp']-self.track_bd09[-1]['stamp']>=1.:
                self.track_bd09.append(copy.deepcopy(self.converted))
            self.conversion_error=""

    def snapshot(self):
        with self.lock:
            if self.mode != "walking" and self.simulation:
                self.simulation = None
                self.guiding = False
            p=self._position_now()
            converted=self.converted if p and self.converted and self.converted.get('epoch', self.pose_epoch)==self.pose_epoch and self.clock()-self.converted["stamp"] <= 4 else None
            guide=self._guidance(converted["point"]) if self.guiding and converted and self.route else None
            if guide and p and 'heading_enu_rad' in p:
                bearing=(90-math.degrees(p['heading_enu_rad']))%360
                turn=(guide['bearing_deg']-bearing+180)%360-180
                guide['relative_turn_deg']=round(turn)
                if not guide.get('instruction'):
                    guide['instruction']=('继续向前' if abs(turn)<25 else '路线在后方，请确认行走方向' if abs(turn)>150
                                          else f"路线在{'右' if turn>0 else '左'}侧约 {abs(turn):.0f}°")
            if guide and not guide.get('instruction'):
                guide['instruction'] = ('路线进度待确认，请核对当前位置' if guide.get('uncertain')
                                        else '沿步行路线前行')
            reason = self.observation_reason if p else "定位未确认或已失效"
            if p and p["source"] == "gnss_lio":
                reason = "卫星校正暂缺，运动跟踪接续，精度可能下降" if p["coasting"] else "卫星与里程计已对齐"
            if p and p['source'] == 'map_lio':
                reason = '地图定位有效，正在连续引导'
            if p and p['source'] == 'manual_lio':
                reason = '手动位置与方向，运动跟踪接续'
            if self.position_conflict:
                reason = '定位存在冲突，引导已暂停'
            result=dict(route=self.route,position=p,bd09=converted,track=list(self.track)[-600:],track_bd09=list(self.track_bd09)[-600:],
                observation_reason=reason,alignment_reason=self.alignment_reason,
                alignment=self.alignment,guiding=self.guiding,guidance=guide,save_error=self.save_error,
                motion_status=self.motion_status,
                phone_motion=self.phone_motion_snapshot(),
                navigation_trial=self.trial.snapshot(self),
                conversion_error=self.conversion_error,mode=self.mode,session_id=self.session_id,
                local_navigation_active=self.local_navigation_active,
                origin_wgs84=self.origin.values if self.origin else None, simulation=None)
            result['geographic'] = self.geographic_snapshot()
            result['saved_map_track'] = self.saved_track_snapshot()
            result['navigation'] = self.navigation_snapshot()
            result['journey'] = self.journey_snapshot()
            result['route_assist'] = self.route_assist.snapshot(self,guide,p)
            if self.simulation:
                simulated = self.simulation.snapshot()
                result.update(simulated)
                result.update(real_observation_reason=reason, position=None, track=[],
                              alignment=None, origin_wgs84=None, conversion_error="",
                              alignment_reason="仅演练路线交互，退出后恢复真实定位")
            return copy.deepcopy(result)

    def alignment_status(self):
        """Current heading evidence for the device screen, never a saved yaw."""
        with self.lock:
            pairs=self.trial.pairs if self.trial.active else self.pairs
            count=len(pairs)
            center=[sum(p[0][i] for p in pairs)/count for i in range(2)] if count else (0.,0.)
            spread=math.sqrt(sum((p[0][0]-center[0])**2+(p[0][1]-center[1])**2 for p in pairs)/count) if count else 0.
            position=self._position_now() or {}
            heading=position.get('heading_enu_rad')
            return dict(pairs=count,spread_m=spread,heading_deg=(90-math.degrees(heading))%360 if finite(heading) else None,
                        residual_m=(self.alignment or {}).get('residual_m'),reason=self.alignment_reason)

    def _guidance(self, point):
        points=self.route["points"]
        selected_destination=(self.route.get('waypoints') or points)[-1]
        selected_distance=distance(point,selected_destination)
        endpoint_offset=distance(points[-1],selected_destination)
        coverage=dict(selected_destination_distance_m=round(selected_distance,1),
                      route_endpoint_offset_m=round(endpoint_offset,1))
        # Project onto each segment (display metric), not just the nearest vertex.
        scale=111195*math.cos(math.radians(point[1]))
        best=None
        prefix=0.
        total=sum(distance(a,b) for a,b in zip(points,points[1:]))
        now=self.clock()
        advance=min(50., max(50. if self.route_progress==0 else 12.,(now-self.progress_stamp)*3+12))
        correction=self.trial.phone_translation
        correction_tag=(id(self.route),correction.tag)
        old_correction=(getattr(self,'route_correction_total',0.)
            if getattr(self,'route_correction_tag',None)==correction_tag else 0.)
        corrected=max(0.,correction.total_m-old_correction) if correction.applies(self) else 0.
        branch=getattr(self,'route_branch_ambiguity',None)
        if branch and branch['route_id']!=id(self.route):branch=None;self.route_branch_ambiguity=None
        # Correcting a wrong geographic origin may legitimately move route
        # progress backwards. It is still not physical walking distance.
        candidates=[]
        for index,(a,b) in enumerate(zip(points,points[1:])):
            ax,ay=(a[0]-point[0])*scale,(a[1]-point[1])*111195
            bx,by=(b[0]-point[0])*scale,(b[1]-point[1])*111195
            dx,dy=bx-ax,by-ay
            length=math.hypot(dx,dy)
            t=max(0,min(1,-(ax*dx+ay*dy)/max(length*length,1e-12)))
            gap=math.hypot(ax+t*dx,ay+t*dy)
            progress=prefix+t*length
            if branch or self.route_progress-15-corrected <= progress <= self.route_progress+advance+corrected:
                remaining=max(0.,total-progress)
                candidates.append((gap,remaining,b,progress,index))
                if best is None or gap<best[0]:
                    best=(gap,remaining,b,progress,index)
            prefix+=length
        if best is None:
            return dict(off_route_m=distance(point,points[0]),remaining_m=max(0.,total-self.route_progress),bearing_deg=0,near_destination=False,display_only=True,uncertain=True,**coverage)
        correction_uncertain=not self.trial.phone_translation.arrival_ready()
        reproject=corrected>1e-9
        branch_uncertain=False
        if reproject or branch:
            similar=[v for v in candidates if v[0]<=best[0]+1.]
            ambiguous=any(abs(v[3]-best[3])>15 for v in similar)
            if ambiguous and branch is None:
                branch=dict(route_id=id(self.route),distance_m=self.odom_distance)
                self.route_branch_ambiguity=branch
            if branch and (ambiguous or self.odom_distance-branch['distance_m']<3.):
                # A correction at an intersection cannot prove which distant
                # visit of the route this is. Repainting cannot resolve this;
                # require subsequent real movement and a unique route branch.
                best=min(similar,key=lambda v:abs(v[3]-self.route_progress))
                correction_uncertain=True;branch_uncertain=True
            elif branch:
                self.route_branch_ambiguity=None;reproject=True
            self.route_correction_tag=correction_tag;self.route_correction_total=correction.total_m
        if best[0] < 20 and not branch_uncertain:
            self.route_progress=best[3] if reproject else max(self.route_progress,best[3])
            self.progress_stamp=now
        from .walking_guidance import look_ahead
        ahead=look_ahead(points,point,best[3],scale)
        arrived=best[1]<10 and distance(point,points[-1])<8 and selected_distance<8 and not correction_uncertain
        if arrived:ahead['instruction']='已到达目的地附近'
        elif best[0]>12:ahead['instruction']='已偏离路线，请查看地图返回步行路线'
        if not arrived and self.route['source']=='baidu_walking' and endpoint_offset>8 and best[1]<10:
            if selected_distance<8:
                ahead['instruction']='已接近所选终点，请核对位置'
            elif distance(point,points[-1])<8:
                ahead['instruction']=f'百度路线结束，距所选终点约{selected_distance:.0f}米'
        if correction_uncertain and best[1]<10:ahead['instruction']='位置正在校正，正在确认是否到达'
        if branch_uncertain:ahead['instruction']='路口位置有多个可能，请继续平稳行走以确认路线'
        return dict(off_route_m=round(best[0],1),remaining_m=round(best[1],1),
                    near_destination=arrived,display_only=True,**coverage,**({'uncertain':True} if correction_uncertain else {}),**ahead)


class RouteSimulation:
    """BD09 route rehearsal isolated from GNSS, ENU, persistence and ROS outputs."""
    def __init__(self, route, clock):
        self.clock = clock
        self.route = copy.deepcopy(route)
        self.guide = OutdoorState(clock=clock)
        self.guide.route = self.route
        self.progress = 0.
        self.last = clock()
        self.running = False
        self.lost = False
        self.offset = False
        self.track = deque(maxlen=600)
        self.started = False

    def update(self):
        now = self.clock()
        elapsed = max(0., now-self.last)
        # A suspended display must not jump across the entire route on resume.
        if elapsed > 5:
            self.running = False
        if self.running and not self.lost:
            self.progress = min(self.route["distance_m"], self.progress + elapsed*3.)
        self.last = now
        if self.progress >= self.route["distance_m"]:
            self.running = False

    def control(self, action):
        self.update()
        if action == "walk":
            self.running = True
            self.started = True
        elif action == "pause":
            self.running = False
        elif action == "lost":
            self.lost = True
        elif action == "offroute":
            self.offset = True
        elif action == "recover":
            self.lost = False
            self.offset = False
        elif action == "restart":
            self.__init__(self.route, self.clock)
        else:
            raise ValueError("不支持此模拟操作")

    def snapshot(self):
        self.update()
        remaining = self.progress
        points = self.route["points"]
        a, b = points[-2:]
        t = 1.
        for a, b in zip(points, points[1:]):
            length = distance(a, b)
            if remaining <= length:
                t = remaining/max(length, 1e-9)
                break
            remaining -= length
        p = [a[i]+(b[i]-a[i])*t for i in range(2)]
        if self.offset:
            scale = 111195*math.cos(math.radians(p[1]))
            east, north = (b[0]-a[0])*scale, (b[1]-a[1])*111195
            norm = max(math.hypot(east, north), 1e-9)
            p[0] += 30*north/norm/scale
            p[1] -= 30*east/norm/111195
        converted = None if self.lost else dict(point=p, stamp=self.clock(), source="simulation")
        if converted and (not self.track or self.clock()-self.track[-1]["stamp"] >= 1):
            self.track.append(converted)
        guidance = self.guide._guidance(p) if self.started and converted else None
        status = "模拟定位中断" if self.lost else "模拟偏航" if self.offset else "模拟行走中" if self.running else "模拟已暂停"
        return dict(simulation=dict(active=True, running=self.running, lost=self.lost,
                                   offset=self.offset, progress_m=round(self.progress,1), speed_mps=3),
                    bd09=converted, track_bd09=list(self.track), guidance=guidance,
                    guiding=self.started, observation_reason=status)

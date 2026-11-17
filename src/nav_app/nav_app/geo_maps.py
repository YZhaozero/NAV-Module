"""Persistent geographic references and quality-gated map/LIO handover.

One selected source supplies the geographic pose; correlated GNSS and ICP estimates
are never averaged as independent measurements. Localizer alone owns map->odom.
All transforms below map child coordinates into parent coordinates.
"""
import copy
from .gnss_alignment import alignment_configuration_ready
from .features.localization_health import MAX_INPUT_AGE_S
import hashlib
import json
import math
import os
from pathlib import Path
import uuid

GEOGRAPHIC_REFERENCE_SOURCES = ('gnss_lio', 'manual_map_selection', 'trajectory_alignment')


def vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


def rotate(q, p):
    x, y, z, w = q
    a, b, c = p
    return [(1-2*y*y-2*z*z)*a+(2*x*y-2*z*w)*b+(2*x*z+2*y*w)*c,
            (2*x*y+2*z*w)*a+(1-2*x*x-2*z*z)*b+(2*y*z-2*x*w)*c,
            (2*x*z-2*y*w)*a+(2*y*z+2*x*w)*b+(1-2*x*x-2*y*y)*c]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def angle(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with tmp.open('w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            tmp.unlink()


def projection_record(projection, origin):
    """Keep the official conversion fit with a capture, never a new live pose."""
    if projection is None or tuple(projection.origin) != tuple(origin):
        return None
    return dict(source='baidu_official_local_fit', origin_wgs84=list(origin),
                converted_samples=[projection.to_bd09(p) for p in
                    ((0, 0), (200, 0), (-200, 0), (0, 200), (0, -200))],
                validation_error_m=projection.validation_error_m)


def saved_geographic_track(data):
    """Read a saved map's trajectory without restoring a live odometry frame.

    The finalized reference can project pre-alignment local samples too. This
    is a historical reconstruction, not evidence of a fresh/fixed GNSS fix.
    Older manifests retain their recorded BD09 points when the official local
    conversion fit was not archived; no coordinate-system shortcut is used.
    """
    from .outdoor import GeoOrigin, coordinate, finite
    from .navigation_trial import LocalMapProjection
    GeoMaps.validate(data)
    result = dict(map_id=data['id'], title=data['title'], saved_at=data['saved_at'],
        session_id=data.get('session_id'), reference_source=data['reference_source'],
        historical=True, live_pose=False, absolute_accuracy_verified=False,
        capture_frame=data.get('capture_frame'), odom_epoch=data.get('odom_epoch'),
        odom_continuity=data.get('odom_continuity'), odom_instance=data.get('odom_instance'),
        origin_wgs84=copy.deepcopy(data.get('origin_wgs84')),
        enu_from_map=copy.deepcopy(data.get('enu_from_map')),
        alignment_evidence=copy.deepcopy(data.get('alignment_evidence')),
        route=copy.deepcopy(data.get('route')), route_context=copy.deepcopy(data.get('route_context')),
        track_wgs84=[], track_bd09=[], local_track=[], skipped_local_samples=0,
        coordinate_contract='track_wgs84=WGS84; track_bd09=BD09; local_track=capture_frame',
        wgs84_derivation='final_saved_reference', bd09_derivation='recorded_online_positions')
    for name in ('track', 'estimated_track', 'track_bd09'):
        result['recorded_'+name] = copy.deepcopy(data.get(name, []))
    result['track_bd09'] = [copy.deepcopy(v) for v in data.get('track_bd09', [])
        if isinstance(v, dict) and finite(v.get('stamp')) and coordinate(v.get('point'))]
    if data['reference_source'] not in GEOGRAPHIC_REFERENCE_SOURCES:
        return result
    projection = None
    raw_projection = data.get('bd09_projection')
    if (isinstance(raw_projection, dict) and raw_projection.get('source') == 'baidu_official_local_fit'
            and raw_projection.get('origin_wgs84') == data['origin_wgs84']):
        try:
            projection = LocalMapProjection(data['origin_wgs84'], raw_projection['converted_samples'])
        except (ValueError, TypeError, KeyError):
            pass
    origin = GeoOrigin(*data['origin_wgs84'])
    a = data['enu_from_map']
    rotation = a.get('rotation_xyzw', [0, 0, math.sin(a['yaw']/2), math.cos(a['yaw']/2)])
    epoch, continuity = data.get('odom_epoch'), data.get('odom_continuity')
    identity_known = type(epoch) is int and type(continuity) is int and bool(data.get('capture_frame'))
    projected_bd09, segment, last_stamp = [], 0, None
    for row in data.get('local_track', []):
        if (not identity_known or not isinstance(row, dict) or row.get('epoch') != epoch
                or row.get('continuity') != continuity or not finite(row.get('stamp'))
                or not vector(row.get('position_m'), 3)):
            result['skipped_local_samples'] += 1
            segment += 1
            last_stamp = None
            continue
        if last_stamp is not None and (row['stamp'] <= last_stamp or row['stamp']-last_stamp > 10):
            segment += 1
        last_stamp = row['stamp']
        enu = [v+a[k] for v, k in zip(rotate(rotation, row['position_m']), ('tx', 'ty', 'tz'))]
        sample = dict(stamp=row['stamp'], point=origin.wgs84(*enu), segment=segment,
                      derived=True, source='saved_lio_reference')
        result['track_wgs84'].append(sample)
        result['local_track'].append(copy.deepcopy(row))
        if projection:
            try:
                projected_bd09.append(dict(sample, point=projection.to_bd09(enu)))
            except ValueError:
                # Keep the exported WGS84 point, but break the display line at
                # this fit's documented 1 km boundary instead of extrapolating.
                segment += 1
    if projection and projected_bd09:
        result['track_bd09'] = projected_bd09
        result['bd09_derivation'] = 'final_saved_reference_and_official_local_fit'
    result['counts'] = {key: len(result[key]) for key in
        ('track_wgs84', 'track_bd09', 'local_track', 'recorded_track', 'recorded_estimated_track')}
    return result


class GeoMaps:
    """Small manifest catalog. Raw PCD, bags and user assets stay in place."""
    def __init__(self, directory):
        self.directory = Path(directory) / 'maps' if directory else None
        self.entries = {}
        self.errors = 0
        if self.directory and self.directory.exists():
            for path in sorted(self.directory.glob('*.json'))[:500]:
                try:
                    if path.stat().st_size > 8_000_000:
                        raise ValueError('oversize')
                    data = json.loads(path.read_text(encoding='utf-8'))
                    self.validate(data)
                    self.entries[data['id']] = self.index_entry(data)
                except (OSError, ValueError, TypeError, KeyError):
                    self.errors += 1

    @staticmethod
    def index_entry(data):
        # Hundreds of saved maps must not load every historic trajectory into RAM.
        return copy.deepcopy({k:v for k,v in data.items() if k not in
                              ('track','track_bd09','local_track','estimated_track','route')})

    @staticmethod
    def validate(data):
        from .outdoor import coordinate, finite
        if not isinstance(data, dict) or data.get('schema') != 1:
            raise ValueError('地理地图格式不支持')
        if (not isinstance(data.get('id'), str) or len(data['id']) != 32
                or any(c not in '0123456789abcdef' for c in data['id'])
                or not isinstance(data.get('cloud_path'), str)
                or not finite(data.get('saved_at')) or not isinstance(data.get('title'), str)
                or not isinstance(data.get('sha256'), str) or len(data['sha256']) != 64
                or any(c not in '0123456789abcdef' for c in data['sha256'])):
            raise ValueError('地图身份信息不完整')
        # A local metric map is usable without satellite coverage or a world anchor.
        if data.get('reference_source') == 'local_only':
            if data.get('origin_wgs84') is not None or data.get('enu_from_map') is not None:
                raise ValueError('未绑定地理坐标的地图不能附带未经验证的变换')
            return
        origin, transform = data.get('origin_wgs84'), data.get('enu_from_map', {})
        if not isinstance(transform,dict):
            raise ValueError('地图坐标变换格式无效')
        rotation = transform.get('rotation_xyzw')
        if rotation is not None and (not vector(rotation, 4) or abs(sum(v*v for v in rotation)-1) > .001):
            raise ValueError('地图地理旋转无效')
        if (not isinstance(data.get('id'), str) or len(data['id']) != 32
                or any(c not in '0123456789abcdef' for c in data['id'])
                or not vector(origin, 3) or not coordinate(origin[:2])
                or not all(finite(transform.get(k)) for k in ('yaw', 'tx', 'ty', 'tz'))
                or not isinstance(data.get('cloud_path'), str)
                or not finite(data.get('saved_at')) or not isinstance(data.get('title'),str)
                or not isinstance(data.get('sha256'), str) or len(data['sha256']) != 64
                or any(c not in '0123456789abcdef' for c in data['sha256'])
                or data.get('reference_source') not in GEOGRAPHIC_REFERENCE_SOURCES):
            raise ValueError('地理地图标定或坐标信息不完整')
        if data['reference_source'] == 'gnss_lio':
            if (not isinstance(data.get('calibration'), dict)
                    or not alignment_configuration_ready(data['calibration'])):
                raise ValueError('地理地图标定或坐标信息不完整')
        else:
            evidence = data.get('alignment_evidence')
            if (not isinstance(evidence, dict) or evidence.get('experimental') is not True
                    or data.get('field_validated') is not False):
                raise ValueError('手动或轨迹地理参考需要保留来源与未验证标记')
            if data['reference_source'] == 'manual_map_selection' and (
                    evidence.get('manual') is not True
                    or evidence.get('source') != 'manual_map_selection'
                    or not coordinate(evidence.get('selected_bd09'))
                    or not coordinate(evidence.get('direction_bd09'))):
                raise ValueError('手动地理参考缺少确认的位置与方向')

    def add(self, data):
        self.validate(data)
        if not self.directory:
            raise ValueError('地图保存目录尚未配置')
        atomic_json(self.directory / (data['id']+'.json'), data)
        self.entries[data['id']] = self.index_entry(data)

    def get(self, identifier, verify=False, full=False):
        data = self.entries.get(identifier)
        if data is None:
            raise ValueError('未找到这份地理地图')
        if full:
            path=self.directory/(identifier+'.json')
            if path.stat().st_size>8_000_000:
                raise ValueError('地图记录过大，无法读取')
            data=json.loads(path.read_text(encoding='utf-8'))
            self.validate(data)
        if verify and file_hash(data['cloud_path']) != data['sha256']:
            raise ValueError('点云内容已变化，请重新登记地理参考')
        return copy.deepcopy(data)


class GeographicNavigation:
    """Mixed into OutdoorState; all state access occurs under its RLock."""
    def init_geographic(self):
        from collections import deque
        self.maps = GeoMaps(self.directory)
        self.selected_map = None
        self.map_match = None
        self.map_reason = '选择已有地图进行定位，或直接开始采集建图'
        self.odom_frame = ''
        self.odom_epoch = 0
        self.map_not_before = self.clock()
        self.map_session = None
        self.map_last_sequence = -1
        self.map_samples = 0
        self.map_confirmation = None
        self.pose_epoch = 0
        self.previous_output = None
        self.position_conflict = False
        self.pending_pose = None
        self.local_track = []
        self.estimated_track = []
        self._saved_map_track = None
        self.requests = deque(maxlen=4)

    def read_saved_map_track(self, identifier):
        """Explicit worker/export read; snapshot polling does no filesystem I/O."""
        return saved_geographic_track(self.maps.get(identifier, full=True))

    def _cache_saved_map_track(self, data):
        history = saved_geographic_track(data) if data else None
        if history:
            # A small whole-route overview, not the last 600 seconds only.
            for key in ('track_wgs84', 'track_bd09'):
                rows = history[key]
                if len(rows) > 600:
                    history[key] = [rows[round(i*(len(rows)-1)/599)] for i in range(600)]
            for key in ('local_track', 'recorded_track', 'recorded_estimated_track', 'recorded_track_bd09'):
                history.pop(key, None)
        self._saved_map_track = history

    def saved_track_snapshot(self):
        history = self._saved_map_track
        if (not self.simulation and self.selected_map and history
                and history['map_id'] == self.selected_map['id']):
            return copy.deepcopy(history)
        return None

    def clear_map_match(self):
        self.map_match = None
        self.map_session = None
        self.map_last_sequence = -1
        self.map_samples = 0
        self.map_confirmation = None
        self.position_conflict = False
        self.pending_pose = None
        self.previous_output = None
        self.converted = None
        self.pose_epoch += 1
        self.map_not_before = self.clock()

    def select_geo_map(self, identifier):
        # Hashing is performed by the HTTP/worker thread, never a GUI timer.
        if identifier is not None and (not isinstance(identifier,str) or len(identifier)!=32):
            raise ValueError('地图编号无效')
        data = self.maps.get(identifier, verify=True, full=True) if identifier else None
        with self.lock:
            if self.simulation or self.guiding or self.local_navigation_active or self.journey.active:
                raise ValueError('请先结束引导并退出模拟，再更换地图')
            previous=self.selected_map
            self.selected_map = self.maps.index_entry(data) if data else None
            try:
                self._save()
            except (OSError,ValueError):
                self.selected_map=previous
                raise
            self._cache_saved_map_track(data)
            self.clear_map_match()
            self.map_reason = ('地图已选择，请启动地图定位' if data else '未选择已有地图，可开始采集')
        return data

    def prepare_local_map(self, cloud_path, title, scene_path=None):
        """Hash on a worker before binding an existing Scene to the common catalog."""
        path = Path(cloud_path).expanduser().resolve(strict=True)
        if path.suffix.lower() != '.pcd':
            raise ValueError('定位地图需要 PCD 格式')
        digest = file_hash(path)
        with self.lock:
            existing = [v for v in self.maps.entries.values()
                        if v['sha256'] == digest and Path(v['cloud_path']).resolve() == path]
            selected=max(existing, key=lambda v: (
                v['reference_source'] == 'gnss_lio',
                v['reference_source'] in GEOGRAPHIC_REFERENCE_SOURCES)) if existing else None
            data = self.maps.get(selected['id'], full=True) if selected else None
        if data is None:
            data=dict(schema=1, id=uuid.uuid4().hex, saved_at=self.clock(), title=str(title)[:80],
                    cloud_path=str(path), sha256=digest, reference_source='local_only',
                    field_validated=False)
        if scene_path:
            from .scene_catalog import read_scene
            from .journey import grid_fingerprint
            scene=read_scene(scene_path)
            if not scene.ready or Path(scene.cloud).resolve()!=path:
                raise ValueError('场景地图坐标或资产未核对')
            data.update(scene_path=str(Path(scene_path).resolve()),grid_sha256=grid_fingerprint(scene.grid))
        return data

    def bind_local_map(self, data):
        """Commit a worker-validated Scene identity when the GUI applies that Scene."""
        with self.lock:
            if self.guiding or self.simulation or self.local_navigation_active or self.journey.active:
                raise ValueError('请先结束当前引导并退出模拟，再更换地图')
            self.maps.add(data)
            previous = self.selected_map
            self.selected_map = self.maps.index_entry(data)
            try:
                self._save()
            except (OSError, ValueError):
                self.selected_map = previous
                raise
            self._cache_saved_map_track(data)
            self.clear_map_match()
            self.map_reason = '地图已选择，等待本次地图匹配'

    def anchor_selected_map(self):
        """Add a world reference to an existing metric map, even if built with no GNSS.

        Run off the GUI thread. Never infer an anchor from a Baidu screen point.
        """
        with self.lock:
            selected = copy.deepcopy(self.selected_map)
        if not selected:
            raise ValueError('请先选择定位地图')
        digest = file_hash(selected['cloud_path'])
        with self.lock:
            g, a = self.selected_map, self.alignment
            if not g or g['id'] != selected['id'] or digest != g['sha256']:
                raise ValueError('地图选择或点云内容已变化，请重试')
            pose = self.local_map_pose()
            if (not g or not pose or not a or a.get('experimental') or not self.origin or self.sensor_replay or self.simulation
                    or not 0 <= self.clock()-a['stamp'] <= 2
                    or not alignment_configuration_ready(self.calibration)):
                raise ValueError('需要同时获得可信地图定位和卫星对齐结果')
            if g.get('reference_source') == 'gnss_lio':
                raise ValueError('这份地图已有地理参考，重新标定需另行核对')
            # ENU<-map = (ENU<-odom) * inverse(map<-odom), including tilt.
            m = self.map_match
            q = m['quaternion']; inverse = [-q[0],-q[1],-q[2],q[3]]
            rotation = multiply([0,0,math.sin(a['yaw']/2),math.cos(a['yaw']/2)], inverse)
            offset = rotate(rotation, m['translation'])
            transform = dict(yaw=yaw(rotation), rotation_xyzw=rotation,
                tx=a['tx']-offset[0], ty=a['ty']-offset[1], tz=a.get('tz',0)-offset[2])
            data = self.maps.get(g['id'], full=True)
            data.update(reference_source='gnss_lio', origin_wgs84=list(self.origin.values),
                        enu_from_map=transform, calibration=copy.deepcopy(self.calibration),
                        alignment_evidence=copy.deepcopy(a), anchored_at=self.clock())
            self.maps.add(data)
            self.selected_map = self.maps.index_entry(data)
            self._cache_saved_map_track(data)
            self.pose_epoch += 1
            self.converted = None
            self.map_reason = '地图已关联地理位置，可接续路网引导'
            return data

    def request_localization(self, action):
        with self.lock:
            if self.mode != 'walking' or self.simulation or self.local_navigation_active:
                raise ValueError('请先退出模拟并切换到手持模式')
            if action not in ('start', 'stop') or (action == 'start' and not self.selected_map):
                raise ValueError('请先选择已保存的地图')
            if self.requests:
                raise ValueError('正在处理上一次操作，请稍候')
            self.requests.append(action)
            self.map_reason = '正在准备地图定位' if action == 'start' else '正在停止地图定位'

    def geographic_snapshot(self):
        from .outdoor import distance
        current = self._position_now()
        entries = []
        for data in self.maps.entries.values():
            entries.append(dict(id=data['id'], title=data.get('title', '采集地图'),
                                saved_at=data['saved_at'], cloud_path=data['cloud_path'],
                                field_validated=False,
                                georeferenced=data['reference_source'] in GEOGRAPHIC_REFERENCE_SOURCES,
                                reference_source=data['reference_source'],
                                distance_m=round(distance(current['wgs84'], data['origin_wgs84'][:2])) if current and data.get('origin_wgs84') else None))
        entries.sort(key=lambda d: (d['distance_m'] if d['distance_m'] is not None else 1e12, -d['saved_at']))
        reason=self.map_reason
        if self.map_match and not self.local_map_pose():
            reason='地图定位已过期，等待重新匹配'
        return dict(maps=entries, selected_map_id=self.selected_map['id'] if self.selected_map else None,
                    map_reason=reason, map_catalog_errors=self.maps.errors,
                    source=current['source'] if current else 'unavailable',
                    heading_ready=bool(current and 'heading_enu_rad' in current),
                    conflict=self.position_conflict, odom_epoch=self.odom_epoch,
                    can_confirm_position=self.pending_pose is not None)

    def capture_reference(self, cloud_frame):
        """Freeze accepted coordinates before SaveMap stops live pose updates.

        An old observation time does not invalidate a fixed transform within
        the same odometry instance. Its source/uncertainty remain historical
        evidence, never a fabricated fresh GNSS fix or a localization seed.
        """
        with self.lock:
            a = self.alignment
            if (self.simulation or self.sensor_replay or not a or not self.origin or not self.odom
                    or cloud_frame != self.odom_frame or not cloud_frame
                    or self.alignment_conflict
                    or a.get('odom_continuity') != self.odom_continuity
                    or not all(type(a.get(k)) in (int, float) and math.isfinite(a[k])
                               for k in ('yaw', 'tx', 'ty', 'stamp'))
                    or self.clock() < a['stamp']
                    or (not a.get('experimental') and not alignment_configuration_ready(self.calibration))):
                return None
            source = ('manual_map_selection' if a.get('manual') and a.get('source') == 'manual_map_selection'
                      else 'trajectory_alignment' if a.get('experimental') else 'gnss_lio')
            return copy.deepcopy(dict(origin_wgs84=self.origin.values,
                enu_from_map={k:a.get(k, 0.) for k in ('yaw', 'tx', 'ty', 'tz')},
                alignment_evidence=a, calibration=self.calibration,
                odom_epoch=self.odom_epoch, odom_continuity=self.odom_continuity,
                odom_instance=self.odom_instance, capture_frame=cloud_frame,
                reference_frozen_at=self.clock(), reference_source=source,
                last_pose_stamp=self.odom[-1][0],
                last_pose_age_at_freeze_s=self.clock()-self.odom[-1][0],
                field_validated=False, route=self.route, route_context=self.route_context(),
                local_track=self.local_track[-3600:], track=list(self.track),
                estimated_track=self.estimated_track[-3600:],
                bd09_projection=projection_record(self.trial.projection, self.origin.values),
                track_bd09=list(self.track_bd09), session_id=self.session_id,
                title=(self.route or {}).get('title', '采集地图')))

    def capture_reference_matches(self, reference, cloud_frame):
        """A save receipt may reuse a frozen reference, but never across resets."""
        with self.lock:
            return bool(isinstance(reference, dict) and not self.simulation and not self.sensor_replay
                        and reference.get('session_id') == self.session_id
                        and reference.get('odom_epoch') == self.odom_epoch
                        and reference.get('odom_continuity') == self.odom_continuity
                        and reference.get('odom_instance') == self.odom_instance
                        and reference.get('capture_frame') == cloud_frame == self.odom_frame
                        and bool(cloud_frame))

    def capture_session(self):
        """Freeze ownership before queuing work that can outlive a capture."""
        with self.lock:
            return copy.deepcopy(self._document())

    def _capture_assets(self, owner, updates, archive=False):
        """Update the owning session, never a newer live navigation trial."""
        if owner['session_id'] == self.session_id:
            self.assets.update(copy.deepcopy(updates))
            if archive:
                self.save_session()
            else:
                self._save()
            return
        if not self.directory:
            return
        target = self.directory / f"session-{owner['session_id']}.json"
        # A trial may have archived more history while hashing was underway.
        # Preserve that final history and merge only the completed assets.
        document = json.loads(target.read_text(encoding='utf-8')) if target.exists() else copy.deepcopy(owner)
        if document.get('session_id') != owner['session_id'] or document.get('schema') != 1:
            raise ValueError('采集记录所属会话不一致')
        document.setdefault('assets', {}).update(copy.deepcopy(updates))
        document['saved_at'] = self.clock()
        atomic_json(target, document)

    def register_capture(self, capture, reference, owner=None):
        owner = self.capture_session() if owner is None else owner
        if not capture.get('map'):
            with self.lock:
                if owner['session_id'] == self.session_id:
                    self.map_reason = '录包已保留，本次尚未生成定位地图'
            return None
        if not reference:
            data = self.prepare_local_map(capture['map'], capture.get('name', '采集地图'))
            with self.lock:
                data['assets'] = copy.deepcopy(capture)
                data['session_id'] = owner['session_id']
                self.maps.add(data)
                self._capture_assets(owner, {'geographic_map_id': data['id']})
                if owner['session_id'] == self.session_id:
                    self.map_reason = '地图已保存，可用于本地定位；地理位置尚未绑定'
            return data
        if (reference.get('session_id') != owner['session_id']
                or reference.get('odom_epoch') != owner.get('odom_epoch')
                or reference.get('odom_continuity') != owner.get('odom_continuity')
                or reference.get('odom_instance') != owner.get('odom_instance')
                or reference.get('capture_frame') != owner.get('odom_frame')):
            raise ValueError('地图地理参考与采集会话不一致，原始点云仍保留')
        path = Path(capture['map']).expanduser().resolve(strict=True)
        if path.suffix.lower() != '.pcd':
            raise ValueError('地理参考只接受本次保存的 PCD 点云')
        data = dict(reference, schema=1, id=uuid.uuid4().hex, saved_at=self.clock(),
                    session_id=owner['session_id'],
                    cloud_path=str(path), sha256=file_hash(path), assets=copy.deepcopy(capture),
                    coordinate_contract='WGS84 ellipsoid -> ENU; enu_from_map; metres; yaw radians',
                    field_validated=False)
        with self.lock:
            self.maps.add(data)
            self._capture_assets(owner, {'geographic_map_id': data['id']})
            if owner['session_id'] == self.session_id:
                self.map_reason = '地图地理参考已保存，下次可选择这份地图进行定位'
        return data

    def finish_capture_assets(self, capture, owner=None):
        owner = self.capture_session() if owner is None else owner
        with self.lock:
            updates = {'capture': copy.deepcopy(capture)}
            for entry in list(self.maps.entries.values()):
                if (entry['cloud_path']==capture.get('map')
                        and entry.get('session_id', owner['session_id'])==owner['session_id']):
                    full=self.maps.get(entry['id'],full=True)
                    self.maps.add(dict(full,assets=copy.deepcopy(capture)))
                    updates['geographic_map_id']=entry['id']
            self._capture_assets(owner, updates, archive=True)

    def feed_map_match(self, data):
        """Quality and transform in ONE message; never combine stale TF/quality."""
        from .outdoor import finite
        with self.lock:
            if not isinstance(data,dict):
                self.map_match=None
                self.map_samples=0
                self.map_confirmation=None
                return False
            now = self.clock()
            geo = self.selected_map
            if not geo:
                return False
            # Wrong map/old or malformed observation invalidates this source.
            stamp = data.get('published_stamp_ns', 0)/1e9 if finite(data.get('published_stamp_ns')) else 0
            success = data.get('last_success_stamp_ns', 0)/1e9 if finite(data.get('last_success_stamp_ns')) else 0
            # Share the localizer's bounded observation lifetime. A fresh status
            # heartbeat cannot extend it; old publishers retain the 2 s policy.
            lifetime = data.get('tracking_timeout_s', 2.)
            transform = data.get('map_from_odom', {})
            if not isinstance(transform,dict):
                transform={}
            q, t = transform.get('rotation_xyzw'), transform.get('translation_m')
            session, sequence = data.get('instance_id'), data.get('match_sequence')
            consistent = (data.get('map_sha256') == geo['sha256']
                     and data.get('odom_frame') == self.odom_frame and bool(self.odom_frame)
                     and data.get('map_frame') == 'map' and isinstance(session, str) and bool(session)
                     and type(sequence) is int and sequence > 0
                     and finite(lifetime) and 0 < lifetime <= 5
                     and -.05 <= now-stamp <= MAX_INPUT_AGE_S and -.05 <= now-success <= lifetime
                     and success >= self.map_not_before
                     and finite(data.get('data_age_s')) and data['data_age_s'] >= -.05
                     and type(data.get('failures',0)) is int and 0 <= data.get('failures',0) < 3
                     and not data.get('input_error') and not data.get('verifying_initial_pose')
                     and not data.get('initial_pose_required')
                     and finite(data.get('fitness')) and data['fitness'] >= 0
                     and vector(q, 4) and .99 < sum(v*v for v in q) < 1.01 and vector(t, 3))
            confirmation = self.map_confirmation
            confirmed = bool(confirmation and consistent
                and confirmation['session'] == session
                and confirmation['odom_epoch'] == self.odom_epoch
                and now <= confirmation['valid_until']
                and sequence >= self.map_last_sequence)
            valid = (consistent and data.get('valid') is True
                     and data['data_age_s'] <= MAX_INPUT_AGE_S)
            if not valid:
                self.map_match = None
                # Retain only admission evidence, never a usable pose. A new
                # fresh status may resume this same confirmed session inside
                # its original observation lifetime. Delayed heartbeats do
                # not extend that deadline; geometry/input faults reset it.
                delayed = (confirmed and data.get('valid') is False
                    and data['data_age_s'] > MAX_INPUT_AGE_S
                    and data.get('operation') == 'tracking'
                    and data.get('last_match_reason','accepted') in ('accepted','observation_expired'))
                if not delayed:
                    self.map_samples = 0
                    self.map_confirmation = None
                self.map_reason = ('定位数据暂时延迟，等待新鲜数据恢复' if delayed
                                   else '等待同一份地图的新鲜匹配结果')
                return False
            if confirmation and not confirmed:
                self.map_samples = 0
                self.map_confirmation = None
            if session != self.map_session:
                self.map_session, self.map_last_sequence, self.map_samples = session, -1, 0
                self.map_match = None
                self.map_confirmation = None
            if sequence < self.map_last_sequence:
                self.map_match = None
                self.map_samples = 0
                self.map_confirmation = None
                return False
            if sequence > self.map_last_sequence:
                self.map_samples += 1
                self.map_last_sequence = sequence
            if self.map_samples < 2:
                self.map_reason = '地图已找到，正在复核定位'
                return False
            self.map_match = dict(translation=t, quaternion=q, stamp=success,
                                  received=stamp, valid_until=success+lifetime, odom_epoch=self.odom_epoch)
            self.map_confirmation = dict(session=session,odom_epoch=self.odom_epoch,
                                         valid_until=success+lifetime)
            self.map_reason = '已通过地图定位，可在卫星较弱处继续引导'
            return True

    def local_map_pose(self):
        """Metric map pose stays valid with no GNSS or geographic reference."""
        m, g = self.map_match, self.selected_map
        now = self.clock()
        if (self.sensor_replay or self.simulation or not m or not g or not self.odom or m['odom_epoch'] != self.odom_epoch
                or not -.05 <= now-m['received'] <= MAX_INPUT_AGE_S or now > m.get('valid_until', m['stamp']+2.)
                or not -.05 <= now-self.odom[-1][0] <= MAX_INPUT_AGE_S):
            return None
        stamp, local, attitude = self.odom[-1]
        mapped = [a+b for a,b in zip(rotate(m['quaternion'], local), m['translation'])]
        q = multiply(m['quaternion'], attitude)
        return dict(frame='map', map_id=g['id'], map_sha256=g['sha256'], stamp=stamp,
                    position_m=mapped, quaternion_xyzw=q, heading_rad=yaw(q), source='map_lio')

    def _map_position(self):
        from .outdoor import GeoOrigin
        pose = self.local_map_pose()
        g = self.selected_map
        if not pose or g.get('reference_source') not in GEOGRAPHIC_REFERENCE_SOURCES:
            return None
        mapped, stamp = pose['position_m'], pose['stamp']
        a = g['enu_from_map']
        rotation = a.get('rotation_xyzw', [0,0,math.sin(a['yaw']/2),math.cos(a['yaw']/2)])
        enu = [p+a[k] for p,k in zip(rotate(rotation,mapped), ('tx','ty','tz'))]
        wgs = GeoOrigin(*g['origin_wgs84']).wgs84(*enu)
        return dict(wgs84=wgs, stamp=stamp, id=f'map-{g["id"]}-{self.odom_epoch}-{stamp:.6f}',
                    source='map_lio', heading_enu_rad=yaw(multiply(rotation,pose['quaternion_xyzw'])),
                    map_id=g['id'], coasting=False, reference_source=g['reference_source'],
                    experimental=g['reference_source'] != 'gnss_lio',
                    absolute_accuracy_verified=False)

    def initial_map_pose(self):
        """GNSS/LIO prior for ICP, never an accepted localization by itself."""
        from .outdoor import GeoOrigin
        with self.lock:
            a, g = self.alignment, self.selected_map
            if (not a or a.get('experimental') or not g or g.get('reference_source') != 'gnss_lio' or not self.origin or not self.odom or self.simulation or self.sensor_replay
                    or not alignment_configuration_ready(self.calibration)
                    or not 0 <= self.clock()-a['stamp'] <= 2
                    or not -.05 <= self.clock()-self.odom[-1][0] <= .5):
                return None
            stamp,p,q=self.odom[-1]
            co,si=math.cos(a['yaw']),math.sin(a['yaw'])
            wgs=self.origin.wgs84(co*p[0]-si*p[1]+a['tx'],si*p[0]+co*p[1]+a['ty'],p[2]+a.get('tz',0))
            height=self.origin.values[2]+p[2]+a.get('tz',0)
            east,north,up=GeoOrigin(*g['origin_wgs84']).enu(*wgs,height)
            t=g['enu_from_map']
            east,north,up=east-t['tx'],north-t['ty'],up-t['tz']
            if math.hypot(east,north)>50000:
                return None
            rotation=t.get('rotation_xyzw',[0,0,math.sin(t['yaw']/2),math.cos(t['yaw']/2)])
            inverse=[-rotation[0],-rotation[1],-rotation[2],rotation[3]]
            return dict(stamp=stamp,frame='map',position_m=rotate(inverse,[east,north,up]),
                quaternion_xyzw=multiply(inverse,multiply([0,0,math.sin(a['yaw']/2),math.cos(a['yaw']/2)],q)),
                horizontal_variance_m2=max(1.,a.get('residual_m',.8)**2),
                yaw_variance_rad2=max(math.radians(5)**2,(a.get('residual_m',.8)/max(4.,a.get('spread_m',4)))**2))

    def navigation_snapshot(self):
        """Capabilities, not indoor/outdoor modes. No odom-only global validity."""
        with self.lock:
            geographic = self._position_now()
            local = None if self.trial.active else self.local_map_pose()
            odom_live = bool(self.odom and -.05 <= self.clock()-self.odom[-1][0] <= .5
                             and not self.sensor_replay and not self.simulation)
            if local:
                source = self.selected_map.get('reference_source')
                reason = '地图定位有效' + ('，地理位置采用手动标定' if source == 'manual_map_selection' else
                    '，地理位置采用轨迹对齐' if source == 'trajectory_alignment' else
                    '，地理位置已关联' if source == 'gnss_lio' else '，可继续地图内导航')
            elif geographic:
                reason = (f"运动跟踪接续 · 距上次卫星校正 {geographic.get('correction_age_s',0):.0f} 秒 · 精度可能下降"
                          if geographic.get('coasting') else
                          '手动位置与方向，运动跟踪接续' if geographic['source'] == 'manual_lio' else
                          '卫星与里程计定位有效' if geographic['source'] == 'gnss_lio' else '卫星位置可用，方向尚待确认')
            else:
                reason = '运动跟踪可用，等待地图或卫星定位' if odom_live else '等待定位数据'
            if self.position_conflict:
                reason = '地理定位冲突，路网引导暂停' + ('；地图内定位仍有效' if local else '')
            if self.trial.active:
                trial=self.trial.snapshot(self)
                reason=trial['phase']+' · '+('轨迹对准，绝对精度待验证' if trial['aligned'] else trial['reason'])
            return dict(schema=1, stamp=self.clock(), valid=bool(local or geographic),
                        local_pose=local, geographic_pose=geographic, odometry_live=odom_live,
                        map_navigation_ready=bool(local), geographic_navigation_ready=bool(geographic),
                        geographic_conflict=self.position_conflict, reason=reason,
                        map_id=self.selected_map['id'] if self.selected_map and not self.trial.active else None,
                        simulation=False, tf_owner='localizer')

    def arbitrate_position(self, lio, gnss):
        from .outdoor import distance
        mapped = None if self.trial.active else self._map_position()
        candidate = mapped or lio or gnss
        self.pending_pose = None
        if mapped and lio and not lio['coasting'] and distance(mapped['wgs84'], lio['wgs84']) > 3:
            self.position_conflict = True
            self.map_reason = '卫星与地图定位不一致，请重新定位后再继续'
            return None
        if not candidate:
            return None
        previous = self.previous_output
        if previous and candidate['stamp'] >= previous['stamp']:
            gap = distance(candidate['wgs84'], previous['wgs84'])
            dt = candidate['stamp']-previous['stamp']
            correction=0.
            correction_state=self.trial.phone_translation
            same_correction=(candidate.get('geographic_correction_identity')==previous.get('geographic_correction_identity')
                and candidate.get('geographic_correction_identity') is not None)
            started_here=(previous.get('geographic_correction_identity') is None
                and candidate['source']==previous['source'] and correction_state.started_stamp is not None
                and previous['stamp']>=correction_state.started_stamp-.01)
            if correction_state.applies(self) and (same_correction or started_here):
                correction=max(0.,candidate.get('geographic_correction_total_m',0.)-previous.get('geographic_correction_total_m',0.))
            if gap > 3 + min(10., dt*2) + correction:
                self.pending_pose = candidate
                self.position_conflict = True
                self.map_reason = '定位来源变化较大，请核对当前位置后继续'
                return None
            if candidate['source'] != previous['source']:
                self.pose_epoch += 1
                self.converted = None
        self.position_conflict = False
        self.previous_output = copy.deepcopy(candidate)
        candidate = dict(candidate, id=f'{self.pose_epoch}:{candidate["id"]}', epoch=self.pose_epoch)
        return candidate

    def confirm_current_position(self):
        with self.lock:
            self._position_now()
            if self.mode != 'walking' or self.simulation or not self.pending_pose:
                raise ValueError('当前没有可确认的新定位')
            self.previous_output = copy.deepcopy(self.pending_pose)
            self.pending_pose = None
            self.position_conflict = False
            self.pose_epoch += 1
            self.converted = None
            self.map_reason = '已确认新位置，正在更新地图'

    def sample_actual_track(self):
        """Keep local metric route separate from Baidu's planned polyline."""
        with self.lock:
            pose=self._position_now()
            if (not self.simulation and not self.sensor_replay and pose
                    and (not self.estimated_track or pose['stamp']-self.estimated_track[-1]['stamp']>=1)):
                self.estimated_track.append(copy.deepcopy(pose))
                self.estimated_track=self.estimated_track[-3600:]
            if self.simulation or self.sensor_replay or not self.odom:
                return
            stamp, p, q = self.odom[-1]
            if -.05 <= self.clock()-stamp <= .5 and (not self.local_track or stamp-self.local_track[-1]['stamp'] >= 1):
                self.local_track.append(dict(stamp=stamp, position_m=p, quaternion_xyzw=q,
                                             epoch=self.odom_epoch, continuity=self.odom_continuity))
                self.local_track = self.local_track[-3600:]

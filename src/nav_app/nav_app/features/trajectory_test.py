"""Independent, non-fusing GNSS field experiments. All I/O runs off ROS/Qt threads."""
from bisect import bisect_left, bisect_right
from collections import deque
from datetime import datetime
from pathlib import Path
import csv
import io
import hashlib
import fcntl
import json
import math
import os
import queue
import shutil
import threading
import time
import zipfile

import numpy as np

STREAMS = ('gnss', 'odometry', 'localization', 'performance', 'events')
PAIR_FIELDS = ['stamp', 'received_at', 'fix_quality', 'paired', 'reason', 'epoch', 'bracket_gap_s', 'odom_age_s', 'enu', 'odom', 'gnss_odom', 'calibration', 'difference_m']


class AnalysisLimitError(ValueError):
    pass


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def atomic(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
    temp.replace(path)


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w*z+x*y), 1-2*(y*y+z*z))


def rotate(q, point):
    q = np.asarray(q, dtype=float)
    p = np.asarray(point, dtype=float)
    return p + 2*np.cross(q[:3], np.cross(q[:3], p)+q[3]*p)


def position_fix(fix):
    return (fix.get('valid_position') is True and fix.get('fix_quality') in (1, 2, 4, 5)
            and finite(fix.get('latitude')) and -90 <= fix['latitude'] <= 90
            and finite(fix.get('longitude')) and -180 <= fix['longitude'] <= 180)


def pairs_for(gnss, odometry, config):
    """No receive-time substitution, extrapolation, or pairing across resets."""
    from ..geodesy import GeoOrigin
    origin = config.get('origin')
    if not origin:
        valid = next((g for g in gnss if position_fix(g)), None)
        if not valid:
            return []
        origin = [valid['longitude'], valid['latitude'], 0.]
    geo = GeoOrigin(*origin)
    odom = sorted((o for o in odometry if o.get('source') == config['source']), key=lambda o: o['stamp'])
    stamps = [o['stamp'] for o in odom]
    result = []
    previous_stamp = None
    for fix in gnss:
        record = dict(stamp=fix.get('stamp'), received_at=fix.get('app_received_at'),
                      fix_quality=fix.get('fix_quality'), paired=False, reason='invalid_position')
        result.append(record)
        if config.get('epoch_changed'):
            record['reason'] = 'reset_requires_new_session'
            if position_fix(fix):
                record['enu'] = list(geo.enu(fix['longitude'], fix['latitude'], 0.))[:2]
            continue
        if not position_fix(fix):
            continue
        record['enu'] = list(geo.enu(fix['longitude'], fix['latitude'], 0.))[:2]
        stamp = fix.get('stamp')
        if fix.get('timestamp_source') != 'receiver_utc' or not finite(stamp):
            record['reason'] = 'receiver_time_unavailable'
            continue
        if previous_stamp is not None and stamp <= previous_stamp:
            record['reason'] = 'duplicate_or_backwards_time'
            continue
        previous_stamp = stamp
        target = stamp + config.get('gnss_to_odom_offset_s', 0.)
        i = bisect_left(stamps, target)
        if i < len(odom) and abs(stamps[i]-target) < 1e-6:
            a = b = odom[i]
        elif 0 < i < len(odom):
            a, b = odom[i-1:i+1]
        else:
            record['reason'] = 'no_bracket'
            continue
        gap = b['stamp']-a['stamp']
        if a.get('epoch') != b.get('epoch') or gap > config.get('max_pair_gap_s', .25):
            record['reason'] = 'gap_or_epoch'
            continue
        k = (target-a['stamp'])/gap if gap else 0.
        p = np.array(a['position']) * (1-k) + np.array(b['position']) * k
        # Antenna lever arm is optional; unknown is explicitly retained as unknown.
        arm = config.get('antenna_in_body_m')
        if arm is not None and config.get('odom_child_frame') != a.get('child_frame'):
            record['reason'] = 'lever_arm_frame_mismatch'
            continue
        if arm is not None:
            qa, qb = np.array(a['quaternion']), np.array(b['quaternion'])
            if np.dot(qa, qb) < 0:
                qb = -qb
            q = qa*(1-k)+qb*k
            q /= np.linalg.norm(q)
            p += rotate(q, arm)
        record.update(paired=True, reason='paired', odom=p.tolist(), epoch=a.get('epoch', 0),
                      bracket_gap_s=gap, odom_age_s=a['app_received_at']-a['stamp'])
    return result


def fit(pairs):
    valid = [p for p in pairs if p['paired']]
    if len(valid) < 8:
        raise ValueError('至少需要 8 个有时间对应的卫星点，请保持开阔并继续行走')
    if len({p['epoch'] for p in valid}) != 1:
        raise ValueError('里程计已重置，请结束本次记录并重新开始')
    a = np.array([p['enu'] for p in valid]); b = np.array([p['odom'][:2] for p in valid])
    span = float(np.linalg.norm(b[-1]-b[0]))
    if span < 10 or np.linalg.norm(a[-1]-a[0]) < 5:
        raise ValueError('校准位移不足：建议在开阔处直行 20–30 米后再确认')
    # A fixed rigid transform only. Never fit scale or re-fit evaluation data.
    x, y = a-a.mean(0), b-b.mean(0)
    angle = math.atan2(float(np.sum(x[:, 0]*y[:, 1]-x[:, 1]*y[:, 0])), float(np.sum(x*y)))
    c, s = math.cos(angle), math.sin(angle)
    r = np.array([[c, -s], [s, c]])
    # Bind the first time-paired antenna position exactly; fit only the direction.
    t = b[0]-r@a[0]
    residual = float(np.sqrt(np.mean(np.sum((a@r.T+t-b)**2, axis=1))))
    if residual > max(3., span*.2):
        raise ValueError(f'校准轨迹不一致（残差 {residual:.1f} m），请结束后重新测试')
    return dict(yaw=angle, translation=t.tolist(), calibration_end=max(p['stamp'] for p in valid),
                anchor_stamp=valid[0]['stamp'], anchor_enu=valid[0]['enu'], anchor_odom=valid[0]['odom'],
                calibration_count=len(valid), residual_m=residual, span_m=span,
                method='initial_segment_rigid_fit', frozen=True)


def projected(point, alignment):
    c, s = math.cos(alignment['yaw']), math.sin(alignment['yaw'])
    x, y = point[:2]; tx, ty = alignment['translation']
    return [c*x-s*y+tx, s*x+c*y+ty]


def analyze(gnss, odom, manifest):
    pairs = pairs_for(gnss, odom, manifest)
    alignment = manifest.get('alignment')
    errors = []
    for p in pairs:
        if alignment and 'enu' in p:
            p['gnss_odom'] = projected(p['enu'], alignment)
        if alignment and p['paired']:
            p['calibration'] = p['stamp'] <= alignment['calibration_end']
            p['difference_m'] = float(np.linalg.norm(np.array(p['gnss_odom'])-np.array(p['odom'][:2])))
            if not p['calibration']:
                errors.append(p['difference_m'])
    report = dict(total=len(pairs), paired=sum(p['paired'] for p in pairs), evaluation_count=len(errors),
                  reference='odometry, not independent ground truth',
                  lever_arm='compensated' if manifest.get('antenna_in_body_m') is not None else 'unknown: antenna versus body',
                  dropped=manifest.get('dropped', 0))
    report['quality_counts'] = {str(k):sum(g.get('fix_quality') == k for g in gnss) for k in (0,1,2,4,5)}
    report['unpaired_reasons'] = {r:sum(p['reason'] == r for p in pairs) for r in {p['reason'] for p in pairs if not p['paired']}}
    if errors:
        report.update(median_m=float(np.median(errors)), p95_m=float(np.percentile(errors, 95)), max_m=max(errors))
    return pairs, report


class TrajectoryTest:
    def __init__(self, root=None):
        self.root = Path(root or os.environ.get('NAV_TRAJECTORY_DIR') or
                         Path.home()/'.local/share/nav/trajectory-tests').absolute()
        self.queue = queue.Queue(maxsize=12000)
        self.lock = threading.RLock()
        self.active = False
        self.error = ''
        self.dropped = 0
        self.queued_bytes = 0
        self.session = None
        self.manifest = {}
        self.samples = {name: deque(maxlen=12000) for name in STREAMS}
        self.files = {}
        self.session_lock = None
        self.last_odom = {}
        self.epochs = {}
        self.closed = False
        self.worker = threading.Thread(target=self._run, name='trajectory-writer', daemon=True)
        self.worker.start()

    def call(self, name, value=None, timeout=120):
        """Use from BackgroundTask, never a ROS callback or the GUI thread."""
        done = threading.Event(); result = []
        self.queue.put(('command', (name, value, done, result)), timeout=2)
        if not done.wait(timeout):
            raise TimeoutError('轨迹文件操作尚未完成，请勿重复操作')
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]

    def feed(self, stream, data):
        if self.closed or not self.active:
            return
        size = len(repr(data))*4+256
        with self.lock:
            if self.queued_bytes+size > 8*1024**2:
                self.dropped += 1
                self.error = '记录队列已满，存在缺失样本；不影响传感器和定位'
                return
            self.queued_bytes += size
        try:
            self.queue.put_nowait((stream, (size, dict(data, app_received_monotonic=time.monotonic()))))
        except queue.Full:
            with self.lock:
                self.queued_bytes -= size
            self.dropped += 1
            self.error = '记录队列已满，存在缺失样本；不影响传感器和定位'

    def snapshot(self):
        with self.lock:
            return dict(active=self.active, error=self.error, dropped=self.dropped,
                        session=str(self.session or ''), manifest=dict(self.manifest),
                        samples={k:list(v) for k,v in self.samples.items()})

    def _run(self):
        checkpoint = time.monotonic()
        while True:
            try:
                kind, value = self.queue.get(timeout=1)
            except queue.Empty:
                kind, value = None, None
            try:
                if kind == 'command':
                    name, arg, done, result = value
                    was_active = self.active
                    try:
                        result.append(getattr(self, '_'+name)(arg))
                    except Exception as exc:
                        result.append(exc)
                        if name == 'start' and not was_active and self.session:
                            self.active = False
                            for f in self.files.values():
                                f.close()
                            self.files = {}
                            self._unlock()
                            self.manifest.update(state='incomplete', error=str(exc))
                            try:
                                atomic(self.session/'manifest.json', self.manifest)
                            except OSError:
                                pass
                    finally:
                        done.set()
                    if name == 'close':
                        return
                elif kind:
                    size, value = value
                    with self.lock:
                        self.queued_bytes -= size
                    if self.active:
                        self._sample(kind, value)
                if self.active and time.monotonic()-checkpoint >= 3:
                    if shutil.disk_usage(self.root).free < 256*1024**2:
                        raise OSError('可用磁盘不足 256 MiB，测试记录已停止')
                    for f in self.files.values():
                        f.flush()
                    self.manifest['dropped'] = self.dropped
                    atomic(self.session/'manifest.json', self.manifest)
                    checkpoint = time.monotonic()
            except Exception as exc:
                self.error = str(exc)
                self.active = False
                for f in self.files.values():
                    try:
                        f.close()
                    except OSError:
                        pass
                self.files = {}
                self._unlock()
                if self.session:
                    try:
                        self.manifest.update(state='incomplete', error=self.error, dropped=self.dropped)
                        atomic(self.session/'manifest.json', self.manifest)
                    except OSError:
                        pass

    def _start(self, config):
        if self.active:
            raise ValueError('本次测试仍在记录')
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        if shutil.disk_usage(self.root).free < 512*1024**2:
            raise OSError('开始测试至少需要 512 MiB 可用空间')
        session = self.root/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        session.mkdir(mode=0o700)
        self.session_lock = (session/'.writer.lock').open('w')
        fcntl.flock(self.session_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with self.lock:
            self.session = session
            self.manifest = dict(schema=1, started_at=time.time(), state='recording',
                                 source='fast_livo', max_pair_gap_s=.25, gnss_to_odom_offset_s=0.,
                                 antenna_in_body_m=None, alignment=None)
            self.manifest.update(config)
            self.manifest['recorder_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            self.samples = {name:deque(maxlen=12000) for name in STREAMS}
            self.epochs = {}; self.last_odom = {}; self.dropped = 0; self.error = ''
        atomic(session/'manifest.json', self.manifest)
        self.files = {s:(session/(s+'.jsonl')).open('a', encoding='utf-8') for s in STREAMS}
        # Store a bounded, display-only map snapshot, not a fake full-resolution map.
        from ..map_processing import load_grid_map, load_pointcloud
        from ..geo_maps import file_hash
        assets = {}
        if config.get('grid_path'):
            grid = load_grid_map(config['grid_path'])
            assets.update(occupancy=grid.occupancy, origin=grid.origin, resolution=grid.resolution)
            self.manifest['grid_sha256'] = file_hash(config['grid_path'])
        if config.get('cloud_path'):
            actual = file_hash(config['cloud_path'])
            if actual != config.get('map_sha256'):
                raise ValueError('场景点云已变化，请重新载入地图后开始测试')
            cloud = load_pointcloud(config['cloud_path']).points
            assets['cloud'] = cloud[::max(1,len(cloud)//20000)]
            self.manifest['preview_cloud_decimated'] = True
        if assets:
            np.savez_compressed(session/'map-preview.npz', **assets)
        atomic(session/'manifest.json', self.manifest)
        self.active = True
        return str(session)

    def _sample(self, kind, data):
        if (kind == 'localization' and self.manifest.get('map_from_odom') and
                data.get('map_sha256') and data['map_sha256'] != self.manifest.get('map_sha256')):
            self.manifest['map_changed'] = True
        if kind == 'localization' and not self.manifest.get('map_from_odom'):
            odom = self.last_odom.get(self.manifest['source'])
            if (odom and odom['frame'] == data.get('odom_frame') and
                    valid_map_quality(data, self.manifest)):
                self.manifest['map_from_odom'] = data['map_from_odom']
                self.manifest['map_odom_frame'] = data['odom_frame']
        if kind == 'odometry':
            key = data['source']; old = self.last_odom.get(key)
            if old and (data['stamp'] < old['stamp'] or data['frame'] != old['frame'] or
                        data['child_frame'] != old['child_frame'] or
                        (data['stamp']-old['stamp'] < .5 and np.linalg.norm(np.array(data['position'])-old['position']) > 10)):
                self.epochs[key] = self.epochs.get(key, 0)+1
                self._sample('events', dict(type='odometry_epoch', source=key, stamp=data['stamp']))
                if key == self.manifest['source']:
                    self.manifest['alignment'] = None
                    self.manifest['epoch_changed'] = True
            data['epoch'] = self.epochs.get(key, 0)
            self.last_odom[key] = data
        if kind == 'gnss' and not self.manifest.get('origin') and position_fix(data):
            self.manifest['origin'] = [data['longitude'], data['latitude'], 0.]
        self.files[kind].write(json.dumps(data, ensure_ascii=False, allow_nan=False)+'\n')
        with self.lock:
            if kind == 'odometry' and data['source'] != self.manifest['source']:
                return
            # Native data is kept in files; preview is bounded and decimated at 5 Hz.
            old = self.samples[kind][-1] if self.samples[kind] else None
            if kind not in ('odometry', 'localization') or not old or data.get('app_received_at', 0)-old.get('app_received_at', 0) >= .19:
                self.samples[kind].append(data)

    def _align(self, _):
        if not self.active or self.manifest.get('alignment'):
            raise ValueError('请先开始记录；已冻结的对齐不可重复拟合')
        if self.manifest.get('epoch_changed'):
            raise ValueError('本次里程计已重置，请结束并重新开始')
        for f in self.files.values():
            f.flush()
        data = load_session(self.session)
        alignment = fit(pairs_for(data['gnss'], data['odometry'], self.manifest))
        self.manifest['alignment'] = alignment
        atomic(self.session/'manifest.json', self.manifest)
        self._sample('events', dict(type='alignment_frozen', **alignment))
        return alignment

    def _preview(self, value):
        """A mapping snapshot must be in the same odometry frame, never body cloud."""
        odom = self.last_odom.get(self.manifest.get('source'))
        if (not self.active or self.manifest.get('context') != 'mapping' or not odom or
                value['frame'] != odom['frame'] or self.manifest.get('epoch_changed')):
            return False
        cloud = np.asarray(value['cloud'])
        if cloud.ndim != 2 or cloud.shape[1] < 3 or not np.isfinite(cloud[:,:3]).all():
            return False
        np.savez_compressed(self.session/'map-preview.npz', cloud=cloud[::max(1,len(cloud)//20000),:3])
        self.manifest['preview_cloud_decimated'] = True
        self.manifest['preview_frame'] = value['frame']
        return True

    def _stop(self, _):
        self.active = False
        try:
            for f in self.files.values():
                f.close()
            self.files = {}
            if not self.session:
                return None
            self.manifest.update(state='complete' if not self.error else 'incomplete', ended_at=time.time(), dropped=self.dropped, error=self.error)
            atomic(self.session/'manifest.json', self.manifest)
        finally:
            self._unlock()
        if _ is False:
            return None
        data = load_session(self.session)
        pairs, report = analyze(data['gnss'], data['odometry'], self.manifest)
        atomic(self.session/'report.json', report)
        with (self.session/'pairs.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=PAIR_FIELDS); writer.writeheader(); writer.writerows(pairs)
        write_viewer(self.session, pairs, report)
        return report

    def _unlock(self):
        if self.session_lock is not None:
            self.session_lock.close()
            self.session_lock = None

    def _close(self, _):
        self.closed = True
        if self.active:
            self._stop(False)

    def close(self):
        if not self.closed:
            try:
                self.call('close', timeout=5)
            except (TimeoutError, queue.Full):
                self.closed = True  # A recording manifest marks any unflushed tail as interrupted.


def load_session(path):
    path = Path(path)
    if path.is_symlink():
        raise ValueError('不读取符号链接测试目录')
    metadata = path/'manifest.json'
    if metadata.is_symlink() or metadata.stat().st_size > 1024*1024:
        raise ValueError('测试元数据为链接或过大')
    manifest = json.loads(metadata.read_text(encoding='utf-8'))
    data = {'manifest': manifest}
    if sum((path/(s+'.jsonl')).stat().st_size for s in STREAMS) > 64*1024**2:
        raise AnalysisLimitError('记录超过设备端 64 MiB 分析预算；原始数据已保留，请在电脑上分段分析')
    row_count = 0
    for stream in STREAMS:
        file = path/(stream+'.jsonl')
        if file.is_symlink() or file.stat().st_size > 256*1024**2:
            raise ValueError('测试文件过大或为链接，请离线分段分析')
        rows = []
        with file.open(encoding='utf-8') as f:
            for line in f:
                row_count += 1
                if row_count > 200000 or len(line) > 1024*1024:
                    raise AnalysisLimitError('记录超过设备端分析预算；原始数据已保留')
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    # A killed process may leave a truncated final line; flag, never conceal.
                    data['incomplete_lines'] = data.get('incomplete_lines', 0)+1
        data[stream] = rows
    return data


def valid_map_quality(data, manifest):
    """A recorded test transform is not authorization to publish a TF."""
    transform = data.get('map_from_odom', {})
    q, p = transform.get('rotation_xyzw', []), transform.get('translation_m', [])
    return (data.get('valid') is True and bool(manifest.get('map_sha256'))
            and data.get('odom_source') == manifest.get('source')
            and data.get('map_sha256') == manifest['map_sha256']
            and data.get('map_frame') == manifest.get('map_frame', 'map')
            and finite(data.get('data_age_s')) and -.05 <= data['data_age_s'] <= 1
            and finite(data.get('stamp')) and -.05 <= data.get('app_received_at', 0)-data['stamp'] <= 1
            and finite(data.get('last_success_stamp_ns'))
            and -.05 <= data.get('app_received_at', 0)-data['last_success_stamp_ns']/1e9 <= min(5., data.get('tracking_timeout_s',2.))
            and len(q) == 4 and all(finite(v) for v in q) and .99 < sum(v*v for v in q) < 1.01
            and len(p) == 3 and all(finite(v) for v in p))


def tracks_for(data, until=None):
    m = data['manifest']; pairs, report = analyze(data['gnss'], data['odometry'], m)
    tracks = {'gnss': [], 'odometry': [], 'localization': []}
    transform = m.get('map_from_odom')
    map_ok = bool(transform) and not m.get('epoch_changed') and not m.get('map_changed')
    def mapped(p):
        if map_ok:
            height = m.get('alignment', {}).get('anchor_odom', [0,0,0])[2]
            return (rotate(transform['rotation_xyzw'], [*p[:2], p[2] if len(p)>2 else height]) + transform['translation_m']).tolist()
        return p
    for p in pairs:
        if until is not None and finite(p.get('stamp')) and p['stamp'] > until:
            continue
        point = p.get('gnss_odom') if m.get('alignment') else p.get('enu')
        tracks['gnss'].append(mapped(point) if point is not None and m.get('alignment') else point)
    if m.get('alignment'):
        for o in data['odometry']:
            if o['source'] == m['source'] and (until is None or o['stamp'] <= until):
                p = np.array(o['position'])
                if m.get('antenna_in_body_m') is not None and o.get('child_frame') == m.get('odom_child_frame'):
                    p += rotate(o['quaternion'],m['antenna_in_body_m'])
                tracks['odometry'].append(mapped(p.tolist()))
        # Each map point uses the contemporaneous valid transform, never the frozen odom transform.
        quality = sorted((q for q in data['localization'] if finite(q.get('stamp'))), key=lambda q:q['stamp'])
        stamps = [q['stamp'] for q in quality]
        if map_ok:
            for o in data['odometry']:
                if o['source'] != m['source'] or (until is not None and o['stamp'] > until):
                    continue
                i = bisect_right(stamps, o['stamp'])
                q = quality[i-1] if i else None
                if q and valid_map_quality(q,m) and 0 <= o['stamp']-q['stamp'] <= 1 and o['frame'] == q.get('odom_frame'):
                    t = q['map_from_odom']
                    tracks['localization'].append((rotate(t['rotation_xyzw'], o['position'])+t['translation_m']).tolist())
                else:
                    tracks['localization'].append(None)
    return tracks, report, ('map' if map_ok and m.get('alignment') else 'odom' if m.get('alignment') else 'enu')


def export_session(path):
    path = Path(path)
    lock = path/'.writer.lock'
    if path.is_symlink() or lock.is_symlink():
        raise ValueError('不导出符号链接测试目录')
    with lock.open('r') as guard:
        try:
            fcntl.flock(guard, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('测试仍在记录，请结束后导出') from None
        return _export_session(path)


def _export_session(path):
    path = Path(path)
    try:
        data = load_session(path)
        pairs, report = analyze(data['gnss'], data['odometry'], data['manifest'])
        report['incomplete_lines'] = data.get('incomplete_lines', 0)
    except AnalysisLimitError as exc:
        data = {'manifest':json.loads((path/'manifest.json').read_text(encoding='utf-8'))}
        pairs, report = [], dict(analysis='deferred', reason=str(exc), reference='not independent ground truth')
    if data['manifest'].get('state') == 'recording':
        # Interrupted sessions remain exportable, explicitly labelled as such.
        data['manifest']['state'] = 'interrupted'
    csv_data = io.StringIO()
    writer = csv.DictWriter(csv_data, fieldnames=PAIR_FIELDS); writer.writeheader(); writer.writerows(pairs)
    target = path.parent/(path.name+'.zip')
    temp = target.with_suffix(f'.{time.time_ns()}.zip.partial')
    created = False
    try:
        with zipfile.ZipFile(temp, 'x', zipfile.ZIP_DEFLATED) as z:
            created = True
            temp.chmod(0o600)
            for name in [s+'.jsonl' for s in STREAMS]:
                file = path/name
                if file.is_file() and not file.is_symlink():
                    z.write(file, path.name+'/'+name)
            for name,content in (('manifest.json',json.dumps(data['manifest'],ensure_ascii=False,indent=2)),
                                 ('report.json',json.dumps(report,ensure_ascii=False,indent=2)),
                                 ('pairs.csv',csv_data.getvalue()),('viewer.html',viewer_html(pairs,report))):
                z.writestr(path.name+'/'+name,content)
            preview = path/'map-preview.npz'
            if preview.is_file() and not preview.is_symlink():
                z.write(preview, path.name+'/map-preview.npz')
        temp.replace(target)
    finally:
        if created and temp.exists():
            temp.unlink()
    return str(target)


def write_viewer(path, pairs, report):
    (path/'viewer.html').write_text(viewer_html(pairs,report), encoding='utf-8')


def viewer_html(pairs, report):
    # Self-contained offline visualization: no CDN, network, credentials, or map uploads.
    payload = json.dumps(dict(pairs=pairs[::max(1, len(pairs)//20000)], report=report), ensure_ascii=False).replace('<', '\\u003c')
    html = '''<!doctype html><meta charset="utf-8"><title>GNSS trajectory experiment</title>
<style>body{background:#10212b;color:#eee;font:16px sans-serif;margin:24px}canvas{width:100%;height:65vh;background:#07141d}pre{white-space:pre-wrap}</style>
<h2>GNSS / Odometry — relative comparison, not ground truth</h2>
<p>Green: GNSS; blue: odometry. Calibration is excluded from evaluation. No alignment: ENU only.</p>
<canvas id="plot"></canvas><input id="time" type="range" min="1" style="width:100%"><pre id="info"></pre>
<script>const data=PAYLOAD;const c=document.getElementById('plot'),ctx=c.getContext('2d'),slider=document.getElementById('time');
slider.max=data.pairs.length;slider.value=slider.max;document.getElementById('info').textContent=JSON.stringify(data.report,null,2);
function draw(){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio;let rows=data.pairs.slice(0,+slider.value),aligned=data.pairs.some(p=>p.gnss_odom);
let tracks=[rows.map(p=>p.gnss_odom||p.enu),aligned?rows.map(p=>p.odom):[]];let pts=tracks.flat().filter(Boolean);if(!pts.length)return;
let xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),x=Math.min(...xs),y=Math.min(...ys),s=Math.min((c.width-40)/Math.max(1,Math.max(...xs)-x),(c.height-40)/Math.max(1,Math.max(...ys)-y));
tracks.forEach((t,i)=>{ctx.strokeStyle=['#4de9b4','#61baff'][i];ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();let connected=false;t.forEach(p=>{if(!p){connected=false;return}let u=20+(p[0]-x)*s,v=c.height-20-(p[1]-y)*s;if(connected)ctx.lineTo(u,v);else ctx.moveTo(u,v);connected=true});ctx.stroke()});}
slider.oninput=draw;window.onresize=draw;draw();</script>'''.replace('PAYLOAD', payload)
    return html

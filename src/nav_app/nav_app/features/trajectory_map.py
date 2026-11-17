"""Display-only WGS84 positions. Never used for fusion, TF or time pairing."""
import math
from .trajectory_test import finite, position_fix

QUALITY = {1: '单点定位', 2: '差分定位', 4: 'RTK 固定解', 5: 'RTK 浮点解'}


def convert_segments(services, segments):
    """Reuse the phone's official WGS84 -> BD09 backend; preserve breaks."""
    from ..baidu_client import points_valid
    points=[p for segment in segments for p in segment]
    if not points:return []
    if not points_valid(points,40):raise ValueError('地图预览坐标无效')
    converted=[]
    for i in range(0,len(points),10):
        batch=points[i:i+10]
        result=services.call('convert',dict(points=batch))
        value=result.get('points')
        if result.get('coordinate_system')!='BD09' or not points_valid(value) or len(value)!=len(batch):
            raise ValueError('地图坐标转换结果无效')
        converted.extend(value)
    result=[]; offset=0
    for segment in segments:
        result.append(converted[offset:offset+len(segment)]); offset+=len(segment)
    return result


def map_fix(fix):
    return (isinstance(fix, dict) and fix.get('coordinate_system') == 'WGS84'
            and position_fix(fix) and abs(fix['latitude']) < 85)


def live_position(fix, received_monotonic, now):
    if not finite(received_monotonic) or not 0 <= now-received_monotonic <= 1.5:
        return None, '未收到新的卫星数据'
    if not map_fix(fix):
        return None, '正在收报，尚无有效经纬度'
    stamp, received = fix.get('stamp'), fix.get('received_at')
    aligned = (fix.get('timestamp_source') == 'receiver_utc' and finite(stamp) and finite(received)
               and -.25 <= received-stamp <= 1.5)
    quality = QUALITY[fix['fix_quality']]
    if not aligned:
        quality += ' · 设备与卫星时间未对齐，仅供位置查看'
    elif fix['fix_quality'] != 4:
        quality += ' · 非固定解，位置可能漂移'
    return [fix['longitude'], fix['latitude']], quality


def geographic_track(rows, fraction=1.):
    """Preserve acquisition order; break invalid, backwards, gaps and jumps.

    Slider is acquisition-order based here, so a bad wall clock cannot erase
    the recording. This does not alter measurement timestamps in the recorder.
    """
    count = len(rows)
    selected = rows[:max(1, math.ceil(count*max(0., min(1., fraction))))] if count else []
    segments, current, previous, last = [], [], None, None
    for fix in selected:
        if not map_fix(fix):
            if current: segments.append(current)
            current, previous = [], None
            continue
        point = [fix['longitude'], fix['latitude']]
        stamp = fix.get('stamp')
        if previous:
            before, t = previous
            dt = stamp-t if finite(stamp) and finite(t) else None
            dx = (point[0]-before[0])*111320*math.cos(math.radians(point[1]))
            dy = (point[1]-before[1])*111320
            if dt is None or not 0 < dt <= 2 or math.hypot(dx, dy) > max(10, 15*dt):
                if current: segments.append(current)
                current = []
        current.append(point); previous = (point, stamp); last = fix
    if current: segments.append(current)
    # Preview only; the recorder/export keeps every original sample.
    segments = [s for s in segments if len(s) >= 2][-8:]
    budget = max(2, 40//max(1, len(segments)))
    segments = [s if len(s) <= budget else [s[round(i*(len(s)-1)/(budget-1))] for i in range(budget)] for s in segments]
    return segments, last

"""Short operator-facing explanation of the current RTK/navigation prerequisite."""
import math
import time
from nav_rtk.parser.observation_time import time_state


def quality_label(value):
    return {0:'未定位',1:'单点定位',2:'差分定位',4:'RTK 固定解',5:'RTK 浮点解'}.get(value,'定位类型未识别')


def receiver_coordinates(observation):
    lat, lon = observation.get('latitude'), observation.get('longitude')
    if (observation.get('fix_quality') in (1,2,4,5)
            and all(type(v) in (int,float) and math.isfinite(v) for v in (lat,lon))
            and abs(lat) < 85 and abs(lon) <= 180):
        return lat, lon
    return None, None


def alignment_summary(outdoor):
    from ..gnss_alignment import alignment_configuration_ready
    if outdoor is None:
        return '方向对齐服务尚未启动'
    configured = alignment_configuration_ready(outdoor.calibration)
    status = outdoor.alignment_status()
    ready = status['heading_deg'] is not None
    rows = ['方向已对齐' if ready else '方向尚未对齐',
            '安装与时间配置：已加载' if configured else '安装与时间配置：未配置，需要填写本设备的天线安装关系',
            '当前条件：' + status['reason']]
    if ready:
        rows.append(f"当前朝向 {status['heading_deg']:.0f}°（正北为0°）")
        if status['residual_m'] is not None:
            rows.append(f"轨迹对齐残差 {status['residual_m']:.2f} 米")
    else:
        rows.append(f"已配对 {status['pairs']} 组观测，轨迹分布 {status['spread_m']:.1f} 米")
    rows.append('先取得RTK固定解，再开始采集并缓慢行走。系统需要有效精度、差分龄期及同步里程计，至少8组位置且轨迹分布达到4米，才会确认方向。')
    rows.append('接收差分数据、出现经纬度或固定解，都不会直接显示为方向已对齐。')
    return '\n\n'.join(rows)


def summary(adapter, outdoor=None):
    observation = getattr(adapter, 'observation', None)
    if observation is None:
        observation = getattr(outdoor, 'observation', None) or {}
    now = outdoor.clock() if outdoor is not None else time.time()
    stamp = observation.get('stamp')
    received = observation.get('received_at', stamp)
    current = (isinstance(received, (int,float)) and math.isfinite(received)
               and -.05 <= now - received <= 1.5)
    if not current:
        if getattr(adapter, 'frame_fresh', False):
            reason = (getattr(adapter, 'corrections', None) or {}).get('gga_rejection_reason')
            state = '卫星位置报文已过期' if observation else '尚未收到卫星位置报文'
            return state + '\n' + (reason or '等待新的卫星定位数据')
        return '等待新的卫星定位数据\n请确认天线连接，并在开阔处接收卫星信号'
    fix = observation.get('fix_quality')
    satellites = observation.get('satellites')
    satellite_text = f'报告卫星数 {satellites}' if satellites is not None else '卫星数未提供'
    quality = quality_label(fix)
    heading = f'{quality} · {satellite_text}'
    if fix not in (1,2,4,5):
        return heading + '\n接收器尚未输出有效位置，差分需等待定位\n固定解会显示在本页及“实时数据 → 定位 → 定位状态”'
    latitude, longitude = receiver_coordinates(observation)
    position = (f'纬度 {latitude:.7f} · 经度 {longitude:.7f}\n' if latitude is not None else '')
    heading += '\n' + position
    timing = time_state(observation, now)
    if timing == 'unavailable':
        return heading + '已收到位置，但卫星 UTC 日期或时间缺失，暂不用于导航'
    if timing == 'waiting':
        return heading + '位置已收到，正在等待同一时刻的数据'
    if timing != 'ready':
        return heading + f'卫星时间与设备时间相差 {abs(now-stamp):.3f} 秒，暂不用于导航'
    if observation.get('valid_position') is False:
        return heading + (observation.get('coordinate_reason') or '接收器经纬度无效，暂不用于导航')
    sigma, age = observation.get('horizontal_std_m'), observation.get('differential_age_s')
    precision = f'精度 {sigma:.2f} m' if isinstance(sigma,(float,int)) and math.isfinite(sigma) and sigma>0 else '精度未提供'
    age_text = f'龄期 {age:.1f} s' if isinstance(age,(float,int)) and math.isfinite(age) and age>=0 else '龄期未提供'
    detail=getattr(outdoor, 'alignment_reason', '等待导航定位')
    if hasattr(outdoor,'alignment_status'):
        status=outdoor.alignment_status()
        if status['heading_deg'] is not None:
            detail=f"朝向 {status['heading_deg']:.0f}°（正北 0°）"
            if status['residual_m'] is not None:detail+=f" · 对齐误差 {status['residual_m']:.2f} m"
        elif status['pairs']:
            detail+=f"\n有效观测 {status['pairs']}/至少 8 · 分布范围 {status['spread_m']:.1f}/4.0 m"
    return f'{heading}{precision} · {age_text}\n{detail}'

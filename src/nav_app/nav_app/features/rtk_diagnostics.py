"""Bounded, read-only operator diagnostics from the existing RTK subscriber."""
import math
import time
from .rtk_readiness import quality_label


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def age_text(value):
    return f'{value:.1f} 秒前' if number(value) and value >= 0 else '尚无记录'


def diagnostics(adapter, now=None):
    now = time.time() if now is None else now
    fresh = bool(getattr(adapter, 'status_fresh', False))
    receiver = getattr(adapter, 'receiver', {}) or {}
    messages = receiver.get('messages', {})
    correction = getattr(adapter, 'corrections', {}) or {}
    serial = getattr(adapter, 'serial', {}) or {}
    stats = getattr(adapter, 'stats', {}) or {}
    observation = getattr(adapter, 'observation', None) or {}
    observed = bool(getattr(adapter, 'observation_fresh', False))
    systems = {}
    raw_lines = []
    latest = None
    names = {'GP': 'GPS', 'GL': 'GLONASS', 'GA': 'Galileo', 'GB': '北斗', 'BD': '北斗', 'GN': '混合系统'}
    for key, entry in sorted(messages.items()):
        text, stamp = entry.get('raw', ''), entry.get('received_at')
        if number(stamp):
            latest = max(latest, stamp) if latest is not None else stamp
        current = fresh and number(stamp) and -.05 <= now-stamp <= 5
        raw_lines.append(('' if current else '[过期] ') + str(text)[:512])
        if current and key.endswith('GSV'):
            fields = text.split('*', 1)[0].split(',')
            try:
                count = int(fields[3])
                if 0 <= count <= 256:
                    name = names.get(key[:2], key[:2])
                    # Different signal bands and multipart GSV repeat the total.
                    systems[name] = max(systems.get(name, 0), count)
            except (ValueError, IndexError):
                pass
    in_view = '；'.join(f'{key} {value} 颗' for key, value in systems.items()) or '未知：未收到新鲜 GSV 搜星报文'
    used = observation.get('satellites')
    used_text = f'{used} 颗' if observed and number(used) and used >= 0 else '未知：等待定位报文'
    fix = quality_label(observation.get('fix_quality')) if observed else '等待新的定位报文'
    antenna = receiver.get('antenna') or {}
    antenna_age = now-antenna['received_at'] if number(antenna.get('received_at')) else None
    if fresh and number(antenna_age) and -.05 <= antenna_age <= 5:
        antenna_text = {0: '芯片报告正常', 1: '芯片报告短路', 2: '芯片报告开路'}.get(antenna.get('code'), '芯片报告未知状态')
    else:
        antenna_text = '未知：接收器未上报天线检测' if not antenna else '检测已过期，等待新的芯片报告'
    antenna_text += '；检测结果依赖接收器的天线检测电路'
    def count(key):
        value = correction.get(key)
        return str(value) if number(value) and value >= 0 else '--'
    correction_text = str(correction.get('message') or ('差分已关闭' if correction.get('enabled') is False else '等待差分状态'))
    download = f"{count('received_bytes')} 字节 / {count('rtcm_packets')} 个 RTCM 包"
    injection = f"{count('injected_bytes')} 字节 / {count('injected_packets')} 包；最近 {age_text(correction.get('last_injected_age_s'))}"
    control = correction.get('receiver_control') or {}
    commands = control.get('commands') or []
    control_text = '尚无控制操作记录'
    if commands:
        control_text = '；'.join(f"{str(row.get('command', '命令'))[:40]}：{'已确认' if row.get('acknowledged') else '未确认'}" for row in commands[-4:])
    error = serial.get('error') or getattr(adapter, 'error', '')
    warning = next((item.get('message') for item in reversed(list(getattr(adapter, 'warnings', ())))
                    if item.get('level', '').upper() in ('WARN', 'ERROR')), None)
    if not fresh:
        correction_text = '状态已过期；上次：' + correction_text
        download += '（上次统计）'
        injection += '（上次统计）'
        control_text += '（上次记录）'
    precision = observation.get('horizontal_std_m') if observed else None
    precision_text = f'{precision:.3f} 米' if number(precision) and precision > 0 else '未提供有效 GST 精度'
    differential_age = observation.get('differential_age_s') if observed else None
    raw = getattr(adapter, 'raw_gnss', {}) or {}
    raw_text = (raw.get('configuration') or {}).get('message', '等待原始测量状态')
    if raw.get('measurement_cells', 0):
        raw_text = f"已收到 {raw.get('msm_packets', 0)} 帧；含伪距和载波相位 {raw['measurement_cells']} 条"
        raw_text += '；最近 ' + age_text(now-raw['last_received_at'] if number(raw.get('last_received_at')) else None)
    if not fresh:
        raw_text = '状态已过期；' + raw_text
    recording = getattr(adapter, 'evidence', {}) or {}
    recording_text = '等待记录状态'
    if recording:
        lost = recording.get('dropped', 0) + recording.get('failed', 0)
        recording_text = f"已写入 {recording.get('written', 0)} 条；未写入 {lost} 条"
        if recording.get('error'):
            recording_text += '；记录异常，请查看日志'
        if not fresh:
            recording_text += '（上次统计）'
    return [
        ('搜到卫星（GSV）', in_view),
        ('参与定位（GGA）', used_text),
        ('定位解 / 水平精度', fix + '；' + precision_text),
        ('天线检测', antenna_text),
        ('差分连接', correction_text),
        ('差分下载', download),
        ('写入接收器', injection),
        ('原始卫星测量', raw_text),
        ('卫星星历', f"已收到 {raw.get('ephemeris_packets', 0)} 帧（本次连接进程累计）"),
        ('原始记录', recording_text),
        ('定位差分龄期', f'{differential_age:.1f} 秒' if number(differential_age) and differential_age >= 0 else '接收器未提供'),
        ('串口接收', f"{'已连接' if fresh and serial.get('ok') else '未连接或状态过期'}；解析 {stats.get('framesTotal', '--')} 帧 / 异常 {stats.get('parseErrors', '--')} 条"),
        ('卫星报文更新', age_text(max(0., now-latest) if latest is not None else None)),
        ('控制回执', control_text),
        ('最近异常', str(error or warning or '当前无已报告异常')),
    ], '\n'.join(raw_lines)

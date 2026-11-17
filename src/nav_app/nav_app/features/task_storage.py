"""Cheap offline-task capacity checks on the actual destination filesystems."""
import math
import os
from pathlib import Path
import shutil


MIB = 1024**2
GIB = 1024**3


def _existing(path):
    value = Path(path).expanduser().resolve()
    while not value.exists() and value != value.parent:
        value = value.parent
    return value if value.is_dir() else value.parent


def _temporary_bytes(kind, payload):
    # Estimate only the current task's stage/copy, never read an entire bag.
    cloud = payload.get('cloud')
    if kind in ('navigation', 'scan_context') and cloud:
        try:
            return max(64*MIB, min(4*GIB, Path(cloud).stat().st_size*2))
        except OSError:
            return 64*MIB
    if kind == 'audit' and payload.get('directory'):
        root = Path(payload['directory'])
        sizes = []
        for pattern in ('*.mcap', '*.db3'):
            for path in root.glob(pattern):
                try:
                    if path.is_file():sizes.append(path.stat().st_size)
                except OSError:pass
        return max([64*MIB, *sizes]) + 16*MIB
    return 64*MIB


def task_storage_reason(kind, payload, work_directory, *, temporary=True):
    try:
        reserve = float(os.environ.get('NAV_TASK_MIN_FREE_GIB', '2.5'))
        if not math.isfinite(reserve) or reserve <= 0:
            reserve = 2.5
    except ValueError:
        reserve = 2.5
    extra = _temporary_bytes(kind, payload) if temporary else 0
    needed = int(reserve*GIB) + extra
    paths = [work_directory]
    paths.extend(payload[key] for key in ('directory', 'cloud', 'service_root') if payload.get(key))
    seen = set()
    for raw in paths:
        try:
            path = _existing(raw)
            device = path.stat().st_dev
            if device in seen:
                continue
            seen.add(device)
            free = shutil.disk_usage(path).free
        except OSError:
            return '无法读取地图任务存储空间，请检查磁盘后重试'
        if free < needed:
            return (f'后台处理等待可用空间：剩余{free/GIB:.1f}GB，需要至少{needed/GIB:.1f}GB'
                    f'（含{reserve:g}GB采集保存余量）。请清理可删除的数据。')
    return ''

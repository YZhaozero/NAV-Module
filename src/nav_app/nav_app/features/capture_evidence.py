"""Streaming snapshots of diagnostics, including the pre-record startup window."""
import hashlib
import base64
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile
import time


def bound_record_buffers(args, largest_message, *, storage_config_path=None):
    """Enforce byte budgets even when old UI settings requested a 1 GB cache."""
    result = list(args)
    def set_option(name, value):
        if name in result:
            result[result.index(name)+1] = str(value)
        else:
            result[2:2] = [name,str(value)]
    def value(name, default):
        return result[result.index(name)+1] if name in result else default
    mcap = value("--storage", value("-s", "sqlite3")) == "mcap"
    # A 128 MiB buffer holds under one second of 30 Hz RGB + two depth
    # streams. Give chunk compression/split flush a bounded extra buffer;
    # the existing resource preflight accounts for both bounded buffers.
    cache_limit = (256 if mcap else 128)*1024**2
    cache = min(max(0,int(value("--max-cache-size",0))),cache_limit)
    set_option("--max-cache-size",cache)
    # Some existing readers decompress one whole file. Bound an individual
    # split too; there is no bound on the number of splits or capture duration.
    split=min(max(1,int(value("--max-bag-size",256*1024**2)) or 256*1024**2),256*1024**2)
    set_option("--max-bag-size",split)
    requested=value("--compression-mode","none")
    # Image compression and 200 Hz IMU shared a message-count queue. A tight
    # image-byte budget starved small IMUs. Spool to disk during acquisition;
    # stream-compress closed files only after the full raw readback audit.
    for name in ("--compression-mode","--compression-format","--compression-queue-size","--compression-threads"):
        if name in result:
            index=result.index(name);del result[index:index+2]
    if mcap and "--storage-config-file" not in result and "--storage-preset-profile" not in result:
        # Native bounded chunks avoid the separate message-count compression
        # queue and reduce write traffic before it reaches SQLite/SSD buffers.
        if storage_config_path is not None:
            set_option("--storage-config-file", storage_config_path)
        else:
            set_option("--storage-preset-profile", "zstd_fast")
    queue = 0
    return result, dict(cache_bytes=cache, cache_buffers=2,split_target_bytes=split,
                        compression_queue_messages=queue,
                        compression_queue_estimate_bytes=queue*largest_message,
                        compression_budget_bytes=64*1024**2,
                        chunk_compression=("configured" if "--storage-config-file" in result else value("--storage-preset-profile", "default")) if mcap else None,
                        post_record_compression="zstd" if not mcap and requested in ("message","file") else None)


def recorder_fault(line):
    return any(phrase in line.lower() for phrase in (
        "messages were lost", "messages lost", "no space left", "failed to write",
        "cache buffer lost", "cache buffers lost", "compression queue is full", "message is lost",
        "dropping message", "queue because it is full"))


def _evidence_inventory(started, session=None, evidence=None):
    session = session or os.environ.get('NAV_LOG_SESSION')
    evidence = evidence or os.environ.get('NAV_EVIDENCE_DIR')
    roots = [('application', Path(session))] if session else [
        ('transport', Path(evidence) if evidence else Path.home()/'.local/state/nav/evidence')]
    rows, errors = [], []
    for label, root in roots:
        if not root.is_dir():
            errors.append(f'诊断来源不存在：{root}'); continue
        for folder, dirs, files in os.walk(root, followlinks=False):
            dirs[:] = [d for d in dirs if not (Path(folder)/d).is_symlink()]
            for name in files:
                source = Path(folder)/name
                if name.startswith('.') or source.suffix not in ('.log','.json','.jsonl','.bin'):
                    continue
                try:
                    info = source.lstat()
                    if not stat.S_ISREG(info.st_mode) or info.st_mtime < started-120:
                        continue
                    length = info.st_size
                    if name.startswith('packets-'): length -= length%80
                    rows.append(dict(path=str(Path(label)/source.relative_to(root)), source=str(source.resolve()),
                        bytes=length, device=info.st_dev, inode=info.st_ino))
                except OSError as exc:
                    errors.append(f'{source}: {exc}')
    return rows, errors


def snapshot_evidence(output, started, session=None, evidence=None):
    """Pin existing file identities and byte boundaries without copying data.

    Same-filesystem hardlinks keep rotated log segments alive. Across filesystems
    the original is referenced explicitly and later verified; a vanished source
    becomes a diagnostic error, never an invented successful archive.
    """
    rows, errors = _evidence_inventory(started, session, evidence)
    pins = Path(output)/'.diagnostic-snapshot'
    pins.mkdir(mode=0o700, parents=True, exist_ok=True)
    for index, row in enumerate(rows):
        source = Path(row['source'])
        target = pins/str(index)
        try:
            os.link(source, target, follow_symlinks=False)
            info = target.lstat()
            if not stat.S_ISREG(info.st_mode) or (info.st_dev,info.st_ino)!=(row['device'],row['inode']):
                target.unlink(missing_ok=True)
                raise OSError('诊断文件在快照时发生变化')
            row['pin'] = str(target)
        except OSError as exc:
            row['unpinned_reason'] = str(exc)
    if os.name == 'posix':
        fd=os.open(pins,os.O_RDONLY|os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)
    return dict(schema_version=1, snapshot_unix=time.time(), startup_window_s=120, files=rows, errors=errors)


def collect_evidence(output, started, session=None, evidence=None, *, snapshot=None, cancelled=None, progress=None):
    """Finite snapshots, 64 KiB IO buffers, no entire log/bag loaded into RAM.

    Copy files touched during capture or the preceding 120 seconds (whole
    segments retain earlier startup context). Running writers remain open;
    their status/byte boundary is explicit, not claimed as a closed trace.
    """
    if snapshot is None:
        rows, errors = _evidence_inventory(started, session, evidence)
        snapshot = dict(snapshot_unix=time.time(), files=rows, errors=errors)
    report = dict(schema_version=1, startup_window_s=120, snapshot_unix=snapshot['snapshot_unix'],
                  files=[], errors=list(snapshot.get('errors', [])))
    destination = Path(output)/"diagnostics"
    destination.mkdir(mode=0o700,parents=True,exist_ok=True)
    sources=snapshot.get('files', [])
    for index, item in enumerate(sources):
        if progress is not None:progress(index, len(sources))
        source = Path(item.get('pin') or item['source'])
        temporary = None
        try:
            if cancelled is not None and cancelled(): raise InterruptedError('诊断归档已暂停')
            relative = Path(item['path'])
            target = destination/relative
            if relative.is_absolute() or '..' in relative.parts or not target.resolve().is_relative_to(destination.resolve()):
                raise ValueError('诊断归档路径无效')
            target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
            length = int(item['bytes'])
            digest = hashlib.sha256(); copied = 0
            if shutil.disk_usage(destination).free-length < 256*1024**2:
                raise OSError('诊断复制剩余磁盘不足256 MiB')
            fd = os.open(source,os.O_RDONLY|getattr(os,'O_NOFOLLOW',0))
            with os.fdopen(fd,'rb') as src:
                info = os.fstat(src.fileno())
                if not stat.S_ISREG(info.st_mode) or (info.st_dev,info.st_ino)!=(item['device'],item['inode']):
                    raise ValueError('诊断来源已被替换，未归档后续行程的数据')
                with tempfile.NamedTemporaryFile(dir=target.parent,prefix='.archive-',delete=False) as dst:
                    temporary=Path(dst.name)
                    while copied < length:
                        if cancelled is not None and cancelled(): raise InterruptedError('诊断归档已暂停')
                        data = src.read(min(65536,length-copied))
                        if not data: raise OSError('诊断文件在复制时缩短')
                        dst.write(data); digest.update(data); copied += len(data)
                    dst.flush(); os.fsync(dst.fileno())
            os.replace(temporary,target); temporary=None
            row = dict(path=str(relative),bytes=copied,sha256=digest.hexdigest(),source=item['source'])
            report['files'].append(row)
            if relative.name == 'status.json' and length < 65536:
                status = json.loads(target.read_text())
                row['writer_status'] = status
                pid_parts = Path(item['source']).parent.name.split('-')
                if len(pid_parts)==3 and pid_parts[1].isdigit() and not status.get('closed'):
                    row['source_process_alive'] = Path('/proc',pid_parts[1]).exists()
                    if not row['source_process_alive']:
                        report['errors'].append(f'诊断流未正常结束：{relative}')
                if any(status.get(k) for k in ('dropped','failed','write_failed_records','error')):
                    report['errors'].append(f'诊断流有缺失：{relative}')
        except InterruptedError:
            raise
        except (OSError,ValueError) as exc:
            report['errors'].append(f'{source}: {exc}')
        finally:
            if temporary is not None:temporary.unlink(missing_ok=True)
    if not report["files"]:
        report["errors"].append("没有可归档的启动或传输诊断日志")
    # These are receiver transport bytes, not proof of rover carrier/pseudorange
    # observations. Count the archived snapshot, including its startup context.
    raw = dict(serial_rx_bytes=0, serial_tx_bytes=0, ntrip_body_rx_bytes=0,
               rover_rtcm_bytes=0, msm_packets=0, measurement_cells=0,
               ephemeris_packets=0, rover_message_types={},
               sessions=0, malformed_records=0, includes_startup_context=True,
               rover_observations_verified=False)
    sessions = set()
    for item in report['files']:
        path = destination / item['path']
        if not path.name.startswith('events-') or not path.parent.name.startswith('rtk-'):
            continue
        sessions.add(path.parent)
        with path.open('rb') as stream:
            for line in stream:
                if cancelled is not None and cancelled(): raise InterruptedError('诊断解析已暂停')
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError('not a transport record')
                    key = str(row.get('event')) + '_bytes'
                    if key not in raw:
                        continue
                    payload = base64.b64decode(row['base64'], validate=True)
                    if len(payload) != row.get('bytes'):
                        raise ValueError('length mismatch')
                    if key == 'rover_rtcm_bytes':
                        from nav_rtk.parser.raw_gnss import describe_rtcm
                        details = describe_rtcm(payload)
                        kind = str(details['message_type'])
                        raw['rover_message_types'][kind] = raw['rover_message_types'].get(kind, 0) + 1
                        raw['msm_packets'] += int(details.get('msm', False))
                        raw['measurement_cells'] += details.get('measurement_cells', 0)
                        raw['ephemeris_packets'] += int(details['ephemeris'])
                    raw[key] += len(payload)
                except (ValueError, KeyError, TypeError):
                    raw['malformed_records'] += 1
    raw['sessions'] = len(sessions)
    raw['rover_observations_verified'] = raw['measurement_cells'] > 0
    report['rtk_raw'] = raw
    if cancelled is not None and cancelled():raise InterruptedError('诊断归档已暂停')
    if progress is not None:progress(len(sources),len(sources))
    return report

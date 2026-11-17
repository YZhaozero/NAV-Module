"""Discover and export interrupted map checkpoints without touching sensors."""
from datetime import datetime
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import uuid

from ..map_storage import unique_map_file
from .map_commit import binary_pcd_dimensions, committed_map_receipt
from .resource_lease import ResourceLease


_CLOSED_MARKER = 'closed_capture.json'
_COMMIT_KEYS = ('receipt', 'bytes', 'points', 'source_spool', 'checkpoint_chunks',
                'archive_generation', 'file_fingerprint', 'receipt_fingerprint', 'receipt_sha256')


def _fingerprint(path):
    value = path.lstat()
    if not stat.S_ISREG(value.st_mode):
        raise ValueError('采集文件不是普通文件')
    return [value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns]


def _small_file(path, limit=16384):
    before = _fingerprint(path)
    if before[2] > limit:
        raise ValueError('采集记录过大')
    with path.open('rb') as stream:
        opened = os.fstat(stream.fileno())
        if before != [opened.st_dev, opened.st_ino, opened.st_size,
                      opened.st_mtime_ns, opened.st_ctime_ns]:
            raise ValueError('采集文件已变化')
        data = stream.read(limit + 1)
    if len(data) > limit or _fingerprint(path) != before:
        raise ValueError('采集文件已变化')
    return data, before


def _source_directory(source, root):
    directory = Path(source)
    allowed = Path(root or checkpoint_root()).resolve()
    if (not directory.is_absolute() or directory.is_symlink() or not directory.is_dir()
            or directory.resolve().parent != allowed or not directory.name.startswith('capture-')):
        raise ValueError('保存回执不属于本机采集检查点')
    return directory.resolve()


def _spool_fingerprint(directory):
    """Read only metadata and stat immutable chunks; never read their payloads."""
    data, metadata_fingerprint = _small_file(directory / 'map.meta')
    fields = data.decode('ascii').split()
    if (len(fields) != 3 or fields[0] != '1' or not math.isfinite(float(fields[1]))
            or float(fields[1]) < .01 or not fields[2].isdigit()):
        raise ValueError('采集检查点元数据无效')
    chunks = []
    for path in directory.glob('points-*'):
        # An incomplete write must remain visible for recovery/diagnosis.
        suffix = path.name[7:]
        if not suffix.endswith('.bin') or not suffix[:-4].isdigit():
            raise ValueError('采集检查点仍有未完成片段')
        index = int(suffix[:-4])
        if path.name != f'points-{index}.bin':
            raise ValueError('采集片段名称无效')
        identity = _fingerprint(path)
        if identity[2] % 16:
            raise ValueError('采集片段长度不完整')
        chunks.append((index, dict(name=path.name, fingerprint=identity)))
    chunks.sort(key=lambda item: item[0])
    if (not chunks or [item[0] for item in chunks] != list(range(len(chunks)))
            or int(fields[2]) != len(chunks) or (directory / 'map.meta.partial').exists()):
        raise ValueError('采集检查点数量不完整')
    if _fingerprint(directory / 'map.meta') != metadata_fingerprint:
        raise ValueError('采集检查点仍在变化')
    return dict(metadata=dict(fingerprint=metadata_fingerprint,
                              sha256=hashlib.sha256(data).hexdigest()),
                chunks=[item[1] for item in chunks])


def _normal_exit(exit_receipt, run_id):
    return (isinstance(exit_receipt, dict) and type(run_id) is int and run_id > 0
            and type(exit_receipt.get('run_id')) is int and exit_receipt['run_id'] == run_id
            and all(exit_receipt.get(key) is True for key in
                    ('normal_stop', 'expected_stop', 'group_exited'))
            and all(exit_receipt.get(key) is False for key in
                    ('forced_kill', 'failure_seen', 'term_escalated', 'start_failed'))
            and type(exit_receipt.get('exit_code')) is int and exit_receipt['exit_code'] == 0
            and exit_receipt.get('exit_status') == 'normal')


def _current_commit(path, frozen):
    if not isinstance(frozen, dict) or any(key not in frozen for key in _COMMIT_KEYS):
        raise ValueError('保存回执缺少文件校验信息')
    current = committed_map_receipt(path, prior_receipts={})
    if (not current or current.get('source') != 'native_commit_receipt'
            or any(current.get(key) != frozen[key] for key in _COMMIT_KEYS)):
        raise ValueError('已保存地图或回执已变化，检查点已保留')
    return {key: current[key] for key in _COMMIT_KEYS}


@contextmanager
def _closed_spool_lock(directory):
    """Hold the native writer's lock through snapshot and atomic publication."""
    import fcntl  # Production capture is Linux. Never silently skip exclusion.
    lock_path = directory / 'active.lock'
    before = _fingerprint(lock_path)
    fd = os.open(lock_path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        def check_identity():
            value = os.fstat(fd)
            opened = [value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns]
            if opened != before or _fingerprint(lock_path) != before:
                raise ValueError('采集锁文件已变化，检查点已保留')
        check_identity()
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('采集仍在运行，检查点已保留') from exc
        check_identity()
        yield check_identity
    finally:
        os.close(fd)


def _sync_directory(directory):
    fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def mark_capture_closed(record, exit_receipt, root=None):
    """Close only this normally saved/stopped capture, retaining every raw chunk.

    The UI freezes save_end_intent before stop and passes the matching run's
    immutable process receipt. File identities originate from the save ACK,
    not from discovery here. A late result can therefore only close its own
    native source_spool. Failure leaves the tail available for recovery.
    """
    if not isinstance(record, dict):
        raise ValueError('采集结果无效')
    intent = record.get('save_end_intent')
    receipt = record.get('map_save_receipt')
    if not isinstance(intent, dict) or not isinstance(receipt, dict):
        raise ValueError('缺少本次保存并结束的回执')
    if (not isinstance(record.get('capture_id'), str) or not record['capture_id']
            or intent.get('capture_id') != record['capture_id']
            or type(intent.get('attempt_id')) is not int or intent['attempt_id'] <= 0
            or type(receipt.get('attempt_id')) is not int
            or receipt['attempt_id'] != intent['attempt_id'] or receipt.get('success') is not True
            or record.get('finished') is not True or record.get('error')
            or receipt.get('path') != record.get('map')):
        raise ValueError('保存结果与本次采集不一致')
    exit_receipt = dict(exit_receipt or {})
    if not _normal_exit(exit_receipt, intent.get('run_id')):
        raise ValueError('尚未确认本次建图服务正常结束')
    requested = record.get('record_requested')
    if type(requested) is not bool or (requested and not (
            record.get('bag_stored') is True and record.get('storage_state') == 'stored'
            and record.get('record_stop_confirmed') is True)):
        raise ValueError('原始录包尚未确认保存并关闭')
    evidence = receipt.get('evidence')
    if (not isinstance(evidence, dict) or evidence.get('source') != 'service_response'
            or evidence.get('commit_source') != 'native_commit_receipt'):
        raise ValueError('缺少正常保存及原生提交回执')
    path = Path(record['map'])
    if not path.is_absolute():
        raise ValueError('地图保存路径无效')
    directory = _source_directory(evidence.get('source_spool', ''), root)
    with _closed_spool_lock(directory) as check_lock:
        commit = _current_commit(path, evidence)
        snapshot = _spool_fingerprint(directory)
        if not 0 < commit['checkpoint_chunks'] <= len(snapshot['chunks']):
            raise ValueError('保存回执与采集检查点数量不一致')
        marker = dict(schema_version=1, kind='normal_capture_closed_tail',
                      capture_id=record['capture_id'], attempt_id=intent['attempt_id'],
                      run_id=intent['run_id'], source_spool=str(directory), map=str(path),
                      record_requested=requested, record_stop_confirmed=record.get('record_stop_confirmed') is True,
                      bag_stored=record.get('bag_stored') is True, storage_state=record.get('storage_state'),
                      exit_receipt=exit_receipt, commit=commit, spool=snapshot)
        marker_path = directory / _CLOSED_MARKER
        if marker_path.exists() or marker_path.is_symlink():
            # Retried callbacks may confirm the identical closed capture, but
            # an old exit receipt cannot endorse chunks appended afterwards.
            try:
                prior, _ = _small_file(marker_path, limit=16 * 1024 * 1024)
                if json.loads(prior) == marker:
                    check_lock()
                    return dict(marker, marker=str(marker_path))
            except (OSError, ValueError, TypeError, UnicodeError, RecursionError):
                pass
            raise ValueError('已结束采集的检查点发生变化，请从恢复列表处理')
        fd, temporary = tempfile.mkstemp(prefix='.closed-capture-', suffix='.tmp', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(marker, stream, ensure_ascii=False, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            # Detect changes even if a non-cooperating writer ignored active.lock.
            if _spool_fingerprint(directory) != snapshot:
                raise ValueError('采集检查点仍在变化')
            _current_commit(path, commit)
            check_lock()
            os.replace(temporary, directory / _CLOSED_MARKER)
            try:
                _sync_directory(directory)
            except OSError:
                # A marker whose publication was not confirmed durable must
                # not suppress recovery in this still-running application.
                (directory / _CLOSED_MARKER).unlink()
                raise
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
    return dict(marker, marker=str(marker_path))


def _valid_closed_capture(directory, root):
    """A broken/stale marker never suppresses an unfinished capture."""
    try:
        data, _ = _small_file(directory / _CLOSED_MARKER, limit=16 * 1024 * 1024)
        marker = json.loads(data)
        if (not isinstance(marker, dict) or type(marker.get('schema_version')) is not int
                or marker['schema_version'] != 1 or marker.get('kind') != 'normal_capture_closed_tail'
                or not isinstance(marker.get('capture_id'), str) or not marker['capture_id']
                or type(marker.get('attempt_id')) is not int or marker['attempt_id'] <= 0
                or marker.get('source_spool') != str(directory.resolve())
                or not _normal_exit(marker.get('exit_receipt'), marker.get('run_id'))
                or type(marker.get('record_requested')) is not bool):
            return False
        if marker['record_requested'] and not (marker.get('bag_stored') is True
                and marker.get('storage_state') == 'stored' and marker.get('record_stop_confirmed') is True):
            return False
        if _source_directory(marker['source_spool'], root) != directory.resolve():
            return False
        commit = _current_commit(Path(marker['map']), marker['commit'])
        if Path(commit['source_spool']).resolve() != directory.resolve():
            return False
        snapshot = _spool_fingerprint(directory)
        return (snapshot == marker['spool'] and 0 < commit['checkpoint_chunks'] <= len(snapshot['chunks'])
                and not _active(directory))
    except (OSError, ValueError, TypeError, KeyError, UnicodeError, RecursionError):
        return False


def checkpoint_root():
    return Path.home()/'.local/state/nav/map-spool'


def checkpoint_snapshot(root=None):
    return [str(path.resolve()) for path in Path(root or checkpoint_root()).glob('capture-*')
            if path.is_dir() and not path.is_symlink()]


def capture_checkpoint(before, started_at, root=None):
    """Only identify one newly created, closed session belonging to this capture."""
    known=set(before)
    candidates=[row for row in recovery_candidates(root)
                if row['path'] not in known and row['modified'] >= started_at-1.]
    return candidates[0]['path'] if len(candidates)==1 else None


def _active(directory):
    try:
        import fcntl
    except ImportError:
        return False  # Map capture runs on Linux; allow offline inspection elsewhere.
    try:
        with (directory/'active.lock').open('rb') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            fcntl.flock(lock, fcntl.LOCK_UN)
    except FileNotFoundError:
        pass  # Legacy captures did not have a lock file.
    return False


def recovery_candidates(root=None, include_closed=False):
    rows = []
    for directory in Path(root or checkpoint_root()).glob('capture-*'):
        if not directory.is_dir() or directory.is_symlink():
            continue
        try:
            if _active(directory):
                continue
            exported = directory/'exported.txt'
            marker = directory / _CLOSED_MARKER
            # Legacy exports keep their original behaviour. Once a closed-tail
            # marker exists, stale fingerprints must not be hidden by the old
            # weaker filename-only export marker.
            if (not marker.exists() and not marker.is_symlink() and exported.is_file()
                    and Path(exported.read_text().strip()).is_file()):
                continue
            chunks = list(directory.glob('points-*.bin'))
            if not chunks:
                continue
            closed = _valid_closed_capture(directory, root)
            if closed and not include_closed:
                continue
            latest = max(p.stat().st_mtime for p in chunks)
            rows.append(dict(path=str(directory), chunks=len(chunks),
                bytes=sum(p.stat().st_size for p in chunks), modified=latest,
                closed=closed,
                label=datetime.fromtimestamp(latest).strftime('%m月%d日 %H:%M:%S') +
                    (' · 已结束采集的在途检查点' if closed else f' · {len(chunks)} 个已保存片段')))
        except OSError:
            continue
    return sorted(rows, key=lambda row: row['modified'], reverse=True)


def _archive_closed_marker_after_recovery(directory, output):
    """A new explicit export supersedes the old closure, retaining its audit."""
    marker = directory / _CLOSED_MARKER
    if not marker.exists() and not marker.is_symlink():
        return
    with _closed_spool_lock(directory) as check_lock:
        try:
            before = _fingerprint(marker)
            exported, _ = _small_file(directory / 'exported.txt')
            if exported.decode('utf-8').strip() != str(output):
                return
            binary_pcd_dimensions(output)
        except (OSError, ValueError, UnicodeError):
            return  # A success code alone cannot supersede the old checkpoint.
        archive = directory / ('closed_capture.recovered-' + uuid.uuid4().hex + '.json')
        # Reserve a unique destination. The later rename only replaces our own
        # empty reservation, never an earlier closure audit.
        fd = os.open(archive, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        os.close(fd)
        renamed = False
        try:
            if _fingerprint(marker) != before:
                raise ValueError('检查点结束记录已变化，请重新查看恢复列表')
            check_lock()
            os.replace(marker, archive)
            renamed = True
            _sync_directory(directory)
        finally:
            if not renamed:
                archive.unlink()


def recover_checkpoint(directory, destination, executable=None, root=None):
    directory = Path(directory).resolve()
    allowed = Path(root or checkpoint_root()).resolve()
    if directory.parent != allowed or not directory.name.startswith('capture-'):
        raise ValueError('请选择列表中的待恢复地图')
    if executable is None:
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('fast_livo'))/'lib/fast_livo/recover_map'
    output = unique_map_file(Path(destination), 'recovered_map', '.pcd')
    # Exclusive lease prevents recovery IO competing with a live acquisition.
    with ResourceLease('model'):
        result = subprocess.run([str(executable), str(directory), str(output)],
                                capture_output=True, text=True, check=False)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()[-500:]
        raise RuntimeError('地图恢复失败，原始片段已保留：' + detail)
    if not output.is_file():
        raise RuntimeError('恢复程序未生成地图，原始片段已保留')
    try:
        binary_pcd_dimensions(output)
    except (OSError, ValueError, KeyError, IndexError, UnicodeError) as exc:
        raise RuntimeError('恢复程序生成的地图不完整，原始片段已保留') from exc
    _archive_closed_marker_after_recovery(directory, output)
    return str(output)

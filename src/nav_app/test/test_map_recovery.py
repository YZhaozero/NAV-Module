from pathlib import Path
import copy
from contextlib import nullcontext
import json
import os
import struct
from types import MappingProxyType
import numpy as np
import pytest
from nav_app.map_processing import load_pointcloud, save_pcd
from nav_app.features.map_recovery import recovery_candidates, recover_checkpoint
from nav_app.features.map_recovery import mark_capture_closed
from nav_app.features.map_commit import committed_map_receipt


def test_binary_map_preview_preserves_full_file_for_navigation(tmp_path):
    points=np.column_stack((np.arange(300001,dtype=np.float32)*.05, np.zeros(300001),np.ones(300001)))
    p=save_pcd(tmp_path/'large.pcd',points)
    preview=load_pointcloud(p,max_points=120000)
    full=load_pointcloud(p)
    assert len(preview.points)<=120000 and preview.source_point_count==300001
    assert len(full.points)==300001
    np.testing.assert_array_equal(full.points,points)
    assert preview.points[-1,0]>14999
    p.write_bytes(p.read_bytes()[:-1])
    with pytest.raises(ValueError,match='数据长度'):load_pointcloud(p,max_points=120000)


def test_candidates_exclude_successful_exports_but_keep_missing_output(tmp_path):
    root=tmp_path/'spool';d=root/'capture-one';d.mkdir(parents=True)
    (d/'points-0.bin').write_bytes(b'0'*16)
    (d/'points-1.bin.partial').write_bytes(b'broken')
    rows=recovery_candidates(root);assert len(rows)==1 and rows[0]['chunks']==1
    out=tmp_path/'exported.pcd';out.touch()
    (d/'exported.txt').write_text(str(out))
    assert not recovery_candidates(root)
    out.unlink();assert len(recovery_candidates(root))==1


def test_recovery_rejects_foreign_directory(tmp_path):
    with pytest.raises(ValueError):
        recover_checkpoint(tmp_path/'foreign',tmp_path,executable='unused',root=tmp_path/'spool')


def test_running_capture_is_not_reported_as_interrupted(tmp_path):
    fcntl = pytest.importorskip('fcntl')
    directory = tmp_path/'capture-live'; directory.mkdir()
    (directory/'points-0.bin').write_bytes(b'0'*16)
    with (directory/'active.lock').open('wb') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert recovery_candidates(tmp_path) == []
    assert len(recovery_candidates(tmp_path)) == 1


def test_auto_recovery_does_not_pick_an_older_or_ambiguous_session(tmp_path):
    from nav_app.features.map_recovery import checkpoint_snapshot,capture_checkpoint
    old=tmp_path/'capture-old';old.mkdir();(old/'points-0.bin').write_bytes(b'0'*16)
    before=checkpoint_snapshot(tmp_path)
    fresh=tmp_path/'capture-new';fresh.mkdir();(fresh/'points-0.bin').write_bytes(b'0'*16)
    assert capture_checkpoint(before,0.,tmp_path)==str(fresh)
    another=tmp_path/'capture-other';another.mkdir();(another/'points-0.bin').write_bytes(b'0'*16)
    assert capture_checkpoint(before,0.,tmp_path) is None


def native_capture(tmp_path, name='one'):
    """A saved snapshot followed by two real, retained in-flight tail chunks."""
    root = tmp_path / 'spool'
    directory = root / ('capture-' + name)
    directory.mkdir(parents=True)
    (directory / 'active.lock').touch()
    (directory / 'map.meta').write_text('1 0.05 3\n')
    for index in range(3):
        (directory / f'points-{index}.bin').write_bytes(struct.pack('<ffff', float(index), 2., 3., 4.))
    path = tmp_path / f'map-{name}.pcd'
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              'COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n')
    path.write_bytes(header.encode('ascii') + struct.pack('<ffff', 0., 2., 3., 4.))
    sidecar = Path(str(path) + '.commit.json')
    sidecar.write_text(json.dumps(dict(schema_version=1, output_path=str(path),
        bytes=path.stat().st_size, points=1, source_spool=str(directory),
        checkpoint_chunks=1, archive_generation=7)))
    evidence = committed_map_receipt(path, prior_receipts={})
    assert evidence and evidence['source'] == 'native_commit_receipt'
    evidence = dict(evidence, source='service_response', commit_source='native_commit_receipt')
    record = dict(capture_id='capture-' + name, map=str(path), finished=True,
        record_requested=True, bag_stored=True, storage_state='stored', record_stop_confirmed=True,
        save_end_intent=dict(capture_id='capture-' + name, attempt_id=3, run_id=2),
        map_save_receipt=dict(success=True, attempt_id=3, path=str(path), evidence=evidence))
    exit_receipt = dict(run_id=2, normal_stop=True, expected_stop=True, group_exited=True,
        forced_kill=False, failure_seen=False, term_escalated=False, start_failed=False,
        exit_code=0, exit_status='normal')
    return root, directory, record, exit_receipt


@pytest.fixture
def closed_capture(tmp_path):
    pytest.importorskip('fcntl', reason='native checkpoint writer and lock run on Linux')
    return native_capture(tmp_path)


def test_closed_tail_is_optional_recovery_and_preserves_every_chunk(closed_capture):
    root, directory, record, exit_receipt = closed_capture
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    marker = mark_capture_closed(record, MappingProxyType(exit_receipt), root)
    assert marker['capture_id'] == record['capture_id']
    assert marker['marker'] == str(directory / 'closed_capture.json')
    assert 'marker' not in json.loads((directory / 'closed_capture.json').read_text())
    assert marker['commit']['checkpoint_chunks'] == 1
    assert len(marker['spool']['chunks']) == 3
    assert not recovery_candidates(root)
    rows = recovery_candidates(root, include_closed=True)
    assert len(rows) == 1 and rows[0]['closed'] and rows[0]['chunks'] == 3
    assert '已结束采集的在途检查点' in rows[0]['label']
    assert {name: (directory / name).read_bytes() for name in before} == before
    assert not list(directory.glob('.closed-capture-*.tmp'))
    assert mark_capture_closed(record, exit_receipt, root) == marker


def test_map_only_capture_can_close_without_a_bag(closed_capture):
    root, _, record, exit_receipt = closed_capture
    record.update(record_requested=False, bag_stored=False, storage_state=None, record_stop_confirmed=False)
    mark_capture_closed(record, exit_receipt, root)
    assert not recovery_candidates(root)


def test_closed_marker_overrides_legacy_export_and_invalid_marker_reappears(closed_capture):
    root, directory, record, exit_receipt = closed_capture
    (directory / 'exported.txt').write_text(record['map'])
    mark_capture_closed(record, exit_receipt, root)
    assert not recovery_candidates(root)
    assert len(recovery_candidates(root, include_closed=True)) == 1
    (directory / 'map.meta').write_text('1 0.06 3\n')
    rows = recovery_candidates(root)
    assert len(rows) == 1 and not rows[0]['closed']


def test_old_duplicate_receipt_cannot_reclose_new_tail(closed_capture):
    root, directory, record, exit_receipt = closed_capture
    mark_capture_closed(record, exit_receipt, root)
    marker_before = (directory / 'closed_capture.json').read_bytes()
    (directory / 'points-3.bin').write_bytes(b'\0' * 16)
    (directory / 'map.meta').write_text('1 0.05 4\n')
    with pytest.raises(ValueError, match='发生变化'):
        mark_capture_closed(record, exit_receipt, root)
    assert (directory / 'closed_capture.json').read_bytes() == marker_before
    assert len(recovery_candidates(root)) == 1


@pytest.mark.parametrize('field,value', [
    ('normal_stop', False), ('expected_stop', False), ('group_exited', False),
    ('forced_kill', True), ('failure_seen', True), ('term_escalated', True),
    ('start_failed', True), ('exit_code', 1), ('exit_status', 'crash'), ('run_id', 3),
    ('normal_stop', 1), ('exit_code', False),
])
def test_abnormal_or_foreign_exit_cannot_close_capture(closed_capture, field, value):
    root, directory, record, exit_receipt = closed_capture
    exit_receipt[field] = value
    with pytest.raises(ValueError):
        mark_capture_closed(record, exit_receipt, root)
    assert not (directory / 'closed_capture.json').exists()
    assert len(recovery_candidates(root)) == 1


@pytest.mark.parametrize('field,value', [
    ('finished', False), ('bag_stored', False), ('storage_state', 'partial'),
    ('record_stop_confirmed', False), ('record_requested', None),
    ('capture_id', 'another'), ('error', '录制提前结束'),
])
def test_unfinished_recording_or_wrong_capture_stays_recoverable(closed_capture, field, value):
    root, _, record, exit_receipt = closed_capture
    record[field] = value
    with pytest.raises(ValueError):
        mark_capture_closed(record, exit_receipt, root)
    assert len(recovery_candidates(root)) == 1


@pytest.mark.parametrize('change', ['attempt', 'map', 'response', 'source', 'legacy', 'fingerprint'])
def test_only_matching_normal_save_receipt_can_close_tail(closed_capture, change):
    root, _, record, exit_receipt = closed_capture
    receipt = record['map_save_receipt']
    if change == 'attempt':
        receipt['attempt_id'] += 1
    elif change == 'map':
        record['map'] += '.different'
    elif change == 'response':
        receipt['success'] = False
    elif change == 'source':
        receipt['evidence']['source'] = 'native_commit_receipt'
    elif change == 'legacy':
        receipt['evidence']['commit_source'] = 'legacy_export_receipt'
    else:
        receipt['evidence'].pop('file_fingerprint')
    with pytest.raises(ValueError):
        mark_capture_closed(record, exit_receipt, root)
    assert len(recovery_candidates(root)) == 1


@pytest.mark.parametrize('target', ['map', 'receipt'])
def test_map_or_receipt_replacement_after_ack_cannot_close_tail(closed_capture, target):
    root, _, record, exit_receipt = closed_capture
    path = Path(record['map'] if target == 'map' else record['map_save_receipt']['evidence']['receipt'])
    before = path.stat()
    replacement = path.with_name(path.name + '.new')
    replacement.write_bytes(path.read_bytes())
    os.utime(replacement, ns=(before.st_atime_ns, before.st_mtime_ns))
    os.replace(replacement, path)
    with pytest.raises(ValueError, match='变化'):
        mark_capture_closed(record, exit_receipt, root)
    assert len(recovery_candidates(root)) == 1


@pytest.mark.parametrize('change', ['map_deleted', 'map_truncated', 'receipt_deleted', 'receipt_changed',
    'new_chunk', 'metadata', 'chunk_replace', 'partial', 'marker_json', 'marker_fields'])
def test_changed_files_invalidate_closed_marker(closed_capture, change):
    root, directory, record, exit_receipt = closed_capture
    mark_capture_closed(record, exit_receipt, root)
    path = Path(record['map'])
    receipt_path = Path(record['map_save_receipt']['evidence']['receipt'])
    if change == 'map_deleted':
        path.unlink()
    elif change == 'map_truncated':
        path.write_bytes(path.read_bytes()[:-1])
    elif change == 'receipt_deleted':
        receipt_path.unlink()
    elif change == 'receipt_changed':
        receipt_path.write_text(receipt_path.read_text() + ' ')
    elif change == 'new_chunk':
        (directory / 'points-3.bin').write_bytes(b'\0' * 16)
        (directory / 'map.meta').write_text('1 0.05 4\n')
    elif change == 'metadata':
        (directory / 'map.meta').write_text('1 0.06 3\n')
    elif change == 'chunk_replace':
        chunk = directory / 'points-2.bin'
        prior = chunk.stat()
        replacement = directory / 'replace.tmp'
        replacement.write_bytes(chunk.read_bytes())
        os.utime(replacement, ns=(prior.st_atime_ns, prior.st_mtime_ns))
        os.replace(replacement, chunk)
    elif change == 'partial':
        (directory / 'points-3.bin.partial').write_bytes(b'incomplete')
    elif change == 'marker_json':
        (directory / 'closed_capture.json').write_text('{')
    else:
        marker = json.loads((directory / 'closed_capture.json').read_text())
        marker['exit_receipt']['normal_stop'] = False
        (directory / 'closed_capture.json').write_text(json.dumps(marker))
    rows = recovery_candidates(root)
    assert len(rows) == 1 and not rows[0]['closed']


def test_active_spool_cannot_be_closed_even_with_normal_exit_claim(closed_capture):
    import fcntl
    root, directory, record, exit_receipt = closed_capture
    with (directory / 'active.lock').open('rb') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ValueError, match='仍在运行'):
            mark_capture_closed(record, exit_receipt, root)
        assert not (directory / 'closed_capture.json').exists()


def test_replaced_lock_file_cannot_authorize_marker(closed_capture, monkeypatch):
    from nav_app.features import map_recovery
    root, directory, record, exit_receipt = closed_capture
    original = map_recovery.os.open
    replaced = False
    def open_with_replacement(path, *args, **kwargs):
        nonlocal replaced
        if Path(path) == directory / 'active.lock' and not replaced:
            replacement = directory / 'new.lock'
            replacement.touch()
            replacement.replace(path)
            replaced = True
        return original(path, *args, **kwargs)
    monkeypatch.setattr(map_recovery.os, 'open', open_with_replacement)
    with pytest.raises(ValueError, match='锁文件已变化'):
        mark_capture_closed(record, exit_receipt, root)
    assert not (directory / 'closed_capture.json').exists()


def test_late_old_record_only_closes_its_own_spool(closed_capture, tmp_path):
    import fcntl
    root, old, record, exit_receipt = closed_capture
    _, current, _, _ = native_capture(tmp_path, name='current')
    with (current / 'active.lock').open('rb') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        mark_capture_closed(copy.deepcopy(record), exit_receipt, root)
        assert not (current / 'closed_capture.json').exists()
    rows = recovery_candidates(root)
    assert [row['path'] for row in rows] == [str(current)]
    assert (old / 'closed_capture.json').is_file()


@pytest.mark.parametrize('failure', ['rename', 'directory_sync'])
def test_failed_marker_publication_leaves_tail_recoverable(closed_capture, monkeypatch, failure):
    from nav_app.features import map_recovery
    root, directory, record, exit_receipt = closed_capture
    def fail(*args):
        raise OSError('disk error')
    monkeypatch.setattr(map_recovery.os if failure == 'rename' else map_recovery,
                        'replace' if failure == 'rename' else '_sync_directory', fail)
    with pytest.raises(OSError, match='disk error'):
        mark_capture_closed(record, exit_receipt, root)
    assert not (directory / 'closed_capture.json').exists()
    assert not list(directory.glob('.closed-capture-*.tmp'))
    assert len(recovery_candidates(root)) == 1


def test_marking_never_reads_large_pcd_or_chunk_payload(closed_capture, monkeypatch):
    root, directory, record, exit_receipt = closed_capture
    original = Path.open
    class HeaderOnly:
        def __init__(self, stream): self.stream = stream
        def __enter__(self): return self
        def __exit__(self, *args): self.stream.close()
        def readline(self, length):
            assert 0 < length <= 1025
            return self.stream.readline(length)
        def fileno(self): return self.stream.fileno()
        def tell(self): return self.stream.tell()
    def guarded(path, *args, **kwargs):
        assert not path.name.endswith('.bin'), 'raw chunk payload must not be read'
        stream = original(path, *args, **kwargs)
        return HeaderOnly(stream) if path.suffix == '.pcd' else stream
    monkeypatch.setattr(Path, 'open', guarded)
    mark_capture_closed(record, exit_receipt, root)
    assert not recovery_candidates(root)
    assert len(list(directory.glob('points-*.bin'))) == 3


def test_closed_marker_cannot_claim_another_root(closed_capture, tmp_path):
    _, directory, record, exit_receipt = closed_capture
    with pytest.raises(ValueError, match='本机采集检查点'):
        mark_capture_closed(record, exit_receipt, tmp_path / 'other-root')
    assert not (directory / 'closed_capture.json').exists()


@pytest.mark.parametrize('outcome', ['success', 'failure', 'wrong_receipt', 'missing_receipt', 'incomplete_output'])
def test_explicit_recovery_supersedes_only_confirmed_old_closed_marker(
        closed_capture, tmp_path, monkeypatch, outcome):
    from types import SimpleNamespace
    from nav_app.features import map_recovery
    root, directory, record, exit_receipt = closed_capture
    mark_capture_closed(record, exit_receipt, root)
    marker = directory / 'closed_capture.json'
    original_marker = marker.read_bytes()
    source_pcd = Path(record['map']).read_bytes()
    Path(record['map']).unlink()  # The old committed target is no longer usable.
    raw = {p.name: p.read_bytes() for p in directory.glob('points-*.bin')}
    old_archive = directory / 'closed_capture.recovered-previous.json'
    old_archive.write_text('previous audit remains')
    assert len(recovery_candidates(root)) == 1
    def native_result(command, **kwargs):
        output = Path(command[-1])
        if outcome == 'failure':
            return SimpleNamespace(returncode=1, stderr='native failure', stdout='')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(source_pcd[:-1] if outcome == 'incomplete_output' else source_pcd)
        if outcome != 'missing_receipt':
            (directory / 'exported.txt').write_text(
                str(output) if outcome != 'wrong_receipt' else record['map'])
        return SimpleNamespace(returncode=0, stderr='', stdout='')
    monkeypatch.setattr(map_recovery, 'ResourceLease', lambda name: nullcontext())
    monkeypatch.setattr(map_recovery.subprocess, 'run', native_result)
    if outcome in ('failure', 'incomplete_output'):
        with pytest.raises(RuntimeError):
            recover_checkpoint(directory, tmp_path / 'recovered', executable='recover_map', root=root)
    else:
        result = recover_checkpoint(directory, tmp_path / 'recovered', executable='recover_map', root=root)
        assert Path(result).read_bytes() == source_pcd
    archives = [p for p in directory.glob('closed_capture.recovered-*.json') if p != old_archive]
    if outcome == 'success':
        assert not marker.exists()
        assert len(archives) == 1 and archives[0].read_bytes() == original_marker
        assert not recovery_candidates(root)
    else:
        assert marker.read_bytes() == original_marker and not archives
        assert len(recovery_candidates(root)) == 1
    assert old_archive.read_text() == 'previous audit remains'
    assert {p.name: p.read_bytes() for p in directory.glob('points-*.bin')} == raw

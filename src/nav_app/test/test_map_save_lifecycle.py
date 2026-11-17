from concurrent.futures import Future
from types import SimpleNamespace
import json
import pytest
from nav_app.features.map_save import MapSaveLifecycle


class Saver(MapSaveLifecycle):
    def __init__(self):
        self.init_save_lifecycle()
        self.save_in_progress = False
        self.save_revision = 0
        self.last_saved_path = None

    def begin(self, path, **kwargs):
        assert not self.save_in_progress, 'Previous save blocks the next capture'
        attempt = self.begin_save(path, **kwargs)
        future = Future()
        self.watch_save(future, path, attempt)
        return future


def test_mapper_death_releases_save_and_next_capture_can_save(tmp_path):
    saver = Saver()
    old = saver.begin(tmp_path/'old.pcd')
    saver.cancel_pending_save('建图进程已停止')
    assert old.cancelled() and saver.save_revision == 1
    assert saver.last_saved_path is None
    path = tmp_path/'new.pcd'
    new = saver.begin(path)
    path.write_bytes(b'new map fixture')
    new.set_result(SimpleNamespace(success=True))
    assert saver.last_saved_path == path and not saver.save_in_progress
    assert saver.save_revision == 2


def test_late_old_response_cannot_consume_new_save(tmp_path):
    saver = Saver()
    old = saver.begin(tmp_path/'old.pcd'); old.set_running_or_notify_cancel()
    saver.cancel_pending_save('建图进程已停止')
    path = tmp_path/'new.pcd'
    new = saver.begin(path)
    path.write_bytes(b'new map fixture')
    old.set_result(SimpleNamespace(success=True))
    assert saver.save_in_progress and saver._pending_save_path == path
    assert saver.last_saved_path is None and saver.save_revision == 1
    new.set_result(SimpleNamespace(success=True))
    assert saver.last_saved_path == path and saver.save_revision == 2


def test_failed_ack_and_missing_file_allow_retry_without_claiming_saved(tmp_path):
    saver = Saver()
    for index, response in enumerate((SimpleNamespace(success=False, message='disk write failed'), SimpleNamespace(success=True))):
        saver.begin(tmp_path/f'absent-{index}.pcd').set_result(response)
        assert not saver.save_in_progress and saver.last_saved_path is None
    saver.begin(tmp_path/'absent.pcd').set_exception(RuntimeError('transport lost'))
    assert not saver.save_in_progress and 'transport lost' in saver.save_status


def test_mapper_can_stop_while_call_async_is_returning(tmp_path):
    saver = Saver()
    saver.save_in_progress = True
    saver._pending_save_path = old_path = tmp_path/'old.pcd'
    saver.cancel_pending_save('建图进程已停止')
    new_path = tmp_path/'new.pcd'
    new = saver.begin(new_path)
    old = Future()
    saver.watch_save(old, old_path)
    assert old.cancelled() and saver._save_future is new


def native_commit(path, *, sidecar=True, legacy_root=None):
    header = (b'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              b'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\n'
              b'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n')
    path.write_bytes(header + bytes(32))
    from pathlib import Path
    receipt = Path(str(path) + '.commit.json')
    payload = dict(schema_version=1, output_path=str(path), bytes=path.stat().st_size,
                   points=2, source_spool=str(path.parent/'capture-native'), checkpoint_chunks=1,
                   archive_generation=3)
    if sidecar:
        receipt.write_text(json.dumps(payload))
    if legacy_root:
        directory = legacy_root/'capture-native'; directory.mkdir(parents=True)
        (directory/'exported.txt').write_text(str(path)+'\n')
    return receipt, payload


class QueuedCallbackFuture(Future):
    """rclpy marks a future done before its executor dispatches callbacks."""
    def add_done_callback(self, callback):
        self.callback = callback


def test_exit_consumes_done_future_before_queued_ros_callback(tmp_path):
    saver = Saver(); path = tmp_path/'done.pcd'
    attempt = saver.begin_save(path)
    future = QueuedCallbackFuture()
    saver.watch_save(future, path, attempt)
    path.write_bytes(b'committed map')
    future.set_result(SimpleNamespace(success=True))
    assert saver.save_in_progress and saver.last_saved_path is None
    saver.mapping_exited('exited before executor callback')
    assert saver.last_saved_path == path and saver.save_revision == 1
    assert saver.last_save_result['evidence']['source'] == 'service_response'
    future.callback(future)
    saver.mapping_exited('duplicate exit')
    assert saver.save_revision == 1


@pytest.mark.parametrize('legacy', [False, True])
def test_commit_survives_transport_loss_and_late_callback_without_double_completion(tmp_path, legacy):
    saver = Saver(); path = tmp_path/'committed.pcd'; root = tmp_path/'spool'
    future = saver.begin(path, native_commit=True, legacy_root=root)
    future.set_running_or_notify_cancel()
    native_commit(path, sidecar=not legacy, legacy_root=root if legacy else None)
    saver.mapping_exited('transport disappeared')
    assert saver.last_saved_path == path and saver.save_revision == 1
    assert saver.last_save_result['evidence']['source'] == (
        'legacy_export_receipt' if legacy else 'native_commit_receipt')
    assert saver.last_save_result['evidence']['points'] == 2
    future.set_result(SimpleNamespace(success=True))
    assert saver.save_revision == 1 and not saver.save_in_progress


@pytest.mark.parametrize('fault', ['absent_receipt', 'truncated', 'extra_payload', 'wrong_path',
    'wrong_size', 'wrong_points', 'invalid_json', 'oversized_receipt',
    'symlink_output', 'symlink_receipt', 'temporary_only', 'wrong_header', 'dlio'])
def test_exit_never_promotes_unproven_or_incomplete_map(tmp_path, fault):
    saver = Saver(); path = tmp_path/'map.pcd'
    saver.begin(path, native_commit=fault != 'dlio')
    receipt, payload = native_commit(path)
    if fault == 'absent_receipt':receipt.unlink()
    if fault == 'truncated':path.write_bytes(path.read_bytes()[:-1])
    if fault == 'extra_payload':path.write_bytes(path.read_bytes()+b'x')
    if fault == 'wrong_path':payload['output_path'] = str(tmp_path/'other.pcd')
    if fault == 'wrong_size':payload['bytes'] += 1
    if fault == 'wrong_points':payload['points'] += 1
    if fault in ('wrong_path', 'wrong_size', 'wrong_points'):receipt.write_text(json.dumps(payload))
    if fault == 'invalid_json':receipt.write_text('{')
    if fault == 'oversized_receipt':receipt.write_bytes(b' ' * 17000)
    if fault in ('symlink_output', 'symlink_receipt'):
        target = path if fault == 'symlink_output' else receipt
        destination = target.with_suffix('.original')
        target.rename(destination)
        try:target.symlink_to(destination)
        except OSError:pytest.skip('symlink privileges unavailable')
    if fault == 'temporary_only':path.rename(path.with_suffix('.pcd.partial'))
    if fault == 'wrong_header':path.write_bytes(path.read_bytes().replace(b'DATA binary', b'DATA ascii '))
    saver.mapping_exited('no proven commit')
    assert saver.last_saved_path is None and not saver.save_in_progress
    assert saver.last_save_result['success'] is False


@pytest.mark.parametrize('payload', [[], None, 'wrong-type', 1])
def test_invalid_json_shape_cannot_leave_pending_save_stuck(tmp_path, payload):
    saver = Saver(); path = tmp_path/'map.pcd'
    saver.begin(path, native_commit=True)
    receipt, _ = native_commit(path)
    receipt.write_text(json.dumps(payload))
    saver.mapping_exited('invalid native receipt')
    assert not saver.save_in_progress and saver.last_saved_path is None
    assert saver.save_revision == 1


def test_preexisting_output_is_not_a_new_attempt(tmp_path):
    saver = Saver(); path = tmp_path/'old.pcd'
    native_commit(path)
    with pytest.raises(ValueError, match='已存在'):
        saver.begin(path, native_commit=True)
    assert saver.save_attempt_id == 0 and not saver.save_in_progress


def test_old_or_reused_attempt_receipts_are_not_accepted(tmp_path):
    saver = Saver(); path = tmp_path/'old.pcd'; root = tmp_path/'spool'
    native_commit(path, sidecar=False, legacy_root=root)
    path.unlink()
    saver.begin(path, native_commit=True, legacy_root=root)
    native_commit(path, sidecar=False)  # Existing legacy receipt was NOT rewritten.
    saver.mapping_exited('no receipt for this attempt')
    assert saver.last_saved_path is None
    path.unlink()
    with pytest.raises(ValueError, match='已存在'):
        saver.begin(path, native_commit=True)
    new = tmp_path/'new.pcd'
    native_commit(new)
    new.unlink()  # An orphan old sidecar cannot become another attempt's proof.
    with pytest.raises(ValueError, match='已存在'):
        saver.begin(new, native_commit=True)


def test_new_commit_does_not_require_wall_clock_ordering(tmp_path):
    import os
    saver = Saver(); path = tmp_path/'new.pcd'
    saver.begin(path, native_commit=True)
    receipt, _ = native_commit(path)
    os.utime(receipt, ns=(1, 1))  # A clock adjustment is not a failed file commit.
    saver.mapping_exited('response lost')
    assert saver.last_saved_path == path


def test_unreadable_old_receipt_cannot_prove_a_new_commit(tmp_path, monkeypatch):
    from pathlib import Path
    saver = Saver(); path = tmp_path/'new.pcd'; root = tmp_path/'spool'
    directory = root/'capture-old'; directory.mkdir(parents=True)
    old = directory/'exported.txt'; old.write_text(str(path))
    lstat = Path.lstat
    def unavailable(value):
        if value == old:raise PermissionError('old receipt inaccessible')
        return lstat(value)
    monkeypatch.setattr(Path, 'lstat', unavailable)
    saver.begin(path, native_commit=True, legacy_root=root)
    monkeypatch.undo()
    native_commit(path, sidecar=False)
    saver.mapping_exited('no new receipt')
    assert saver.last_saved_path is None and not saver.save_in_progress


def test_late_attempt_call_failure_or_receipt_does_not_override_new_request(tmp_path):
    saver = Saver(); old_path = tmp_path/'old.pcd'
    old_attempt = saver.begin_save(old_path, native_commit=True)
    saver.mapping_exited('old mapper died')
    new_path = tmp_path/'new.pcd'
    current = saver.begin(new_path, native_commit=True)
    native_commit(old_path)
    old_future = Future()
    old_future.set_result(SimpleNamespace(success=True))
    saver.watch_save(old_future, old_path, old_attempt)
    saver.fail_save_call(old_attempt, 'late call_async exception')
    assert saver.save_in_progress and saver._save_future is current
    assert saver.last_saved_path is None and saver.save_revision == 1
    native_commit(new_path)
    saver.mapping_exited('current mapper died')
    assert saver.last_saved_path == new_path and saver.save_revision == 2
    assert saver.last_save_result['attempt_id'] != old_attempt


def test_explicit_discard_keeps_cancel_semantics_even_after_native_commit(tmp_path):
    saver = Saver(); path = tmp_path/'discard.pcd'
    saver.begin(path, native_commit=True)
    native_commit(path)
    saver.cancel_pending_save('user abandoned wait')
    assert saver.last_saved_path is None and path.is_file()
    assert saver.last_save_result['success'] is False


def test_service_error_with_native_commit_preserves_error_evidence(tmp_path):
    saver = Saver(); path = tmp_path/'committed.pcd'
    future = saver.begin(path, native_commit=True)
    native_commit(path)
    future.set_exception(RuntimeError('response transport lost'))
    assert saver.last_saved_path == path and saver.save_revision == 1
    assert 'response transport lost' in saver.last_save_result['evidence']['response_error']


def test_normal_ack_retains_native_snapshot_identity_without_reclassifying_service_response(tmp_path):
    saver = Saver(); path = tmp_path/'normal.pcd'
    future = saver.begin(path, native_commit=True)
    native_commit(path)
    future.set_result(SimpleNamespace(success=True))
    evidence = saver.last_save_result['evidence']
    assert evidence['source'] == 'service_response' and evidence['commit_source'] == 'native_commit_receipt'
    assert evidence['archive_generation'] == 3 and evidence['checkpoint_chunks'] == 1
    assert evidence['source_spool'] == str(tmp_path/'capture-native')

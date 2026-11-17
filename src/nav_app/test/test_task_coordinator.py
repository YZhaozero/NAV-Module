"""Real local worker, cancellation, store recovery, and single-owner checks.

Workers are tiny synthetic processes. No sensors, ROS launch, GPU, or device
connections are started, and all state belongs to pytest's temporary directory.
"""
import json
import os
import signal
import sqlite3
import sys
import threading
import time

import pytest

from nav_app.features.task_coordinator import (
    TaskCoordinator, TaskStore, job_id, normalize_task_payload,
)


@pytest.fixture(autouse=True)
def isolated_recording_receipt(tmp_path, monkeypatch):
    from nav_app.features import task_storage
    monkeypatch.setenv('NAV_RECORD_STATE_DIR', str(tmp_path / 'recording-state'))
    monkeypatch.setattr(task_storage, 'task_storage_reason', lambda *_: '')


def until(predicate, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.025)
    raise AssertionError('Timed out waiting for synthetic task state')


def test_model_compute_lease_defers_processing_and_foreground_without_launch(tmp_path, worker, monkeypatch):
    from nav_app.features.resource_lease import ResourceLease
    monkeypatch.setenv('NAV_RUNTIME_DIR',str(tmp_path/'compute'))
    lease=ResourceLease('model').acquire()
    coordinator=TaskCoordinator(tmp_path/'tasks',command=worker,systemd=False)
    try:
        until(lambda:coordinator.snapshot()['loaded'])
        job=coordinator.submit('probe',dict(tag='after-model',seconds=.1))
        until(lambda:row(coordinator,job))
        assert not coordinator.reserve_idle('capture:A')
        assert coordinator.foreground('capture:A')
        until(lambda:coordinator.snapshot().get('wait_reason'))
        assert not (tmp_path/'tasks'/job/'starts.log').exists()
        assert not coordinator.snapshot()['released']
        lease.close()
        until(lambda:coordinator.snapshot()['released'])
        assert not (tmp_path/'tasks'/job/'starts.log').exists()
        coordinator.foreground(None)
        until(lambda:row(coordinator,job)['state']=='succeeded')
    finally:
        lease.close();coordinator.close();coordinator.thread.join(timeout=9)


def test_bad_job_json_is_local_failure_and_does_not_break_capture_receipt(tmp_path):
    store=TaskStore(tmp_path)
    try:
        identifier='f'*32
        store.submit(identifier,'navigation',dict(capture_id='a'*32))
        with store.db:store.db.execute('UPDATE jobs SET payload=?,progress=?,result=? WHERE id=?',('{broken','[]','[1]',identifier))
        store.capture(dict(capture_id='a'*32,title='kept',map='/original.pcd'))
        assert store.get(identifier)['state']=='failed'
        assert store.snapshot()['captures'][0]['map']=='/original.pcd'
    finally:store.db.close()


def test_selected_navigation_priority_preempts_background_without_losing_it(tmp_path, worker):
    coordinator=TaskCoordinator(tmp_path/'tasks',command=worker,systemd=False)
    try:
        until(lambda:coordinator.snapshot()['loaded'])
        old=coordinator.submit('probe',dict(tag='background-A',seconds=2.),priority=40)
        until(lambda:(tmp_path/'tasks'/old/'starts.log').exists())
        selected=coordinator.submit('probe',dict(tag='selected-B',seconds=.1),priority=40)
        coordinator.promote(selected)
        until(lambda:(value:=row(coordinator,selected)) and value['state']=='succeeded')
        assert row(coordinator,old)['state']!='succeeded'
        until(lambda:row(coordinator,old)['state']=='succeeded')
        assert row(coordinator,old)['attempt']==2
        assert row(coordinator,selected)['attempt']==1
    finally:finish(coordinator)


def row(coordinator, identifier):
    return next((value for value in coordinator.snapshot()['jobs'] if value['id'] == identifier), None)


@pytest.fixture
def worker(tmp_path):
    script = tmp_path / 'synthetic_worker.py'
    script.write_text('''import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--spec',required=True);a,_=p.parse_known_args()
spec=Path(a.spec);root=spec.parent;data=json.loads(spec.read_text())['payload'];stop=False
def publish(name,value):
 temporary=root/(name+'.tmp');temporary.write_text(json.dumps(value));os.replace(temporary,root/name)
def requested(*args):
 global stop
 if data.get('mode')=='fork_exit':raise SystemExit(2)
 stop=True
signal.signal(signal.SIGTERM,requested)
with (root/'starts.log').open('a') as log:log.write(str(os.getpid())+'\\n')
if data.get('mode')=='fork_exit':
 child=subprocess.Popen([sys.executable,'-c',"import os,signal,time;from pathlib import Path;signal.signal(signal.SIGTERM,signal.SIG_IGN);Path("+repr(str(root/'child.pid'))+").write_text(str(os.getpid()));time.sleep(30)"])
 while not (root/'child.pid').exists():time.sleep(.01)
 publish('progress.json',dict(stage='synthetic',done=1,total=2))
 while True:time.sleep(.05)
end=time.monotonic()+data.get('seconds',.1)
while time.monotonic()<end and not stop:time.sleep(.01)
if stop:
 publish('result.json',dict(ok=False,error='cancelled'));raise SystemExit(2)
publish('result.json',dict(ok=True,value=dict(tag=data.get('tag','done'))))
''', encoding='utf-8')
    return [sys.executable, str(script)]


def finish(coordinator):
    coordinator.close()
    coordinator.thread.join(timeout=9)
    assert not coordinator.thread.is_alive()


def test_real_worker_deduplicates_and_retains_result_after_reopen(tmp_path, worker):
    root = tmp_path / 'tasks'
    coordinator = TaskCoordinator(root, command=worker, systemd=False)
    try:
        until(lambda: coordinator.snapshot()['loaded'])
        payload = dict(tag='A', seconds=.15)
        first = coordinator.submit('probe', payload)
        assert coordinator.submit('probe', payload) == first
        final = until(lambda: (value := row(coordinator, first)) and value['state'] == 'succeeded' and value)
        assert final['attempt'] == 1 and final['result'] == {'tag': 'A'}
        assert len((root / first / 'starts.log').read_text().splitlines()) == 1
    finally:
        finish(coordinator)
    store = TaskStore(root)
    try:
        assert store.get(first)['state'] == 'succeeded'
        assert store.get(first)['result'] == {'tag': 'A'}
    finally:
        store.db.close()


def test_foreground_cancels_then_resumes_same_job_after_release(tmp_path, worker):
    coordinator = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    try:
        until(lambda: coordinator.snapshot()['loaded'])
        job = coordinator.submit('probe', dict(tag='A', seconds=.6))
        until(lambda: (tmp_path / 'tasks' / job / 'starts.log').exists())
        assert coordinator.foreground('capture:A')
        assert coordinator.snapshot()['released'] is False
        held = until(lambda: coordinator.snapshot()['released'] and coordinator.snapshot())
        assert held['foreground'] == 'capture:A'
        assert row(coordinator, job)['state'] == 'waiting_foreground'
        assert not coordinator.foreground('capture:B')
        assert coordinator.foreground(None)
        done = until(lambda: (value := row(coordinator, job)) and value['state'] == 'succeeded' and value)
        assert done['attempt'] == 2 and done['result'] == {'tag': 'A'}
    finally:
        finish(coordinator)


def test_user_cancelled_queued_job_does_not_restart(tmp_path, worker):
    coordinator = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    try:
        until(lambda: coordinator.snapshot()['loaded'])
        assert coordinator.foreground('capture:A')
        job = coordinator.submit('probe', dict(tag='not-started'))
        coordinator.cancel(job)
        until(lambda: (value := row(coordinator, job)) and value['state'] == 'cancelled')
        assert coordinator.foreground(None)
        time.sleep(.3)
        assert row(coordinator, job)['state'] == 'cancelled'
        assert not (tmp_path / 'tasks' / job / 'starts.log').exists()
    finally:
        finish(coordinator)


def test_store_recovery_queues_interrupted_jobs_but_keeps_cancelled(tmp_path):
    store = TaskStore(tmp_path)
    for n, state in enumerate(('running', 'cancelling', 'waiting_foreground', 'cancelled', 'succeeded')):
        identifier = str(n) * 32
        store.submit(identifier, 'probe', {})
        store.update(identifier, state=state, attempt=2)
    store.db.close()
    store = TaskStore(tmp_path)
    try:
        store.recover()
        assert [store.get(str(n) * 32)['state'] for n in range(5)] == ['queued', 'queued', 'queued', 'cancelled', 'succeeded']
        assert all(store.get(str(n) * 32)['attempt'] == 2 for n in range(5))
    finally:
        store.db.close()


def test_old_navigation_completion_cannot_replace_capture_current_job(tmp_path):
    store = TaskStore(tmp_path)
    capture = 'a' * 32; old = '1' * 32; current = '2' * 32
    try:
        store.submit(old, 'navigation', dict(capture_id=capture, cloud='old.pcd'))
        store.submit(current, 'navigation', dict(capture_id=capture, cloud='new.pcd'))
        store.capture(dict(capture_id=capture, map='new.pcd', navigation_job_id=current))
        store.update(old, state='succeeded', result={'scene': 'old.scene.json'})
        store.complete_capture_job(store.get(old))
        value = store.snapshot()['captures'][0]
        assert value['navigation_job_id'] == current
        assert value.get('navigation_scene') != 'old.scene.json'
        assert value['map'] == 'new.pcd'
    finally:
        store.db.close()


@pytest.mark.skipif(os.name != 'posix', reason='Linux flock and process groups')
def test_second_coordinator_cannot_own_same_store(tmp_path, worker):
    first = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    second = None
    try:
        until(lambda: first.snapshot()['loaded'])
        second = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
        until(lambda: second.snapshot()['error'])
        assert second.snapshot()['released'] is False
        assert first.reserve_idle('capture:A')
    finally:
        if second is not None: finish(second)
        finish(first)


@pytest.mark.skipif(os.name != 'posix', reason='Linux process groups')
def test_worker_exit_does_not_release_live_descendant(tmp_path, worker):
    coordinator = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    child_pid = None
    try:
        until(lambda: coordinator.snapshot()['loaded'])
        job = coordinator.submit('probe', dict(mode='fork_exit'))
        child_path = tmp_path / 'tasks' / job / 'child.pid'
        until(child_path.exists)
        child_pid = int(child_path.read_text())
        assert coordinator.foreground('capture:A')
        until(lambda: row(coordinator, job)['state'] in ('cancelling','waiting_foreground'))
        # The synthetic worker exits promptly but the child deliberately ignores
        # TERM. The coordinator must hold ownership until group cleanup finishes.
        time.sleep(.2)
        try:
            os.kill(child_pid, 0)
        except ProcessLookupError:
            pass
        else:
            assert not coordinator.snapshot()['released']
        until(lambda: coordinator.snapshot()['released'], timeout=9)
    finally:
        if child_pid is not None:
            try: os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError: pass
        finish(coordinator)


def test_invalid_receipts_do_not_block_valid_capture_recovery(tmp_path, worker):
    root = tmp_path / 'tasks'
    captures = root / 'captures'; captures.mkdir(parents=True)
    requests = root / 'requests'; requests.mkdir()
    invalid = ['{truncated', 'null', '[]', '{"capture_id":null}', '{"capture_id":"invalid"}']
    for n, text in enumerate(invalid):
        (captures / f'bad-{n}.json').write_text(text)
        (requests / f'bad-{n}.json').write_text(text)
    bad_id = 'd' * 32
    (requests / (bad_id + '.json')).write_text(json.dumps(dict(id=bad_id, kind=None, payload={})))
    bad_payload = 'e' * 32
    (requests / (bad_payload + '.json')).write_text(json.dumps(dict(id=bad_payload, kind='probe', payload=[])))
    capture = 'a' * 32
    (captures / (capture + '.json')).write_text(json.dumps(dict(capture_id=capture, map='preserved.pcd', finished=True)))
    coordinator = TaskCoordinator(root, command=worker, systemd=False)
    try:
        coordinator.foreground('capture:hold')
        view = until(lambda: coordinator.snapshot()['loaded'] and coordinator.snapshot())
        assert not view['error']
        assert len(view['captures']) == 1 and view['captures'][0]['capture_id'] == capture
        assert not view['jobs']
        assert all((captures / f'bad-{n}.json').exists() for n in range(len(invalid)))
    finally:
        finish(coordinator)


def test_corrupt_sqlite_is_preserved_and_rebuilt_from_capture_and_request_receipts(tmp_path, worker):
    root = tmp_path / 'tasks'
    store = TaskStore(root)
    capture, pending, cancelled, completed = 'a' * 32, 'b' * 32, 'c' * 32, 'f' * 32
    source = tmp_path / 'recorded-map.pcd'; source.write_bytes(b'original measured data')
    store.capture(dict(capture_id=capture, map=str(source), finished=True, bag_stored=True))
    store.submit(pending, 'probe', dict(tag='pending', seconds=.1))
    store.submit(cancelled, 'probe', dict(tag='never-restart'))
    store.update(cancelled, state='cancelled', error='user cancelled')
    store.submit(completed, 'probe', dict(tag='already-completed'))
    store.update(completed, state='succeeded', result={'tag': 'keep result'})
    store.db.close()
    corrupt = b'not a database\x00preserve this failed original database'
    (root / 'tasks.sqlite3').write_bytes(corrupt)
    coordinator = TaskCoordinator(root, command=worker, systemd=False)
    try:
        coordinator.foreground('capture:hold')
        view = until(lambda: coordinator.snapshot()['loaded'] and coordinator.snapshot())
        assert not view['error']
        isolated = list(root.glob('tasks.sqlite3.corrupt-*'))
        assert len(isolated) == 1 and isolated[0].read_bytes() == corrupt
        assert view['captures'][0]['capture_id'] == capture
        assert view['captures'][0]['map'] == str(source)
        assert source.read_bytes() == b'original measured data'
        assert row(coordinator, pending)['state'] == 'queued'
        assert row(coordinator, cancelled)['state'] == 'cancelled'
        assert row(coordinator, completed)['state'] == 'succeeded'
        assert row(coordinator, completed)['result'] == {'tag': 'keep result'}
        coordinator.foreground(None)
        done = until(lambda: (value := row(coordinator, pending)) and value['state'] == 'succeeded' and value)
        assert done['result'] == {'tag': 'pending'}
        assert row(coordinator, cancelled)['state'] == 'cancelled'
        assert not (root / cancelled / 'starts.log').exists()
        assert not (root / completed / 'starts.log').exists()
    finally:
        finish(coordinator)


def test_higher_schema_is_rejected_without_downgrade_or_quarantine(tmp_path, worker):
    root = tmp_path / 'tasks'; root.mkdir()
    connection = sqlite3.connect(root / 'tasks.sqlite3')
    connection.executescript("PRAGMA user_version=9; CREATE TABLE future_data(value TEXT); INSERT INTO future_data VALUES('keep me');")
    connection.close()
    with pytest.raises(ValueError, match='更新'):
        TaskStore(root)
    coordinator = TaskCoordinator(root, command=worker, systemd=False)
    try:
        view = until(lambda: coordinator.snapshot()['error'] and coordinator.snapshot())
        assert '更新' in view['error'] and not view['released']
    finally:
        finish(coordinator)
    connection = sqlite3.connect(root / 'tasks.sqlite3')
    try:
        assert connection.execute('PRAGMA user_version').fetchone()[0] == 9
        assert connection.execute('SELECT value FROM future_data').fetchone()[0] == 'keep me'
        assert not list(root.glob('*.corrupt-*'))
    finally:
        connection.close()


def test_close_does_not_block_when_command_queue_is_full(tmp_path, worker, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = TaskStore.__init__
    def delayed_init(self, directory):
        entered.set()
        assert release.wait(5)
        original(self, directory)
    monkeypatch.setattr(TaskStore, '__init__', delayed_init)
    coordinator = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    closer = None
    try:
        assert entered.wait(3)
        for _ in range(coordinator.commands.maxsize):
            coordinator.commands.put_nowait(('foreground', None))
        assert coordinator.commands.full()
        closer = threading.Thread(target=coordinator.close, daemon=True)
        closer.start(); closer.join(timeout=.3)
        assert not closer.is_alive(), 'close() blocked the caller on a full queue'
        assert coordinator.closed
    finally:
        release.set()
        if closer is not None: closer.join(timeout=3)
        coordinator.close(); coordinator.thread.join(timeout=5)
        assert not coordinator.thread.is_alive()


def test_shutdown_between_saved_capture_and_timer_repairs_missing_jobs(tmp_path):
    root = tmp_path / 'tasks'
    cloud = tmp_path / 'saved.pcd'; cloud.write_bytes(b'original measured cloud')
    bag = tmp_path / 'recording'; bag.mkdir()
    manifest = dict(state='stored', storage_flush_confirmed=True)
    (bag / 'nav_manifest.json').write_text(json.dumps(manifest))
    capture = 'a' * 32
    audit_payload = dict(directory=str(bag), capture_id=capture, title='本次录制')
    value = dict(capture_id=capture, finished=True, map=str(cloud), title='本次地图',
                 navigation_directory=str(tmp_path / 'maps'), gravity_aligned=True,
                 bag=str(bag), audit_request=audit_payload)
    store = TaskStore(root)
    store.capture(value)  # Durable raw-save acknowledgement; GUI timer has not run.
    store.db.close()
    store = TaskStore(root)
    try:
        assert not store.snapshot()['jobs']
        store.reconcile_capture_tasks(last_capture_path=False)
        jobs = store.snapshot()['jobs']
        assert sorted(row['kind'] for row in jobs) == ['audit', 'navigation']
        navigation = next(row for row in jobs if row['kind'] == 'navigation')
        expected = dict(cloud=str(cloud), directory=str(tmp_path / 'maps'), title='本次地图',
                        capture_id=capture, gravity_aligned=True)
        expected = normalize_task_payload('navigation', expected)
        assert navigation['payload'] == expected
        assert navigation['id'] == job_id('navigation', expected)
        audit = next(row for row in jobs if row['kind'] == 'audit')
        assert audit['id'] == job_id('audit', normalize_task_payload('audit', audit_payload))
        restored = store.snapshot()['captures'][0]
        assert restored['navigation_job_id'] == navigation['id']
        assert restored['audit_job_id'] == audit['id']
        assert restored['bag_stored'] and restored['storage_state'] == 'stored'
        assert not restored['bag_complete'] and restored['audit_state'] == 'queued'
        assert cloud.read_bytes() == b'original measured cloud'
        assert json.loads((bag / 'nav_manifest.json').read_text()) == manifest
        store.reconcile_capture_tasks(last_capture_path=False)
        assert len(store.snapshot()['jobs']) == 2
    finally:
        store.db.close()


def test_reconcile_preserves_paused_failed_ready_and_cancelled_outcomes(tmp_path):
    store = TaskStore(tmp_path / 'tasks')
    cloud = tmp_path / 'saved.pcd'; cloud.write_bytes(b'cloud')
    try:
        for number, phase in enumerate(('ready', 'failed', 'deferred', 'paused', 'cancelled')):
            capture = str(number) * 32
            store.capture(dict(capture_id=capture, finished=True, map=str(cloud),
                navigation_directory=str(tmp_path / 'maps'), navigation_state=phase,
                navigation_scene='already-ready.scene.json' if phase == 'ready' else ''))
        capture = 'a' * 32
        payload = normalize_task_payload('navigation', dict(cloud=str(cloud), directory=str(tmp_path / 'maps'),
            title='已取消地图', capture_id=capture, gravity_aligned=False))
        identifier = job_id('navigation', payload)
        store.submit(identifier, 'navigation', payload)
        store.update(identifier, state='cancelled')
        store.capture(dict(capture_id=capture, finished=True, map=str(cloud),
            navigation_directory=str(tmp_path / 'maps'), navigation_job_id=identifier))
        store.reconcile_capture_tasks(last_capture_path=False)
        assert len(store.snapshot()['jobs']) == 1
        assert store.get(identifier)['state'] == 'cancelled'
        values = {row['capture_id']: row for row in store.snapshot()['captures']}
        assert [values[str(i) * 32]['navigation_state'] for i in range(5)] == ['ready', 'failed', 'deferred', 'paused', 'cancelled']
        assert values[capture]['navigation_state'] == 'deferred'
    finally:
        store.db.close()


def test_only_explicit_latest_stored_recording_gets_missing_audit(tmp_path, monkeypatch):
    state_dir = tmp_path / 'recording-state'; state_dir.mkdir()
    bag = tmp_path / 'latest-recording'; bag.mkdir()
    other = tmp_path / 'unrelated-old-bag'; other.mkdir()
    (other / 'nav_manifest.json').write_text(json.dumps(dict(state='stored', storage_flush_confirmed=True)))
    manifest = bag / 'nav_manifest.json'
    manifest.write_text(json.dumps(dict(state='stored', storage_flush_confirmed=False)))
    (state_dir / 'last_capture.json').write_text(json.dumps(dict(path=str(bag), state='stored')))
    store = TaskStore(tmp_path / 'tasks')
    try:
        store.reconcile_capture_tasks()
        assert not store.snapshot()['jobs']  # A state label alone is not a flush proof.
        manifest.write_text(json.dumps(dict(state='stored', storage_flush_confirmed=True)))
        store.reconcile_capture_tasks()
        jobs = store.snapshot()['jobs']
        assert len(jobs) == 1 and jobs[0]['kind'] == 'audit'
        assert jobs[0]['payload'] == dict(directory=str(bag.resolve()), capture_id=None, title=bag.name)
        store.update(jobs[0]['id'], state='cancelled')
        store.reconcile_capture_tasks()
        assert len(store.snapshot()['jobs']) == 1 and store.get(jobs[0]['id'])['state'] == 'cancelled'
        assert not list(other.glob('nav_audit.json'))
    finally:
        store.db.close()


def test_large_queue_cannot_hide_priority_navigation_completion_or_failure(tmp_path):
    store = TaskStore(tmp_path / 'tasks')
    completed, failed, capture = 'f' * 32, 'e' * 32, 'a' * 32
    try:
        store.submit(completed, 'navigation', dict(capture_id=capture, cloud='measured.pcd'), priority=0)
        store.capture(dict(capture_id=capture, finished=True, map='measured.pcd', navigation_job_id=completed))
        store.submit(failed, 'probe', dict(tag='failed-selected-map'), priority=0)
        for number in range(1, 221):
            store.submit(f'{number:032x}', 'probe', dict(tag=f'waiting-{number}'))
        store.update(completed, state='succeeded', result={'scene': 'ready.scene.json'})
        store.complete_capture_job(store.get(completed))
        store.update(failed, state='failed', error='索引输入缺失')
        view = store.snapshot()
        visible = {job['id']: job for job in view['jobs']}
        assert visible[completed]['state'] == 'succeeded'
        assert visible[completed]['result']['scene'] == 'ready.scene.json'
        assert visible[failed]['state'] == 'failed' and visible[failed]['error'] == '索引输入缺失'
        assert view['captures'][0]['navigation_state'] == 'ready'
        assert 200 <= len(view['jobs']) <= 400
        assert store.db.execute('SELECT count(*) FROM jobs').fetchone()[0] == 222
    finally:
        store.db.close()


def test_deleted_grid_invalidates_only_its_maps_and_preserves_shared_cloud(tmp_path):
    store = TaskStore(tmp_path / 'tasks')
    cloud = tmp_path / 'measured.pcd'; cloud.write_bytes(b'original geometry')
    grid_a = tmp_path / 'a.yaml'; grid_a.write_text('image: a.pgm')
    grid_b = tmp_path / 'b.yaml'; grid_b.write_text('image: b.pgm')
    scene_a = tmp_path / 'a.scene.json'; scene_a.write_text(json.dumps(dict(cloud=cloud.name, grid=grid_a.name)))
    scene_b = tmp_path / 'b.scene.json'; scene_b.write_text(json.dumps(dict(cloud=cloud.name, grid=grid_b.name)))
    current, unrelated, capture = 'a' * 32, 'b' * 32, 'c' * 32
    payload = dict(cloud=str(cloud), capture_id=capture)
    try:
        store.submit(current, 'navigation', payload)
        store.update(current, state='succeeded', result=dict(scene=str(scene_a)))
        store.capture(dict(capture_id=capture, finished=True, map=str(cloud), navigation_job_id=current))
        store.submit(unrelated, 'navigation', dict(cloud=str(cloud)))
        store.update(unrelated, state='succeeded', result=dict(scene=str(scene_b)))
        grid_a.unlink()
        assert store.invalidate_scene(str(scene_a), [str(grid_a)]) == 1
        assert store.get(current)['state'] == 'failed' and store.get(current)['result'] is None
        assert store.get(unrelated)['state'] == 'succeeded'
        value = store.snapshot()['captures'][0]
        assert value['navigation_state'] == 'failed' and not value['navigation_scene']
        assert '已删除' in value['navigation_error']
        assert cloud.read_bytes() == b'original geometry'
        assert scene_b.exists() and grid_b.exists()
        assert store.invalidate_scene(str(scene_a), [str(grid_a)]) == 0
        store.submit(current, 'navigation', payload, retry=True)
        assert store.get(current)['state'] == 'queued'
    finally:
        store.db.close()


def test_low_storage_keeps_job_queued_then_starts_after_space_recovers(tmp_path, worker, monkeypatch):
    from nav_app.features import task_storage
    low = [True]; checks = []
    def reason(kind, payload, work_directory):
        checks.append((kind, dict(payload), str(work_directory)))
        return '地图输出空间不足，需要保留采集收尾余量' if low[0] else ''
    monkeypatch.setattr(task_storage, 'task_storage_reason', reason)
    coordinator = TaskCoordinator(tmp_path / 'tasks', command=worker, systemd=False)
    try:
        until(lambda: coordinator.snapshot()['loaded'])
        identifier = coordinator.submit('probe', dict(tag='space-restored'))
        current = until(lambda: (value := row(coordinator, identifier))
                        and value['progress'].get('stage') == 'storage' and value)
        assert current['state'] == 'queued' and current['attempt'] == 0
        assert '空间不足' in current['error']
        assert '空间不足' in coordinator.snapshot()['wait_reason']
        assert not (tmp_path / 'tasks' / identifier / 'starts.log').exists()
        assert len(checks) == 1  # The 100 ms owner loop does not repeatedly stat disks.
        assert checks[0] == ('probe', {'tag': 'space-restored'}, str(tmp_path / 'tasks' / identifier / 'work'))
        assert coordinator.reserve_idle('capture:next')
        until(lambda: coordinator.snapshot()['released'])
        assert not (tmp_path / 'tasks' / identifier / 'starts.log').exists()
        low[0] = False
        coordinator.foreground(None)
        final = until(lambda: (value := row(coordinator, identifier)) and value['state'] == 'succeeded' and value,
                      timeout=9)
        assert final['attempt'] == 1 and final['result'] == {'tag': 'space-restored'}
        assert not final['error'] and not coordinator.snapshot()['wait_reason']
        assert len(checks) == 2
    finally:
        finish(coordinator)

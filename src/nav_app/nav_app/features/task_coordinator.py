"""Durable local jobs with one heavy process and explicit foreground ownership.

SQLite, process management and progress reads run on the coordinator thread.
GUI callers only enqueue small commands and read bounded copied snapshots.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import sqlite3
import subprocess
import sys
import threading
import time
import uuid


TERMINAL = frozenset(('succeeded', 'failed', 'cancelled'))


def source_revision(path):
    """Cheap immutable-input revision; hashing the point cloud stays in workers."""
    try:
        stamp = Path(path).stat()
        return [stamp.st_dev, stamp.st_ino, stamp.st_size, stamp.st_mtime_ns]
    except OSError:
        return None


def normalize_task_payload(kind, payload):
    value = copy.deepcopy(payload)
    if not isinstance(value, dict):
        raise ValueError('地图任务输入格式无效')
    if kind in ('navigation', 'scan_context'):
        source = Path(value['cloud']).expanduser().resolve()
        value['cloud'] = str(source)
        value['source_revision'] = source_revision(source)
        if kind == 'navigation':
            value['grid_method'] = value.get('grid_method', 'observed')
            if value['grid_method'] not in ('observed', 'terrain'):
                raise ValueError('不支持的栅格生成方法')
            value['directory'] = str(Path(value['directory']).expanduser().resolve())
            if value.get('preserve_scene'):
                scene = Path(value['preserve_scene']).expanduser().resolve()
                value['preserve_scene'] = str(scene)
                value['preserve_scene_revision'] = source_revision(scene)
        # A navigation request must regenerate the v2 sampling-aware grid even
        # when an older successful job for the same cloud already exists.
        value['task_schema'] = 2 if kind == 'navigation' else 1
    return value


def job_id(kind, normalized_payload):
    """Identify an already normalized request without another filesystem read."""
    encoded = json.dumps(dict(kind=kind, payload=normalized_payload), sort_keys=True,
                         ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()[:32]


def _json_object(path):
    try:
        path = Path(path)
        if path.stat().st_size > 2 * 1024**2:
            return None
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError, TypeError):
        return None


def _stored_recording(directory):
    value = _json_object(Path(directory) / 'nav_manifest.json')
    return bool(value and value.get('state') == 'stored'
                and value.get('storage_flush_confirmed') is True)


class TaskStore:
    def __init__(self, directory):
        self.root = Path(directory).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.root / 'tasks.sqlite3')
        try:
            self.db.row_factory = sqlite3.Row
            version=self.db.execute('PRAGMA user_version').fetchone()[0]
            if version>1:
                self.db.close();raise ValueError('地图任务记录来自更新的软件版本，请使用对应版本打开')
            if self.db.execute('PRAGMA quick_check').fetchone()[0]!='ok':
                self.db.close();raise sqlite3.DatabaseError('database integrity check failed')
            self.db.execute('PRAGMA journal_mode=WAL')
            self.db.execute('PRAGMA synchronous=FULL')
            self.db.executescript('''
              CREATE TABLE IF NOT EXISTS jobs(
                id TEXT PRIMARY KEY, kind TEXT NOT NULL, payload TEXT NOT NULL,
                state TEXT NOT NULL, priority INTEGER NOT NULL, attempt INTEGER NOT NULL DEFAULT 0,
                progress TEXT NOT NULL DEFAULT '{}', result TEXT, error TEXT NOT NULL DEFAULT '',
                created REAL NOT NULL, updated REAL NOT NULL);
              CREATE TABLE IF NOT EXISTS captures(id TEXT PRIMARY KEY, value TEXT NOT NULL, updated REAL NOT NULL);
              PRAGMA user_version=1;
            ''')
            # Workers belong to this coordinator. A previous owner is reaped before
            # recover() is called, never adopt an arbitrary PID as a current worker.
        except Exception:
            self.db.close()
            raise

    def recover(self):
        with self.db:
            self.db.execute("UPDATE jobs SET state='queued',error='上次处理已中断，等待继续',updated=? "
                            "WHERE state IN ('running','cancelling','waiting_foreground')", (time.time(),))

    def submit(self, identifier, kind, payload, priority=50, retry=False):
        now = time.time()
        from .capture_quality import atomic_json
        requests=self.root/'requests';requests.mkdir(exist_ok=True)
        if not self.get(identifier):
            atomic_json(requests/(identifier+'.json'),dict(id=identifier,kind=kind,payload=payload,priority=priority,state='queued'))
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO jobs(id,kind,payload,state,priority,created,updated) VALUES(?,?,?,?,?,?,?)',
                            (identifier, kind, json.dumps(payload, ensure_ascii=False), 'queued', priority, now, now))
            if retry:
                self.db.execute("UPDATE jobs SET state='queued',error='',updated=? WHERE id=? AND state IN ('failed','cancelled')",
                                (now, identifier))
            self.db.execute('UPDATE jobs SET priority=min(priority,?) WHERE id=?', (priority, identifier))

    def update(self, identifier, **values):
        allowed = {'state','progress','result','error','attempt','priority'}
        assert set(values) <= allowed
        values = dict(values, updated=time.time())
        for key in ('progress','result'):
            if key in values: values[key] = json.dumps(values[key], ensure_ascii=False)
        with self.db:
            self.db.execute('UPDATE jobs SET '+','.join(key+'=?' for key in values)+' WHERE id=?',
                            (*values.values(), identifier))
        if 'state' in values:
            from .capture_quality import atomic_json
            requests=self.root/'requests';requests.mkdir(exist_ok=True)
            atomic_json(requests/(identifier+'.json'),self.get(identifier))

    def capture(self, value):
        identifier = value.get('capture_id')
        if not identifier: return
        value = dict(value)
        for row in self.db.execute("SELECT * FROM jobs WHERE CASE WHEN json_valid(payload) THEN json_extract(payload,'$.capture_id') END=? ORDER BY created", (identifier,)):
            job = self.row(row)
            if job['kind']=='navigation' and value.get('navigation_job_id') not in (None,job['id']):continue
            value.update(capture_job_update(job))
        from .capture_quality import atomic_json
        receipt = self.root/'captures'
        receipt.mkdir(exist_ok=True)
        if len(identifier)!=32 or any(c not in '0123456789abcdef' for c in identifier):
            raise ValueError('采集记录编号无效')
        atomic_json(receipt/(identifier+'.json'), value)
        with self.db:
            self.db.execute('INSERT OR REPLACE INTO captures VALUES(?,?,?)',
                            (identifier, json.dumps(value, ensure_ascii=False), time.time()))

    @staticmethod
    def row(row):
        value = dict(row)
        for key in ('payload','progress','result'):
            try:value[key] = json.loads(value[key]) if value[key] else None
            except (TypeError,ValueError):value[key]=None
        if not isinstance(value['payload'],dict):
            value.update(payload={},state='failed',error='任务输入记录不完整，请重新选择原始数据')
        if not isinstance(value['progress'],dict):value['progress']={}
        if not isinstance(value['result'],dict):value['result']=None
        if value['state']=='succeeded' and value['kind']=='navigation' and not (value['result'] or {}).get('scene'):
            value.update(state='failed',error='地图处理结果记录不完整，请重试')
        return value

    def get(self, identifier):
        row = self.db.execute('SELECT * FROM jobs WHERE id=?', (identifier,)).fetchone()
        return self.row(row) if row else None

    def next(self):
        row = self.db.execute("SELECT * FROM jobs WHERE state='queued' ORDER BY priority,created LIMIT 1").fetchone()
        return self.row(row) if row else None

    def snapshot(self):
        # A large backlog must not hide the just-completed priority navigation
        # job from its GUI callback. Bound both portions independently instead
        # of placing every queued row ahead of all terminal acknowledgements.
        active_states = "('queued','running','cancelling','waiting_foreground')"
        rows = list(self.db.execute('SELECT * FROM jobs WHERE state IN '+active_states+
                                    ' ORDER BY updated DESC LIMIT 200'))
        rows.extend(self.db.execute('SELECT * FROM jobs WHERE state NOT IN '+active_states+
                                    ' ORDER BY updated DESC LIMIT 100'))
        jobs = [self.row(r) for r in rows]
        for job in jobs:
            if job['kind']=='audit' and isinstance(job.get('result'),dict):
                full=job['result']
                job['result']={key:full.get(key) for key in ('state','errors','warnings')}
        captures = [json.loads(r[0]) for r in self.db.execute('SELECT value FROM captures ORDER BY updated DESC LIMIT 100')]
        return dict(jobs=jobs, captures=captures)

    def complete_capture_job(self, job):
        identifier = job['payload'].get('capture_id')
        row = self.db.execute('SELECT value FROM captures WHERE id=?', (identifier,)).fetchone()
        if not row: return
        value = json.loads(row[0])
        if job['kind']=='navigation' and value.get('navigation_job_id') not in (None,job['id']):return
        value.update(capture_job_update(job)); self.capture(value)

    def invalidate_scene(self, manifest, removed_paths):
        """Invalidate successful derived results whose explicit inputs vanished.

        The deletion owner supplies the actual removed paths. This only changes
        task/capture metadata; unrelated source files and completed maps remain.
        """
        removed = {str(Path(path).expanduser().resolve()) for path in removed_paths if path}
        if not removed:
            return 0
        selected = str(Path(manifest).expanduser().resolve()) if manifest else None
        changed = 0
        rows = self.db.execute("SELECT * FROM jobs WHERE state='succeeded' AND kind='navigation'").fetchall()
        for row in rows:
            job = self.row(row)
            result = job.get('result') or {}
            scene = result.get('scene')
            dependencies = set()
            source = job['payload'].get('cloud')
            if source:
                dependencies.add(str(Path(source).expanduser().resolve()))
            if scene:
                scene = Path(scene).expanduser().resolve()
                dependencies.add(str(scene))
                document = _json_object(scene)
                if document:
                    for name in ('cloud', 'grid', 'scan_context'):
                        path = document.get(name)
                        if isinstance(path, str) and path:
                            dependencies.add(str((scene.parent / path).expanduser().resolve()))
            # An already deleted manifest is still identifiable from its job's
            # result, without requiring the removed JSON document to be read.
            if selected and selected in removed and scene is not None and str(scene) == selected:
                dependencies.add(selected)
            if not dependencies.intersection(removed):
                continue
            message = '地图所需文件已删除，请重新选择数据或重新准备导航地图'
            self.update(job['id'], state='failed', result=None, error=message)
            capture_id = job['payload'].get('capture_id')
            capture_row = self.db.execute('SELECT value FROM captures WHERE id=?', (capture_id,)).fetchone()
            if capture_row:
                value = json.loads(capture_row[0])
                if value.get('navigation_job_id') in (None, job['id']):
                    value.update(navigation_scene='', navigation_warning='')
                    value.update(capture_job_update(self.get(job['id'])))
                    self.capture(value)
            changed += 1
        return changed

    def _capture_job(self, capture_id, kind, hint=None):
        if hint:
            found = self.get(hint)
            if found and found['kind'] == kind and found['payload'].get('capture_id') == capture_id:
                return found
            return None
        row = self.db.execute("SELECT * FROM jobs WHERE kind=? AND CASE WHEN json_valid(payload) "
                              "THEN json_extract(payload,'$.capture_id') END=? ORDER BY created DESC LIMIT 1",
                              (kind, capture_id)).fetchone()
        return self.row(row) if row else None

    def _directory_audit(self, directory):
        target = Path(directory).expanduser().resolve()
        for row in self.db.execute("SELECT * FROM jobs WHERE kind='audit' ORDER BY created DESC"):
            value = self.row(row)
            source = value['payload'].get('directory')
            if isinstance(source, str) and source and Path(source).expanduser().resolve() == target:
                return value
        return None

    def reconcile_capture_tasks(self, *, last_capture_path=None):
        """Repair the durable-save/GUI-callback gap without replaying old failures.

        Only registered captures and the recorder's one explicit latest receipt
        are inspected. No bag library scan or source-data modification occurs.
        Existing jobs, including cancelled/failed ones, retain their outcome.
        """
        captures = []
        rows = self.db.execute('SELECT value FROM captures ORDER BY updated').fetchall()
        stopped_navigation = {'ready', 'failed', 'deferred', 'paused', 'cancelled'}
        stopped_audit = {'complete', 'incomplete', 'failed', 'cancelled', 'paused', 'deferred'}
        for row in rows:
            try:
                value = json.loads(row[0])
                if not isinstance(value, dict):
                    continue
                capture_id = value['capture_id']
                if value.get('finished') is True and value.get('map') and value.get('navigation_directory'):
                    old = self._capture_job(capture_id, 'navigation', value.get('navigation_job_id'))
                    if old:
                        value.update(capture_job_update(old))
                    elif value.get('navigation_state') not in stopped_navigation:
                        cloud = Path(value['map']).expanduser().resolve()
                        if cloud.is_file() and cloud.suffix.lower() == '.pcd':
                            payload = normalize_task_payload('navigation', dict(cloud=str(cloud),
                                directory=value['navigation_directory'], title=value.get('title') or cloud.stem,
                                capture_id=capture_id, gravity_aligned=value.get('gravity_aligned', False),
                                grid_method=value.get('grid_method', 'observed')))
                            identifier = job_id('navigation', payload)
                            # A missing receipt for another revision cannot be
                            # silently rebound to a modified source file.
                            if value.get('navigation_job_id') in (None, identifier):
                                self.submit(identifier, 'navigation', payload, priority=40)
                                value.update(capture_job_update(self.get(identifier)))
                bag = value.get('bag')
                if isinstance(bag, str) and bag and value.get('audit_state') not in stopped_audit:
                    stored = (value.get('bag_stored') is True or value.get('storage_state') == 'stored'
                              or _stored_recording(bag))
                    if stored:
                        value.update(bag_stored=True, storage_state='stored')
                    old = self._capture_job(capture_id, 'audit', value.get('audit_job_id')) or self._directory_audit(bag)
                    if old:
                        value.update(capture_job_update(old))
                    elif stored:
                        requested = value.get('audit_request')
                        payload = copy.deepcopy(requested) if isinstance(requested, dict) else dict(
                            directory=str(Path(bag).expanduser().resolve()), capture_id=capture_id,
                            title=Path(bag).name)
                        if (isinstance(payload.get('directory'), str)
                                and Path(payload['directory']).expanduser().resolve() == Path(bag).expanduser().resolve()
                                and payload.get('capture_id') in (None, capture_id)):
                            payload = normalize_task_payload('audit', payload)
                            identifier = job_id('audit', payload)
                            self.submit(identifier, 'audit', payload, priority=60)
                            value.update(bag_stored=True, storage_state='stored', audit_request=payload)
                            value.update(capture_job_update(self.get(identifier)))
                self.capture(value)
                captures.append(value)
            except (OSError, ValueError, TypeError, KeyError):
                continue

        if last_capture_path is False:
            return
        if last_capture_path is None:
            state_dir = Path(os.environ['NAV_RECORD_STATE_DIR']).expanduser() if os.environ.get('NAV_RECORD_STATE_DIR') else Path.home()/'.local/state/nav'
            last_capture_path = state_dir/'last_capture.json'
        receipt = _json_object(last_capture_path)
        bag = receipt.get('path') if receipt else None
        if not isinstance(bag, str) or not bag or not _stored_recording(bag) or self._directory_audit(bag):
            return
        target = Path(bag).expanduser().resolve()
        owner = next((value for value in captures if isinstance(value.get('bag'), str)
                      and Path(value['bag']).expanduser().resolve() == target), None)
        if owner and owner.get('audit_state') in stopped_audit:
            return
        payload = normalize_task_payload('audit', dict(directory=str(target),
            capture_id=owner.get('capture_id') if owner else None, title=target.name))
        identifier = job_id('audit', payload)
        self.submit(identifier, 'audit', payload, priority=60)
        if owner:
            owner.update(bag_stored=True, storage_state='stored', audit_request=payload)
            owner.update(capture_job_update(self.get(identifier)))
            self.capture(owner)


def capture_job_update(job):
    """Only a job's own capture can consume its outcome; usable data is retained."""
    state = job['state']; result = job.get('result') or {}
    if job['kind'] == 'navigation':
        phase = {'succeeded':'ready','failed':'failed','cancelled':'deferred'}.get(state, 'waiting' if state != 'running' else 'processing')
        value = dict(navigation_state=phase, navigation_job_id=job['id'], navigation_progress=job.get('progress') or {})
        if state == 'succeeded': value.update(navigation_scene=result['scene'], navigation_warning=result.get('warning',''), navigation_error='',
            map_id='map-'+hashlib.sha256(str(result['scene']).encode()).hexdigest()[:24])
        elif state == 'failed': value['navigation_error'] = job.get('error','')
        return value
    if job['kind'] == 'audit':
        complete = result.get('state') == 'complete'
        return dict(audit_job_id=job['id'], audit_state=('complete' if complete else 'incomplete') if state == 'succeeded' else state,
                    bag_complete=complete, audit_error=job.get('error','') or '；'.join(result.get('errors') or []))
    if job['kind']=='recovery' and state=='succeeded':
        return dict(map=result['map'],recovery=True,finished=True,recovery_job_id=job['id'])
    return {}


class TaskCoordinator:
    def __init__(self, directory, *, command=None, systemd=None):
        self.root = Path(directory).expanduser()
        self.command = command or [sys.executable, '-m', 'nav_app.task_worker']
        self.systemd = (sys.platform == 'linux' and os.environ.get('NAV_TASK_SYSTEMD', '1') != '0') if systemd is None else systemd
        self.commands = queue.Queue(maxsize=1024)
        self.lock = threading.RLock()
        self.view = dict(jobs=[], captures=[], foreground=None, released=False, error='', loaded=False)
        self.owner = None
        self.active_id = None
        self.resource_lease=None
        self.closed = False
        self.stop_requested=threading.Event()
        self.failed=False
        self.thread = threading.Thread(target=self._run, name='nav-map-tasks', daemon=False)
        self.thread.start()

    def _put(self, *command):
        if self.closed or self.failed: raise RuntimeError('地图任务服务已关闭或需要恢复')
        try: self.commands.put_nowait(command)
        except queue.Full: raise RuntimeError('地图操作较多，请稍后重试') from None

    def submit(self, kind, payload, *, priority=50, retry=False):
        payload = normalize_task_payload(kind, payload)
        identifier = job_id(kind, payload)
        self._put('submit', identifier, kind, copy.deepcopy(payload), priority, retry)
        return identifier

    def save_capture(self, value): self._put('capture', copy.deepcopy(value))

    def invalidate_scene(self, manifest, removed_paths):
        self._put('invalidate_scene', str(manifest) if manifest else '', [str(path) for path in removed_paths])

    def foreground(self, owner):
        with self.lock:
            if self.closed or self.failed:return False
            if self.owner and owner and self.owner != owner: return False
            self.owner = owner
            self.view['foreground'] = owner
            # Prevent the old released snapshot authorizing a new owner before
            # the worker thread has observed this reservation.
            if owner: self.view['released'] = False
        try:self.commands.put_nowait(('foreground',owner))
        except queue.Full:pass  # self.owner is authoritative; the loop polls it.
        return True

    def reserve_idle(self, owner):
        """Atomic admission for legacy immediate starts, never a stale snapshot."""
        with self.lock:
            if self.closed or self.failed:return False
            if not self.view.get('loaded') or self.view.get('error') or self.active_id:
                return False
            if self.owner not in (None,owner):return False
            if not self._acquire_compute():return False
            self.owner=owner;self.view['foreground']=owner
        try:self.commands.put_nowait(('foreground',owner))
        except queue.Full:pass
        return True

    def _acquire_compute(self):
        if self.resource_lease is not None:return True
        from .resource_lease import ResourceLease, DeviceBusy
        try:self.resource_lease=ResourceLease('live').acquire()
        except DeviceBusy:return False
        return True

    def cancel(self, identifier): self._put('cancel', identifier)
    def promote(self, identifier): self._put('promote', identifier)
    def snapshot(self):
        with self.lock: return copy.deepcopy(self.view)

    def close(self):
        if self.closed: return
        self.closed = True;self.stop_requested.set()
        try:self.commands.put_nowait(('close',))
        except queue.Full:pass
        # GUI never joins a slow disk/process. Shutdown is handled by the owner
        # service and the coordinator; no new work can be admitted after this.

    def retry_failed(self):
        """Explicit user retry; keep accepted commands and all existing bindings.

        The previous thread must finish its process/resource cleanup first. GUI
        callers never join it, and a failed retry waits for another user action.
        """
        with self.lock:
            if self.closed:return False
            if not self.failed:return True
            if self.thread.is_alive():return False
            self.failed=False
            self.view.update(error='', released=False, loaded=False)
            self.active_id=None
            self.thread=threading.Thread(target=self._run,name='nav-map-tasks',daemon=False)
            self.thread.start()
            return True

    def _run(self):
        store = None; active = None; stopping = None; closed = False; owner = None; log = None; view_at=0.
        storage_checks = {}; storage_wait = ''
        lock_stream = None
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            lock_stream = (self.root/'owner.lock').open('a+b')
            if os.name == 'posix':
                import fcntl
                fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Scope names are deterministic and private to this database. Reap
            # interrupted workers via systemd before retrying their inputs.
            try:store = TaskStore(self.root)
            except sqlite3.DatabaseError as exc:
                if not any(word in str(exc).lower() for word in ('malformed','not a database','integrity')):raise
                suffix='.corrupt-'+uuid.uuid4().hex
                for name in ('tasks.sqlite3','tasks.sqlite3-wal','tasks.sqlite3-shm'):
                    path=self.root/name
                    if path.exists():path.rename(path.with_name(name+suffix))
                store=TaskStore(self.root)
            for request in (self.root/'requests').glob('*.json'):
                try:
                    value=json.loads(request.read_text(encoding='utf-8'))
                    if (not isinstance(value,dict) or request.stem!=value.get('id') or len(request.stem)!=32
                            or any(c not in '0123456789abcdef' for c in request.stem)
                            or not isinstance(value.get('kind'),str) or not value['kind']
                            or not isinstance(value.get('payload'),dict)):continue
                    if not store.get(value['id']):
                        store.submit(value['id'],value['kind'],value['payload'],value.get('priority',50))
                        if value.get('state') in TERMINAL:
                            store.update(value['id'],state=value['state'],result=value.get('result'),error=value.get('error',''))
                except (OSError,ValueError,KeyError,TypeError):pass
            for receipt in (self.root/'captures').glob('*.json'):
                try:
                    value=json.loads(receipt.read_text(encoding='utf-8'))
                    if not store.db.execute('SELECT 1 FROM captures WHERE id=?',(value['capture_id'],)).fetchone():store.capture(value)
                except (OSError,ValueError,KeyError,TypeError):pass
            store.reconcile_capture_tasks()
            stale = list(self.root.glob('*/unit.json'))
            for marker in stale:
                if marker.exists() and self.systemd:
                    unit = json.loads(marker.read_text())['unit']
                    if not unit.startswith('nav-task-'): raise ValueError('地图任务进程标识无效')
                    subprocess.run(['systemctl','--user','stop',unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
            store.recover()
            while True:
                closed=closed or self.stop_requested.is_set()
                try: command = self.commands.get(timeout=.1)
                except queue.Empty: command = None
                while command:
                    op, *args = command
                    if op == 'submit': store.submit(*args)
                    elif op == 'capture': store.capture(args[0])
                    elif op == 'invalidate_scene': store.invalidate_scene(*args)
                    elif op == 'foreground': owner = args[0]
                    elif op == 'promote': store.update(args[0], priority=0)
                    elif op == 'cancel':
                        row = store.get(args[0])
                        if row and row['state'] not in TERMINAL:
                            if active and active['id'] == args[0]: active['user_cancel'] = True
                            else: store.update(args[0], state='cancelled')
                    elif op == 'close': closed = True
                    try: command = self.commands.get_nowait()
                    except queue.Empty: command = None

                if active:
                    process = active['process']
                    pending=store.next()
                    current=store.get(active['id'])
                    priority_preempt=bool(pending and current and pending['priority']<current['priority'])
                    if (self.owner or closed or active.get('user_cancel') or priority_preempt) and stopping is None:
                        stopping = time.monotonic()
                        store.update(active['id'], state='cancelling')
                        self._signal(active, signal.SIGTERM)
                    if stopping is not None and time.monotonic()-stopping > 5 and process.poll() is None:
                        self._signal(active, signal.SIGKILL if os.name == 'posix' else signal.SIGTERM)
                    progress = active['path']/'progress.json'
                    if progress.is_file() and progress.stat().st_size <= 32768:
                        try:
                            value = json.loads(progress.read_text(encoding='utf-8'))
                            if not isinstance(value,dict):value={}
                            if value != active.get('progress'):
                                # Progress updates are deliberately coalesced to
                                # at most one durable write every two seconds.
                                active['progress'] = value
                                if time.monotonic()-active.get('progress_at',0) >= 2:
                                    store.update(active['id'], progress=value); active['progress_at'] = time.monotonic()
                        except (OSError, ValueError): pass
                    code = process.poll()
                    if code is not None and not self._children_released(active):
                        if 'cleanup_at' not in active:
                            active['cleanup_at']=time.monotonic()
                            store.update(active['id'],state='cancelling',error='正在回收尚未退出的地图子任务')
                            self._signal(active,signal.SIGTERM)
                        elif time.monotonic()-active['cleanup_at']>5:
                            self._signal(active,signal.SIGKILL if os.name=='posix' else signal.SIGTERM)
                        code=None
                    if code is not None:
                        data = {}
                        try: data = json.loads((active['path']/'result.json').read_text(encoding='utf-8'))
                        except (OSError, ValueError): pass
                        if not isinstance(data,dict):data={}
                        if code == 0 and data.get('ok') and 'cleanup_at' not in active:
                            store.update(active['id'], state='succeeded', result=data.get('value'), progress=active.get('progress',{}), error='')
                        elif stopping is not None:
                            store.update(active['id'], state='cancelled' if active.get('user_cancel') else
                                         ('waiting_foreground' if self.owner else 'queued'), error='')
                        else:
                            store.update(active['id'], state='failed', error=str(data.get('error') or f'地图处理进程退出（{code}），已有数据保留')[:1000])
                        store.complete_capture_job(store.get(active['id']))
                        if log: log.close(); log = None
                        active = None; stopping = None
                        with self.lock:self.active_id=None
                with self.lock:
                    owner = self.owner
                    pending_work=store.next() or store.db.execute("SELECT 1 FROM jobs WHERE state='waiting_foreground' LIMIT 1").fetchone()
                    if not active and not owner and not pending_work and self.resource_lease is not None:
                        self.resource_lease.close();self.resource_lease=None
                    compute_ready = self._acquire_compute() if owner and not active else True
                if not owner and not closed and not active:
                    with store.db:
                        store.db.execute("UPDATE jobs SET state='queued' WHERE state='waiting_foreground'")
                    job = store.next()
                    storage_wait = ''
                    if job:
                        sample = storage_checks.get(job['id'])
                        if sample is None or time.monotonic()-sample[0] >= 5:
                            from .task_storage import task_storage_reason
                            reason = task_storage_reason(job['kind'], job['payload'], self.root/job['id']/'work')
                            sample = (time.monotonic(), reason)
                            storage_checks[job['id']] = sample
                        storage_wait = sample[1]
                        if storage_wait:
                            if job['error'] != storage_wait or job['progress'].get('stage') != 'storage':
                                store.update(job['id'], progress=dict(stage='storage',state='waiting'), error=storage_wait)
                        elif job['progress'].get('stage') == 'storage':
                            store.update(job['id'], progress={}, error='')
                    with self.lock:
                        if storage_wait and self.owner is None and self.resource_lease is not None:
                            self.resource_lease.close(); self.resource_lease=None
                        can_start = self._acquire_compute() if job and not storage_wait else False
                    if job and can_start:
                        path = self.root/job['id']; path.mkdir(exist_ok=True)
                        for name in ('result.json','progress.json'): (path/name).unlink(missing_ok=True)
                        from .capture_quality import atomic_json
                        atomic_json(path/'spec.json', dict(kind=job['kind'], payload=job['payload'], work_directory=str(path/'work')))
                        attempt = job['attempt']+1
                        unit = 'nav-task-'+job['id']+'-'+str(attempt)+'.scope'
                        atomic_json(path/'unit.json', dict(unit=unit))
                        owner_start=''
                        if sys.platform=='linux':owner_start=Path(f'/proc/{os.getpid()}/stat').read_text().rsplit(')',1)[1].split()[19]
                        argv = [*self.command, '--spec', str(path/'spec.json'), '--owner-pid', str(os.getpid()), '--owner-start',owner_start]
                        if self.systemd:
                            argv = ['systemd-run','--user','--scope','--quiet','--unit='+unit,
                                    '--property=CPUQuota=200%','--property=MemoryHigh=3G','--property=MemoryMax=5G',
                                    '--property=IOWeight=10',*argv]
                        env = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
                        log = (path/'worker.log').open('ab')
                        with self.lock:
                            if self.owner is None:
                                process = subprocess.Popen(argv, stdout=log, stderr=log, env=env, start_new_session=os.name=='posix')
                                active = dict(id=job['id'], process=process, unit=unit, path=path)
                                self.active_id = job['id']
                                store.update(job['id'], state='running', attempt=attempt, error='')
                            else:log.close();log=None
                if time.monotonic()-view_at<.25 and not closed:continue
                view_at=time.monotonic()
                view = store.snapshot()
                if active and active.get('progress'):
                    for row in view['jobs']:
                        if row['id'] == active['id']: row['progress'] = active['progress']
                with self.lock:
                    ready=active is None and (self.owner is None or self.resource_lease is not None)
                    view.update(foreground=self.owner, released=ready, loaded=True, error='',
                                wait_reason=('正在等待前台算法释放设备，可结束前台任务后继续或取消本次准备'
                                             if self.owner and self.resource_lease is None else
                                             (storage_wait if not active and not self.owner else '')))
                    self.view = view
                if closed and not active: break
        except Exception as exc:
            with self.lock:
                self.failed=True
                self.view.update(error=str(exc), released=False, loaded=True)
        finally:
            if active:
                try:
                    self._signal(active,signal.SIGTERM)
                    end=time.monotonic()+5
                    while time.monotonic()<end:
                        if active['process'].poll() is not None and self._children_released(active):break
                        time.sleep(.05)
                    else:self._signal(active,signal.SIGKILL if os.name=='posix' else signal.SIGTERM)
                except (OSError,subprocess.SubprocessError):
                    try:self._signal(active,signal.SIGKILL if os.name=='posix' else signal.SIGTERM)
                    except (OSError,subprocess.SubprocessError):pass
            if log: log.close()
            if self.resource_lease is not None:self.resource_lease.close();self.resource_lease=None
            if store: store.db.close()
            if lock_stream: lock_stream.close()

    def _signal(self, active, sig):
        if self.systemd:
            subprocess.run(['systemctl','--user','kill','--signal='+str(int(sig)),active['unit']],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        else:
            try:
                if os.name == 'posix': os.killpg(active['process'].pid, sig)
                else: active['process'].terminate()
            except ProcessLookupError: pass

    def _children_released(self, active):
        if self.systemd:
            result=subprocess.run(['systemctl','--user','show',active['unit'],'--property=ActiveState','--property=LoadState'],
                                  capture_output=True,text=True,timeout=5)
            fields=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
            if fields.get('LoadState')=='not-found':return True
            return fields.get('ActiveState') in ('inactive','failed')
        if sys.platform=='linux':
            for path in Path('/proc').glob('[0-9]*/stat'):
                try:
                    data=path.read_text().rsplit(')',1)[1].split()
                    if data[0]!='Z' and int(data[2])==active['process'].pid:return False
                except (OSError,ValueError,IndexError):continue
        return True

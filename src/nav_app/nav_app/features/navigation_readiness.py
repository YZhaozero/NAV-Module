"""Nonblocking, generation-scoped Nav2 lifecycle readiness and bounded resume."""
import threading
import time


class NavigationReadiness:
    def __init__(self, state_client, state_request, *, manager_client=None,
                 resume_request=None, localization_ready=lambda: False,
                 clock=time.monotonic):
        self.state_client, self.state_request = state_client, state_request
        self.manager_client, self.resume_request = manager_client, resume_request
        self.localization_ready, self.clock = localization_ready, clock
        self.lock = threading.RLock()
        self.clients = {}; self.rows = {}; self.generation = 0
        self.backend = 'walking'; self.mode = 'walking'; self.enabled = True
        self.reason = '正在确认路线服务状态'
        self._manager = None; self._resume = None
        self._resume_at = 0.; self._resume_attempted = False
        self._inactive_since = None; self._startup_failed = False

    @staticmethod
    def _discard(client, future):
        if future is None:return
        try:
            remove = getattr(client, 'remove_pending_request', None)
            if callable(remove):remove(future)
        except (KeyError, RuntimeError, ValueError):pass
        try:future.cancel()
        except (AttributeError, RuntimeError):pass

    def reset(self):
        with self.lock:
            self.generation += 1
            for name, row in self.rows.items():
                self._discard(self.clients.get(name), row.get('future'))
            self.rows = {}
            self._discard(self._manager, self._resume)
            self._resume = None; self._resume_attempted = False
            self._inactive_since = None; self._startup_failed = False
            self.enabled = False
            self.reason = '路线服务已停止，请重新准备设备'

    def configure(self, backend, mode='walking'):
        self.reset()
        with self.lock:
            self.backend, self.mode = backend, mode
            self.enabled = backend is not None
            self.reason = '正在确认路线服务状态'

    def observe_log(self, text):
        with self.lock:
            if self.enabled and any(marker in text.lower() for marker in (
                    'failed to bring up all requested nodes', 'aborting bringup',
                    'failed to change state for node')):
                self._startup_failed = True

    def _state(self, name, now):
        client = self.clients.get(name)
        if client is None:
            client = self.clients[name] = self.state_client('/'+name+'/get_state')
        row = self.rows.setdefault(name, dict(state=None, received=-1e9, next_at=0., future=None))
        future = row['future']
        if future is not None and now-row['requested'] > 2.:
            row['future'] = None; row['state'] = None
            self._discard(client, future)
        if row['future'] is None and now >= row['next_at']:
            row['next_at'] = now+.5
            try:
                if not client.service_is_ready():
                    row['state'] = None
                else:
                    generation = self.generation
                    future = client.call_async(self.state_request())
                    row['future'] = future; row['requested'] = now
                    def completed(reply):
                        with self.lock:
                            if generation != self.generation or row.get('future') is not reply:return
                            row['future'] = None
                            try:row['state'] = int(reply.result().current_state.id)
                            except Exception:row['state'] = None
                            row['received'] = self.clock()
                    future.add_done_callback(completed)
            except (RuntimeError, ValueError):
                row['future'] = None; row['state'] = None
        return row['state'] if now-row['received'] <= 1.5 else None

    def ready(self, action_ready, robot_action_ready=True):
        with self.lock:
            if not self.enabled:return False
            if self.backend not in ('walking', 'nav2'):
                return bool(action_ready and (self.mode != 'robot' or robot_action_ready))
            now = self.clock()
            required = ['planner_server']
            if self.backend == 'walking' or self.mode == 'robot':required += ['map_server']
            if self.mode == 'robot':required += ['bt_navigator', 'controller_server']
            states = {name:self._state(name, now) for name in required}
            if self.backend == 'walking' and self.mode != 'robot':
                map_state = states['map_server']
                self._recover_walking(states['planner_server'], map_state, now)
            if not action_ready or self.mode == 'robot' and not robot_action_ready:
                self.reason = '正在等待路线服务连接'
                return False
            if any(value != 3 for value in states.values()):
                if not self._resume_attempted:
                    labels = dict(planner_server='路线规划', map_server='地图服务',
                                  bt_navigator='导航任务', controller_server='运动控制')
                    pending = [labels[name] for name, value in states.items() if value is None]
                    if pending:
                        self.reason = '等待 Nav2 状态回执：' + '、'.join(pending)
                    else:
                        inactive = [labels[name] for name, value in states.items() if value != 3]
                        self.reason = 'Nav2 尚未激活：' + '、'.join(inactive)
                return False
            self.reason = ''
            return True

    def _recover_walking(self, planner, map_state, now):
        if self._resume is not None and now-self._resume_at > 8.:
            reply, self._resume = self._resume, None
            self._discard(self._manager, reply)
            self.reason = '路线服务恢复回执超时，请结束后重新准备；定位结果已保留'
        if self._resume_attempted:return
        if planner != 2 or map_state != 2 or not self.localization_ready():
            self._inactive_since = None
            return
        if self._inactive_since is None:self._inactive_since = now
        # A clear startup abort allows recovery promptly. Otherwise wait beyond
        # the usual service-call window, never compete with a normal activation.
        if not self._startup_failed and now-self._inactive_since < 30.:
            return
        if self.manager_client is None or self.resume_request is None:return
        generation = self.generation
        try:
            if self._manager is None:self._manager = self.manager_client('/lifecycle_manager_walking/manage_nodes')
            if not self._manager.service_is_ready():return
            future = self._manager.call_async(self.resume_request())
            self._resume = future; self._resume_at = now; self._resume_attempted = True
            self.reason = '正在恢复路线服务激活，定位结果已保留'
            def completed(reply):
                with self.lock:
                    if generation != self.generation or self._resume is not reply:return
                    self._resume = None
                    try:success = bool(reply.result().success)
                    except Exception:success = False
                    self.reason = ('正在核对路线服务激活结果' if success else
                                   '路线服务未能激活，请结束后重新准备；定位结果已保留')
            future.add_done_callback(completed)
        except (RuntimeError, ValueError):
            self._resume_attempted = True
            self.reason = '路线服务恢复请求失败，请结束后重新准备'

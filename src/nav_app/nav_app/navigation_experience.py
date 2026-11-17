"""One live walking workflow for the touch screen and paired phone.

ROS/capture commands are queued to the GUI owner. Network planning runs outside
the state lock; results are committed only to the session that requested them.
"""
import copy
import math
import threading

from .outdoor import coordinate


def experience_for(state, services):
    with state.lock:
        if not hasattr(state, '_walking_experience'):
            state._walking_experience = NavigationExperience(state, services)
        return state._walking_experience


class NavigationExperience:
    def __init__(self, state, services):
        self.state, self.services = state, services
        self.waypoints = []
        self.manual = False
        self.editing = False
        self.revision = 0
        self.route_token = None
        self.route_reference = None
        self.busy = False
        self.message = ''
        self.runtime = {}
        self.locating = False
        self.session = state.trial.identifier

    def sync_session(self):
        if self.session != self.state.trial.identifier:
            self.session = self.state.trial.identifier
            self.waypoints = []
            self.editing = False
            self.message = ''
            self.invalidate()

    def token(self):
        s = self.state
        return (s.trial.identifier, s.trial.active, s.odom_epoch,
                s.odom_continuity, s.trial.quality_policy, self.revision)

    def ready(self):
        s = self.state
        t = s.trial.snapshot(s)
        return self.reference_ready(t) and t['odometry_live']

    def reference_ready(self, trial=None):
        """Editing geographic waypoints needs a valid frame, not a new pose."""
        t = trial if trial is not None else self.state.trial.snapshot(self.state)
        return bool(not self.locating and t['active'] and t['aligned']
                    and t['map_projection_ready']
                    and not any(self.runtime.get(key) for key in ('saving', 'preparing', 'restarting')))

    def current(self):
        value = self.state.snapshot().get('bd09')
        if not value:
            raise ValueError('当前位置尚未更新，请等待定位恢复')
        return list(value['point'])

    def invalidate(self):
        self.revision += 1
        self.route_token = None
        self.route_reference = None

    def map_reference(self):
        """Display-only last accepted satellite location, never a navigation fix."""
        value=getattr(self.state,'_map_display_reference',None)
        if (not isinstance(value,dict) or value.get('epoch')!=self.state.pose_epoch
                or not coordinate(value.get('point'))):return None
        age=self.state.clock()-value['stamp']
        if not math.isfinite(age) or age<-.05:return None
        return dict(point=list(value['point']),stamp=value['stamp'],age_s=max(0.,age),
                    fresh=age<=1.5,quality=value.get('quality'),source='gnss',display_only=True)

    def snapshot(self):
        with self.state.lock:
            self.sync_session()
            s = self.state.snapshot()
            t = s['navigation_trial']
            reference_ready = self.reference_ready(t)
            live_ready = reference_ready and t['odometry_live']
            valid_route = self.route_token == self.token() and self.route_reference is self.state.route
            phase = ('idle' if not t['active'] else 'navigate' if s['guiding'] else
                     'plan' if self.editing and reference_ready else 'align')
            return dict(phase=phase, trial=t, visual=self.state.trial.visual_snapshot(self.state),
                        position=s['position'], bd09=s['bd09'], guidance=s['guidance'],
                        map_position_age_s=(max(0.,self.state.clock()-s['bd09']['stamp']) if s['bd09'] else None),
                        map_reference=self.map_reference(),
                        guiding=s['guiding'], route=s['route'] if valid_route else None,
                        waypoints=copy.deepcopy(self.waypoints), manual=self.manual,
                        busy=self.busy, message=self.message, runtime=copy.deepcopy(self.runtime),
                        locating=self.locating,
                        can_plan=live_ready or phase == 'plan',
                        can_guide=bool(valid_route and live_ready and s['bd09']),
                        phone_motion=s['phone_motion'],
                        conversion_error=s['conversion_error'], route_assist=s['route_assist'])

    def command(self, data):
        if not isinstance(data, dict):
            raise ValueError('操作格式无效')
        action = data.get('action')
        s = self.state
        with s.lock:
            self.sync_session()
            if 'trial_id' in data and data['trial_id'] != s.trial.identifier:
                raise ValueError('行程已变化，请查看当前状态后重新操作')
            if action == 'route_assist':
                s.set_route_assistance(data.get('value'))
                return {'ok': True}
            if action=='phone_motion':
                s.set_phone_motion(data.get('value'))
                self.invalidate()
                return {'ok':True}
            if action in ('begin', 'save', 'restart', 'discard', 'camera', 'satellites', 'back'):
                if action == 'begin' and (s.trial.active or self.runtime.get('preparing')):
                    return {'ok': True}
                if action == 'save' and not self.runtime.get('can_save',
                        s.trial.active and not self.runtime.get('saving')):
                    raise ValueError('当前没有可保存的采集，或正在保存')
                if action == 'restart' and not self.runtime.get('can_restart', False):
                    raise ValueError('当前正在准备或保存，请等待本段数据保存完成')
                if action == 'discard' and not self.runtime.get('can_discard', False):
                    raise ValueError('数据正在保存整理，请保持通电并等待完成')
                request = dict(action='walking_ui', command=action, trial_id=s.trial.identifier)
                if request not in s.requests:
                    s.requests.append(request)
                return {'ok': True}
            if action == 'quality':
                s.trial.set_quality_policy(s, data.get('value'))
                self.invalidate()
                return {'ok': True}
            if action == 'rtk':
                if not isinstance(data.get('value'), bool):
                    raise ValueError('RTK开关状态无效')
                s.trial.set_rtk(s, data['value'])
                return {'ok': True}
            if action == 'stop':
                s.control('stop')
                self.editing = True
                return {'ok': True}
            if action == 'manual_anchor':
                if (not s.trial.active or s.guiding or self.busy or self.locating
                        or self.runtime.get('saving') or self.runtime.get('preparing')):
                    raise ValueError('请先开始导航与建图，并结束当前引导后确认位置')
                if not coordinate(data.get('point')) or not coordinate(data.get('direction')):
                    raise ValueError('请确认当前位置和设备正前方的方向点')
                if not s.odom or not -.05 <= s.clock()-s.odom[-1][0] <= .5:
                    raise ValueError('运动跟踪尚未恢复，可以先选点，恢复后再确认')
                self.invalidate()
                token = self.token()
                odometry = copy.deepcopy(s.odom[-1])
                self.locating = True
                self.message = '正在确认手动位置与方向，请保持设备稳定'
                threading.Thread(target=self._manual_anchor,
                    args=(token, list(data['point']), list(data['direction']), odometry),
                    name='manual-map-position', daemon=True).start()
                return {'ok': True}
            editing_action = action in ('select', 'undo', 'delete', 'loop', 'manual', 'plan')
            can_edit = editing_action and self.editing and self.reference_ready()
            if not can_edit and not self.ready():
                raise ValueError('请等待方向对齐和运动跟踪恢复；建图与录制继续')
            if action == 'guide':
                if self.route_token != self.token() or self.route_reference is not s.route:
                    raise ValueError('路线已变更，请重新规划')
                if s.guiding:
                    return {'ok': True}
                s.control('start')
                return {'ok': True}
            if s.guiding and action != 'replan':
                raise ValueError('请先结束引导或选择从当前位置重规划')
            if action in ('edit', 'replan', 'current'):
                point = self.current()  # Validate before stopping the current guide.
                if s.guiding:
                    s.control('stop')
                destination = self.waypoints[-1:] if action == 'replan' else []
                self.waypoints = [point] + copy.deepcopy(destination)
                self.editing = True
                self.invalidate()
                self.message = '已以当前位置为起点，请按顺序点选途经点'
            elif action == 'select':
                if not self.editing or not coordinate(data.get('point')):
                    raise ValueError('请先进入路线规划，再在地图上选点')
                if len(self.waypoints) >= 20:
                    raise ValueError('已选20个点，请先规划或删除一个点')
                s.trial.projection.to_enu(data['point'])
                self.waypoints.append(list(data['point']))
                self.invalidate()
            elif action in ('undo', 'delete'):
                index = len(self.waypoints)-1 if action == 'undo' else data.get('index')
                if not isinstance(index, int) or not 0 < index < len(self.waypoints):
                    raise ValueError('起点保留，可用“当前位置”重设起点')
                self.waypoints.pop(index)
                self.invalidate()
            elif action == 'loop':
                if not 2 <= len(self.waypoints) < 20:
                    raise ValueError('请先选择途经点，最多20个点')
                self.waypoints.append(list(self.waypoints[0]))
                self.invalidate()
            elif action == 'manual':
                if not isinstance(data.get('value'), bool):
                    raise ValueError('路线类型无效')
                self.manual = data['value']
                self.invalidate()
            elif action == 'plan':
                if self.busy:
                    raise ValueError('正在规划，请稍候；可以修改选点以取消旧结果')
                if len(self.waypoints) < 2:
                    raise ValueError('请在地图上选择终点')
                token = self.token()
                points, manual = copy.deepcopy(self.waypoints), self.manual
                self.busy = True
                self.message = '正在规划步行路线'
                threading.Thread(target=self._plan, args=(token, points, manual),
                                 name='walking-route-plan', daemon=True).start()
            else:
                raise ValueError('不支持此操作')
            return {'ok': True}

    def _manual_anchor(self, token, selected, direction, odometry):
        from .manual_alignment import map_reference, alignment_at
        s = self.state
        try:
            origin, projection = map_reference(self.services, selected)
            with s.lock:
                if token != self.token():
                    return
                if (not s.odom or not -.05 <= s.clock()-s.odom[-1][0] <= .5
                        or math.dist(s.odom[-1][1], odometry[1]) > 1.):
                    raise ValueError('设备已移动，请停稳后重新确认当前位置')
                anchor = alignment_at(origin, projection, selected, direction,
                                      s.odom[-1], s.odom_continuity, s.clock())
                # Explicit manual reference, never a manufactured RTK fix.
                s.origin = origin
                s.alignment = anchor
                s._anchor_odometry()
                s.trial.projection = projection
                s.trial.rtk_enabled = False
                s.trial.preview = None
                s.trial.display_anchor = None
                s.trial.pairs.clear()
                s.trial.pair_weights.clear()
                s.trial.last_candidate = None
                s.trial.stable = 0
                from .alignment_samples import StableAlignmentSamples
                s.trial.stable_samples = StableAlignmentSamples()
                s._alignment_pending.clear()
                s.previous_output = None
                s.pose_epoch += 1
                s.converted = None
                from .geo_maps import projection_record
                s.assets['manual_location'] = copy.deepcopy(dict(anchor,
                    origin_wgs84=list(origin.values), capture_frame=s.odom_frame,
                    odom_epoch=s.odom_epoch, session_id=s.session_id,
                    bd09_projection=projection_record(projection, origin.values)))
                s.trial.reason = '位置与方向已手动确认，由运动跟踪接续'
                s.trial.event(s, 'manual_alignment', selected_bd09=selected, direction_bd09=direction,
                              reference=copy.deepcopy(s.assets['manual_location']))
                pose = s._position_now()
                if pose:
                    s.update_conversion(pose, projection.to_bd09(
                        origin.enu(*pose['wgs84'], 0.)[:2]))
                # Confirmation is a durable user action, not a volatile UI
                # preview that relies on a later route plan or shutdown to save.
                try:
                    s.save_session()
                except ValueError:
                    self.message = '手动定位已确认，但记录未保存，请检查存储空间并重新确认'
                else:
                    self.message = '手动定位已确认并保存，可规划路线；卫星数据继续记录'
        except (ValueError, KeyError, TypeError) as exc:
            with s.lock:
                if token == self.token():
                    self.message = str(exc) if isinstance(exc, ValueError) else '手动位置确认未完成，请重试'
        finally:
            with s.lock:
                self.locating = False

    def _plan(self, token, points, manual):
        result, error = None, ''
        try:
            result = self.services.plan_waypoints(points, manual=manual)
        except ValueError as exc:
            error = str(exc)
        except Exception:
            error = '步行路线暂不可用，请检查联网或选择手动画线'
        with self.state.lock:
            self.busy = False
            if token != self.token() or not self.reference_ready() or self.state.guiding:
                self.message = '选点或定位已变化，请重新规划'
                return
            if error:
                self.message = error
                return
            try:
                # Validate the entire route in the same projection used by LIO.
                for p in result['points']:
                    self.state.trial.projection.to_enu(p)
                self.state.set_route(result)
                self.route_token = token
                self.route_reference = self.state.route
                self.message = '步行路线已准备好，可开始引导'
            except (ValueError, OSError) as exc:
                self.message = str(exc) if isinstance(exc, ValueError) else '路线保存失败，请检查存储空间'

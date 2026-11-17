"""ROS boundary for outdoor observations. It intentionally publishes no TF."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import copy
import json
import math
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from ..outdoor import OutdoorState
from ..outdoor_service import MapServices, PositionConverter


def read_calibration(path):
    if not path:
        return {}
    try:
        path=Path(path).expanduser()
        if path.stat().st_size>8192:
            return {}
        data=json.loads(path.read_text(encoding="utf-8"))
        arm=data.get("antenna_in_body_m")
        offset=data.get("gnss_to_odom_offset_s")
        if (not isinstance(arm,list) or len(arm)!=3 or any(type(v) not in (float,int) or not math.isfinite(v) or abs(v)>3 for v in arm)
                or type(offset) not in (float,int) or not math.isfinite(offset) or abs(offset)>1
                or data.get("odom_child_frame") not in ("body", "base_link")
                or type(data.get("time_verified")) is not bool or type(data.get("lever_arm_verified")) is not bool):
            return {}
        keys = ("antenna_in_body_m", "gnss_to_odom_offset_s", "time_verified", "lever_arm_verified",
                "odom_child_frame", "timing_policy", "installation_policy", "installation_source",
                "antenna_uncertainty_m")
        result = {k:data[k] for k in keys if k in data}
        if 'installation_policy' in result or 'timing_policy' in result:
            from ..gnss_alignment import alignment_configuration_ready
            if not alignment_configuration_ready(result):
                return {}
        return result
    except (OSError,ValueError,TypeError,AttributeError):
        return {}


class OutdoorAdapter:
    def __init__(self,node,initial_pose_adapter=None):
        self.node=node
        self.initial_pose_adapter=initial_pose_adapter
        self.calibration_path=Path(os.environ.get('NAV_OUTDOOR_CALIBRATION') or
                                   Path.home()/'.config/nav/outdoor-calibration.json').expanduser()
        self.state=OutdoorState(os.environ.get("NAV_OUTDOOR_DATA",str(Path.home()/".local/share/nav/outdoor")),
            clock=lambda:node.get_clock().now().nanoseconds/1e9,
            calibration=read_calibration(self.calibration_path))
        self.services=MapServices()
        self.converter=PositionConverter(self.state,self.services)
        self.pending=deque(maxlen=40)
        from ..phone_observation import PhoneFallback
        self.phone_fallback=PhoneFallback()
        self.subscription=node.create_subscription(String,"/rtk/observation",self.receive,10)
        self.phone_subscription=node.create_subscription(String,"/phone_gnss/rtk_solution",self.receive_phone,10)
        self.phone_motion_subscription=node.create_subscription(String,"/phone_gnss/motion_solution",self.receive_phone_motion,10)
        self.phone_absolute_subscription=node.create_subscription(String,"/phone_gnss/absolute_observations",self.receive_phone_absolute,32)
        self.phone_absolute_publisher=node.create_publisher(String,"/nav/phone_absolute_solution",10)
        self.odom_identity_subscription=node.create_subscription(String,'/fast_livo/diagnostics',self.odometry_identity,5)
        self.timer=node.create_timer(.1,self.poll)
        self.last_child=None
        self.map_subscription=node.create_subscription(String, '/localizer/localizer_node/quality', self.map_quality, 1)
        self.map_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='geo-map-save')
        self.map_save=None
        self.map_saves=deque()
        self.capture_owners={}
        self.status_publisher=node.create_publisher(String, '/nav/outdoor/localization', 10)
        self.navigation_publisher=node.create_publisher(String, '/nav/navigation/localization', 10)
        self.last_publish=0.
        self.initial_publisher=(
            initial_pose_adapter.initial_pose_publisher
            if initial_pose_adapter is not None else
            node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10))
        self.seeded_instance=None

    def install_calibration(self,path):
        calibration=read_calibration(path)
        if not calibration:raise ValueError('标定文件无效，请使用本设备的天线与时间标定文件')
        from .capture_quality import atomic_json
        with self.state.lock:
            if self.state.guiding or self.state.local_navigation_active:
                raise ValueError('请先结束导航，再更换方向标定')
            self.calibration_path.parent.mkdir(parents=True,exist_ok=True)
            atomic_json(self.calibration_path,calibration)
            self.state.calibration=calibration
            self.state.reset_odometry()
            self.pending.clear()
            self.state.alignment_reason='标定已更新，等待同步定位数据'
        return calibration

    def _publish_initial_pose(self, message, instance):
        if self.initial_pose_adapter is not None:
            if self.initial_pose_adapter.initial_pose_subscription_count() <= 0:
                return False
            return self.initial_pose_adapter.publish_rtk_initial_pose(
                message, instance)
        if self.initial_publisher.get_subscription_count() <= 0:
            return False
        self.initial_publisher.publish(message)
        return True

    def map_quality(self, message):
        try:
            if len(message.data)>8192:
                return
            data=json.loads(message.data)
            if isinstance(data,dict):
                if hasattr(self.node, 'field_performance'):
                    self.node.field_performance.quality = dict(data, app_received_at=self.state.clock())
                if hasattr(self.node, 'trajectory_test'):
                    self.node.trajectory_test.feed('localization', dict(data, app_received_at=self.state.clock(),
                        stamp=data.get('published_stamp_ns', 0)/1e9,
                        odom_source=getattr(self.node, '_odometry_backend', '')))
                self.state.feed_map_match(data)
                geo=self.state.selected_map
                instance=data.get('instance_id')
                if (geo and instance and data.get('map_sha256')==geo['sha256']
                        and data.get('valid') is False and instance!=self.seeded_instance
                        and data.get('match_sequence')==0):
                    pose=self.state.initial_map_pose()
                    if pose:
                        msg=PoseWithCovarianceStamped()
                        msg.header.frame_id=pose['frame']
                        msg.header.stamp=self.node.get_clock().now().to_msg()
                        p,q=msg.pose.pose.position,msg.pose.pose.orientation
                        p.x,p.y,p.z=pose['position_m']
                        q.x,q.y,q.z,q.w=pose['quaternion_xyzw']
                        msg.pose.covariance[0]=msg.pose.covariance[7]=pose['horizontal_variance_m2']
                        msg.pose.covariance[14]=4.
                        msg.pose.covariance[21]=msg.pose.covariance[28]=.1
                        msg.pose.covariance[35]=pose['yaw_variance_rad2']
                        if self._publish_initial_pose(msg,instance):
                            self.seeded_instance=instance
                            self.state.map_reason='已提供卫星初始位置与方向，等待点云匹配确认'
        except (ValueError,TypeError,KeyError,OverflowError):
            with self.state.lock:
                self.state.map_match=None

    def save_capture_reference(self, capture, reference):
        capture=copy.deepcopy(capture)
        owner=self.state.capture_session()
        key=capture.get('map') or capture.get('bag')
        if key:
            self.capture_owners[key]=owner
        self.map_save=self.map_worker.submit(self.state.register_capture,
            capture, copy.deepcopy(reference), owner)
        self.map_saves.append((self.map_save,owner['session_id']))

    def finish_capture(self, capture):
        # Same worker guarantees metadata update follows registration.
        capture=copy.deepcopy(capture)
        key=capture.get('map') or capture.get('bag')
        owner=self.capture_owners.pop(key,None) or self.state.capture_session()
        with self.state.lock:
            if owner['session_id']==self.state.session_id:
                owner=self.state.capture_session()
        def finish():
            try:
                self.state.finish_capture_assets(capture,owner)
            except (OSError,ValueError):
                with self.state.lock:
                    if owner['session_id']==self.state.session_id:
                        self.state.save_error='采集关联保存失败，原始点云和录包仍保留'
        self.map_worker.submit(finish)

    def receive(self,message):
        try:
            if len(message.data)>8192:
                return
            data=json.loads(message.data)
            if not isinstance(data,dict) or data.get("schema")!=1:
                return
            if hasattr(self.node, 'trajectory_test'):
                self.node.trajectory_test.feed('gnss', dict(data, app_received_at=self.state.clock()))
            if self.phone_fallback.receiver(data, self.state.clock()):
                self.pending.append((self.state.clock(),data))
        except (ValueError,TypeError):
            return

    def receive_phone(self, message):
        try:
            if len(message.data)>8192:return
            if self.state.phone_motion_enabled:return
            data=self.phone_fallback.phone(json.loads(message.data),self.state.clock())
            if data:
                if hasattr(self.node,'trajectory_test'):
                    self.node.trajectory_test.feed('gnss',dict(data,app_received_at=self.state.clock()))
                self.pending.append((self.state.clock(),data))
        except (ValueError,TypeError):
            return

    def receive_phone_motion(self,message):
        try:
            # Up to two bounded sets of 96 scalar satellite constraints.
            if len(message.data)>98304:return
            solution=json.loads(message.data)
            self.state.feed_phone_motion(solution)
            if not self.state.phone_motion_enabled:return
            data=self.phone_fallback.motion(solution,self.state.clock())
            if data:
                if hasattr(self.node,'trajectory_test'):
                    self.node.trajectory_test.feed('phone_motion',dict(data,app_received_at=self.state.clock()))
                self.pending.append((self.state.clock(),data))
        except (ValueError,TypeError):return

    def receive_phone_absolute(self,message):
        try:
            if len(message.data)>98304:return
            row=json.loads(message.data)
            with self.state.lock:self.state.phone_absolute.feed(self.state,row)
        except (ValueError,TypeError,KeyError,OverflowError):return

    def poll(self):
        from nav_rtk.parser.observation_time import time_state
        now=self.state.clock()
        while self.pending and now-self.pending[0][0]>=.12:
            if time_state(self.pending[0][1], now) == 'waiting':
                break
            _,data=self.pending.popleft()
            source=data.get('source')
            if source in ('phone_software_rtk','phone_continuity'):
                expected='phone_continuity' if self.state.phone_motion_enabled else 'phone_software_rtk'
                if source!=expected:continue
            elif not self.phone_fallback.receiver(data,now):
                # A weak receiver row may have queued just before the first
                # usable phone epoch. Recheck arbitration at consumption time.
                continue
            self.state.feed_fix(data)
        self.state.sample_actual_track()
        with self.state.lock:
            self.state.trial.poll_phone_heading(self.state)
            self.state.phone_absolute.poll(self.state)
            while self.state.phone_absolute.completed:
                row=self.state.phone_absolute.completed.popleft()
                try:self.phone_absolute_publisher.publish(String(data=json.dumps(row,ensure_ascii=False,allow_nan=False)))
                except (TypeError,ValueError):pass
        if self.state.trial.active and now-self.state.trial.last_logged>=1.:
            from .log_store import log_store
            with self.state.lock:
                trial=self.state.trial
                trial.last_logged=now
                record=dict(stamp=now,session_id=self.state.session_id,
                    trial=trial.snapshot(self.state),observation=self.state.observation,
                    odometry=self.state.odom[-1] if self.state.odom else None,
                    position=self.state._position_now(),origin=self.state.origin.values if self.state.origin else None)
            try:
                text=json.dumps(record,ensure_ascii=False,allow_nan=False)
            except (TypeError,ValueError):
                # A malformed optional observation must not stop the ROS timer
                # or live capture. The raw RTK log remains the primary record.
                record['observation']=None
                text=json.dumps(record,ensure_ascii=False,allow_nan=False)
            log_store().write('navigation','navigation-mapping',text)
        while getattr(self,'map_saves',None) and self.map_saves[0][0].done():
            completed,owner_session=self.map_saves.popleft()
            try:
                completed.result()
            except (OSError,ValueError):
                with self.state.lock:
                    if owner_session==self.state.session_id:
                        self.state.map_reason='地图地理参考保存失败，原始点云和录包仍保留'
            self.map_save=self.map_saves[-1][0] if self.map_saves else None
        if now-self.last_publish>=.5:
            with self.state.lock:
                pose=self.state._position_now()
                message=String()
                message.data=json.dumps(dict(schema=1,stamp=now,valid=pose is not None,pose=pose,
                    coordinate_system='WGS84',heading_convention='ENU yaw radians',
                    conflict=self.state.position_conflict,tf_owner='localizer',
                    simulation_included=False),allow_nan=False)
            self.status_publisher.publish(message)
            message=String()
            message.data=json.dumps(self.state.navigation_snapshot(),ensure_ascii=False,allow_nan=False)
            self.navigation_publisher.publish(message)
            self.last_publish=now

    def odometry_identity(self, message):
        if getattr(self.node, '_odometry_backend', '') != 'fast_livo':
            return
        try:
            if len(message.data) <= 16000:
                data = json.loads(message.data)
                if isinstance(data, dict) and data.get('schema') == 1:
                    self.state.feed_odom_instance(data.get('instance'))
        except (ValueError, TypeError):
            pass

    def odometry(self,message):
        child=message.child_frame_id
        frames=(message.header.frame_id,child)
        if self.last_child is not None and self.last_child!=frames:
            self.state.reset_odometry()
        self.last_child=frames
        self.state.odom_frame=message.header.frame_id
        configured=self.state.calibration.get("odom_child_frame")
        if configured and child!=configured:
            self.state.reset_odometry()
            self.state.alignment_reason="里程计坐标与天线配置不一致"
            return
        p,q=message.pose.pose.position,message.pose.pose.orientation
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.state.feed_odom(stamp,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    def close(self):
        self.state.phone_absolute.close()
        if self.state.trial.phone_heading_worker is not None:self.state.trial.phone_heading_worker.close()
        self.converter.close()
        self.map_worker.shutdown(wait=True)
        try:
            self.state.save_session()
        except ValueError:
            pass

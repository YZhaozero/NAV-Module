"""On-device evidence, deliberately conservative about causality."""
from collections import deque
import json
import math
import struct
import time


def header_stamp(payload):
    # ROS 2 CDR serialized Header is first in Image/Imu/PointCloud2/CustomMsg.
    if len(payload) < 12 or payload[0] != 0 or payload[1] not in (0, 1):
        raise ValueError('unsupported CDR encapsulation')
    sec, ns = struct.unpack_from('<iI' if payload[1] == 1 else '>iI', payload, 4)
    if ns >= 1000000000:
        raise ValueError('invalid header time')
    return sec+ns*1e-9


def diagnose(samples, now):
    if not samples or now-samples[-1]['received_monotonic'] > 3:
        return dict(code='no_telemetry', title='证据不足：FAST-LIVO2 遥测未更新',
                    advice='检查里程计是否启动、是否安装带诊断的版本；长时间计算或进程停顿也会造成此状态。')
    last = samples[-1]
    same = [s for s in samples if s['data'].get('instance') == last['data'].get('instance')]
    window = [s for s in same if now-s['received_monotonic'] <= 12]
    d = last['data']; raw = last.get('raw', {})
    if d.get('tracking_suspended'):
        return dict(code='tracking_suspended', title='运动跟踪已暂停，已采地图保留',
                    advice='请保存本段并重启跟踪。原始录制是否继续，请查看采集状态；缺失期间的位置不会自动补齐。')
    if d.get('pose_output_failed') or d.get('pose_output_pending', 0) > 20:
        return dict(code='pose_output_wait', title='位置输出延迟',
                    advice='正在等待位置数据送达。请保持设备稳定；保存地图不需要等待位置输出完成。')
    if len(window) < 5:
        return dict(code='warming', title='正在收集诊断证据（约 10 秒）', advice='保持当前启动批次，暂时不要切换或重启。')
    required = [raw.get(k) for k in ('lidar', 'imu')]
    if any(not r or now-r['received_monotonic'] > 2 for r in required):
        return dict(code='missing_input', title='原始雷达 / IMU 未持续到达', advice='检查传感器启动、话题和 QoS；这不是地图匹配耗时。')
    age = d.get('state_age_s', 0.)
    raw_age = max(abs(r['age_s']) for r in required)
    compute = [s['data'].get('busy_ratio', 0.) for s in window]
    backlog = [s['data'].get('lidar_backlog_s', 0.) for s in window]
    raw_bad = sum(any(abs(r.get('age_s', 0)) > 1 for k,r in s.get('raw', {}).items() if k in ('lidar', 'imu')) for s in window)
    slow = sum(c > .8 for c in compute) >= len(window)*.6
    waiting = sum(s['data'].get('wait_reason') in ('imu_behind', 'lidar_behind') for s in window) >= len(window)*.6
    offset = abs(d.get('latest_lidar_s', 0)-d.get('latest_imu_s', 0))
    if raw_bad >= len(window)*.6:
        return dict(code='input_time', title='原始输入时间链异常（待区分时钟偏差 / 上游延迟）',
                    advice='独立接收器也看到原始时间戳偏旧或超前。核查设备时钟、驱动时间源及补偿；没有硬件测量时无法仅凭时间戳断言是哪一种。')
    if age > 1 and raw_age < .5 and slow and max(backlog) > .5:
        return dict(code='compute_backlog', title='强证据：FAST-LIVO2 处理积压',
                    advice='原始输入新鲜，内部待处理时间跨度偏大且处理线程持续繁忙。对比 LIO / VIO / 回调耗时，再做降低图像负载或点量的对照测试。')
    if waiting and offset > .5 and not slow:
        return dict(code='sync_wait', title='强证据：跨传感器时间对齐受阻',
                    advice='处理线程多数时间在等数据，雷达与 IMU 有持续时差。检查原始 / 补偿后时间、输入频率及补偿参数，不要先放宽新鲜度阈值。')
    if age <= 1 and age >= -.05:
        return dict(code='fresh', title='FAST-LIVO2 状态时间正常', advice='如仍不能定位，请查看配对点云是否到达、定位阶段和匹配结果。时间正常不代表定位精度达标。')
    return dict(code='uncertain', title='输出滞后，原因证据不足或混合',
                advice='保留本次记录；可能是此前积压、上游延迟、时钟偏差或混合原因。观察队列与耗时趋势，不能只按 CPU 使用率判断。')


class FieldPerformance:
    def __init__(self, node):
        from std_msgs.msg import String
        self.node = node
        self.samples = deque(maxlen=120)
        self.raw = {}
        self.subscriptions = {}
        self.latest = None
        self.subscription = node.create_subscription(String, '/fast_livo/diagnostics', self.receive, 5)
        self.input_subscription = node.create_subscription(String, '/fast_livo/input_diagnostics', self.receive_inputs, 5)
        self.input_received = 0.
        self.input_identity = None
        self.input_counts = {}
        self.timer = node.create_timer(1., self.poll)
        self.quality = None
        self.error = ""

    def receive_inputs(self, msg):
        """Compact telemetry from the independent C++ receive thread.

        This remains live while estimation stalls. Counts that do not change
        cannot refresh a stale stream, including when a replay clock is paused.
        Legacy estimators without this topic keep the original app observers.
        """
        try:
            if len(msg.data) > 4096: return
            data = json.loads(msg.data)
            if data.get('schema') != 1 or not isinstance(data.get('instance'), str): return
            inputs = data['inputs']
            for key in ('lidar', 'imu', 'image'):
                row = inputs[key]
                if (type(row['count']) is not int or row['count'] < 0 or
                    any(type(row[k]) not in (float, int) or not math.isfinite(row[k])
                        for k in ('stamp', 'received_at', 'interval_s'))): return
            now = time.monotonic()
            ros_now = self.node.get_clock().now().nanoseconds/1e9
            if data['instance'] != self.input_identity:
                self.input_counts.clear()
                self.input_identity = data['instance']
                for key in inputs: self.raw.pop(key, None)
            for key in ('lidar', 'imu', 'image'):
                row = inputs[key]
                if row['count'] and row['count'] != self.input_counts.get(key):
                    self.raw[key] = dict(row, age_s=row['received_at']-row['stamp'],
                        received_monotonic=now-max(0.,ros_now-row['received_at']),
                        source='estimator_receive_thread')
                self.input_counts[key] = row['count']
            self.input_received = now
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            self.error = f"输入诊断解析失败：{exc}"

    def receive(self, msg):
        try:
            if len(msg.data) > 16000:
                return
            data = json.loads(msg.data)
            if data.get('schema') != 1:
                return
            if (not isinstance(data.get('instance'), str) or type(data.get('sequence')) is not int or
                    any(type(data.get(k)) not in (float,int) or not math.isfinite(data[k])
                        for k in ('stamp','state_age_s','busy_ratio','lidar_backlog_s','latest_lidar_s','latest_imu_s'))):
                return
            if any(not isinstance(data.get(k+'_topic'),str) or len(data[k+'_topic']) > 256
                   for k in ('lidar','imu','image')):
                return
            self.latest = dict(data=data, received_monotonic=time.monotonic())
        except (ValueError, TypeError, AttributeError) as exc:
            self.error = f"性能诊断解析失败：{exc}"

    def poll(self):
        from .log_store import log_store
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        if not self.latest:
            return
        d = self.latest['data']
        # Discover actual configured inputs. Serialized subscriptions inspect only Header,
        # never convert point clouds or images or perform per-point Python work.
        topics = dict(self.node.get_topic_names_and_types())
        for key in ('lidar', 'imu', 'image', 'cloud'):
            if key != 'cloud' and self.input_received and time.monotonic()-self.input_received < 2.5:
                if key in self.subscriptions:
                    self.node.destroy_subscription(self.subscriptions.pop(key)[1])
                continue
            topic = d.get(key+'_topic') if key != 'cloud' else '/fast_livo/cloud_body'
            if not topic or (key in self.subscriptions and self.subscriptions[key][0] == topic):
                continue
            types = topics.get(topic, [])
            allowed = ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/Imu', 'livox_ros_driver2/msg/CustomMsg')
            if not types or types[0] not in allowed:
                continue
            try:
                if key in self.subscriptions:
                    self.node.destroy_subscription(self.subscriptions.pop(key)[1])
                    self.raw.pop(key, None)
                subscription = self.node.create_subscription(get_message(types[0]), topic,
                    lambda payload, k=key: self.raw_message(k, payload), qos_profile_sensor_data, raw=True)
                self.subscriptions[key] = (topic, subscription)
            except (ImportError, RuntimeError, ValueError) as exc:
                self.error = f"诊断输入订阅失败：{topic}：{exc}"
        # Continue recording independent inputs even when the estimator heartbeat
        # stops. Keep its original receipt time so repeated snapshots cannot
        # revive stale telemetry or conceal a long blocked computation.
        sample = dict(self.latest, raw={k:dict(v) for k,v in self.raw.items()},
                      localizer=self.quality, app_received_at=self.node.get_clock().now().nanoseconds/1e9)
        self.samples.append(sample)
        decision = diagnose(list(self.samples), time.monotonic())
        sample = dict(sample, decision=decision)
        log_store().write('localization', 'field-performance', json.dumps(sample, ensure_ascii=False, allow_nan=False))

    def raw_message(self, key, payload):
        try:
            stamp = header_stamp(payload)
        except (ValueError, struct.error) as exc:
            self.error = f"诊断输入时间戳解析失败：{key}：{exc}"
            return
        now = time.monotonic(); received = self.node.get_clock().now().nanoseconds/1e9
        old = self.raw.get(key)
        self.raw[key] = dict(stamp=stamp, received_at=received, received_monotonic=now, age_s=received-stamp,
                             interval_s=now-old['received_monotonic'] if old else None,
                             backwards=bool(old and stamp < old['stamp']), count=(old['count']+1 if old else 1))

    def snapshot(self):
        samples = list(self.samples)
        return dict(decision=diagnose(samples, time.monotonic()), latest=samples[-1] if samples else None, error=self.error)

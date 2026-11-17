"""Bounded capture metrics shared by the recorder, offline audit and diagnostics."""
from __future__ import annotations
from dataclasses import dataclass, asdict
import json
import math
import os
from pathlib import Path
import shutil
import struct
import time


STATIC_TYPES = {"tf2_msgs/msg/TFMessage", "realsense2_camera_msgs/msg/Extrinsics"}
# Satellite data can be absent or lose its solution while local mapping keeps
# running. Phone solver/status streams have the same optional capture contract.
OPTIONAL_RTK_TOPICS = {"/rtk/fix":"sensor_msgs/msg/NavSatFix", "/rtk/observation":"std_msgs/msg/String",
    "/phone_gnss/rtk_solution":"std_msgs/msg/String", "/phone_gnss/observation":"std_msgs/msg/String",
    "/phone_gnss/status":"std_msgs/msg/String", "/phone_gnss/motion_solution":"std_msgs/msg/String",
    "/phone_gnss/absolute_observations":"std_msgs/msg/String", "/nav/phone_absolute_solution":"std_msgs/msg/String"}
STAMPLESS_DIAGNOSTIC_TOPICS = {"/phone_gnss/status":"std_msgs/msg/String", "/nav/phone_absolute_solution":"std_msgs/msg/String"}


def recording_history_depth(message_type):
    # IMU is small but arrives at 200 Hz. A uniform 64-sample history only
    # absorbs 0.32 s of image/recorder startup stalls. Keep five seconds bounded.
    return 1000 if message_type == "sensor_msgs/msg/Imu" else 64


def capture_quality_warnings(audit, protocol):
    """Review thresholds, separate from file integrity and localization accuracy."""
    limits = {"imu": .05, "lidar": .3, "rgb": .25, "depth": .15, "aligned_depth": .25}
    limits.update(protocol.get("warning_max_gap_by_role_s", {}))
    names = {"imu": "IMU", "lidar": "雷达", "rgb": "RGB", "depth": "深度", "aligned_depth": "对齐深度"}
    result = []
    for role, topic in protocol.get("roles", {}).items():
        limit = limits.get(role)
        gap = audit.get("topics", {}).get(topic, {}).get("max_gap_s", 0.)
        if isinstance(limit, (float, int)) and math.isfinite(limit) and limit > 0 and gap > limit:
            result.append(f"{names.get(role, role)} 最大数据间隔 {gap:.3f} 秒，回放建图前需检查这一段")
    return result


class FramePairs:
    """Bounded exact header-stamp join of image and CameraInfo, with edge grace."""
    def __init__(self, *, window=None):
        self.pending = {}
        self.window = window
        self.outside_images = self.outside_info = 0
        self.first = None
        self.latest = 0
        self.matched = self.missing_images = self.missing_info = 0

    def add(self, stamp, image):
        if stamp is None or stamp <= 0:
            return
        if self.window is not None and not self.window[0] <= stamp <= self.window[1]:
            self.outside_images += int(image)
            self.outside_info += int(not image)
            return
        self.first = stamp if self.first is None else min(stamp, self.first)
        self.latest = max(self.latest, stamp)
        self.pending[stamp] = self.pending.get(stamp, 0) | (1 if image else 2)
        if self.window is not None:
            # JPEG encoding can legitimately put its payload behind raw
            # CameraInfo in the bag. Count exact pairs without expiring them
            # on a fixed two-second header distance. Only unmatched tiny header
            # entries are retained, with a separate bounded cardinality.
            if self.pending[stamp] == 3:
                self.pending.pop(stamp);self.matched += 1
            if len(self.pending)>4096:
                raise ValueError('图像与相机信息配对积压超过校验窗口')
            return
        self.flush(self.latest-2_000_000_000)

    def flush(self, before, final=False):
        for stamp in list(self.pending):
            if stamp > before:
                continue
            flags = self.pending.pop(stamp)
            edge = self.window is None and (stamp < self.first+200_000_000 or (final and stamp > self.latest-200_000_000))
            if flags == 3:
                self.matched += 1
            elif not edge:
                self.missing_images += int(flags == 2)
                self.missing_info += int(flags == 1)

    def document(self):
        self.flush(self.latest, final=True)
        reference = self.matched+self.missing_images
        result = {"matched":self.matched, "missing_images":self.missing_images,
                "missing_camera_info":self.missing_info,
                "image_retention":self.matched/reference if reference else None,
                "reference":"recorded CameraInfo with identical header stamp; 0.2s edge grace"}
        if self.window is not None:
            result.update(reference='recorded CameraInfo inside explicit encoder accepted-header window; strict exact join',
                window_start_ns=self.window[0],window_end_ns=self.window[1],
                outside_images=self.outside_images,outside_camera_info=self.outside_info)
        return result


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    if os.name == "posix":
        fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)


def header_stamp(data, message_type):
    if message_type == 'std_msgs/msg/String':
        # Preserve the receiver/phone measurement UTC, not bridge publication
        # or recorder arrival time. A phone status snapshot has no such epoch.
        try:
            if len(data)<9 or len(data)>98304 or data[:2] not in (b'\x00\x01',b'\x00\x00'):return None
            length=struct.unpack_from('<I' if data[1]&1 else '>I',data,4)[0]
            if length<2 or 8+length>len(data) or data[8+length-1]!=0:return None
            document=json.loads(bytes(data[8:8+length-1]).decode('utf-8'))
            if (document.get('source') in ('phone_software_rtk', 'phone_gnss', 'phone_continuity')
                    or (document.get('schema_version')==1 and isinstance(document.get('signals'),list)
                        and isinstance(document.get('base_ecef_m'),list))):
                value, scale = document.get('utc_ms'), 1_000_000
            else:
                if document.get('schema') != 1:return None
                value, scale = document.get('stamp'), 1_000_000_000
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:return None
            return int(round(value*scale))
        except (ValueError,TypeError,AttributeError,struct.error,OverflowError):return None
    # These ROS messages start with std_msgs/Header; CDR encapsulation is 4 bytes.
    if message_type in STATIC_TYPES or len(data) < 12 or data[:2] not in (b"\x00\x01", b"\x00\x00"):
        return None
    if not message_type.startswith(("sensor_msgs/msg/", "livox_ros_driver2/msg/", "realsense2_camera_msgs/msg/Metadata")):
        return None
    endian = "<" if data[1] & 1 else ">"
    sec, nsec = struct.unpack_from(endian + "iI", data, 4)
    return sec * 1_000_000_000 + nsec if sec >= 0 and nsec < 1_000_000_000 else None


@dataclass
class StreamStats:
    count: int = 0
    bytes: int = 0
    largest_message: int = 0
    first_received: float = 0.
    last_received: float = 0.
    first_stamp: int | None = None
    last_stamp: int | None = None
    non_increasing: int = 0
    invalid_stamps: int = 0
    max_gap_s: float = 0.
    timestamp_required: bool = True

    def add(self, data, message_type, received=None):
        now = time.monotonic() if received is None else received
        stamp = header_stamp(data, message_type) if self.timestamp_required else None
        self.count += 1
        self.bytes += len(data)
        self.largest_message = max(self.largest_message, len(data))
        if self.count == 1:
            self.first_received = now
            self.first_stamp = stamp
        self.last_received = now
        if stamp is not None:
            if stamp == 0:
                self.invalid_stamps += 1
            if self.last_stamp is not None:
                gap = (stamp - self.last_stamp) / 1e9
                self.non_increasing += int(gap <= 0)
                self.max_gap_s = max(self.max_gap_s, gap)
            self.last_stamp = stamp
        elif self.timestamp_required and message_type not in STATIC_TYPES:
            self.invalid_stamps += 1

    def document(self):
        row = asdict(self)
        span = self.last_received - self.first_received
        row["observed_rate_hz"] = (self.count-1)/span if span > 0 else 0.
        return row


def resource_snapshot(directory):
    free = shutil.disk_usage(directory).free
    available = None
    memfile = Path("/proc/meminfo")
    if memfile.exists():
        fields = {line.split(":", 1)[0]: int(line.split()[1])*1024
                  for line in memfile.read_text().splitlines() if len(line.split()) >= 2}
        available = fields.get("MemAvailable")
    headroom = available
    # Inspect every ancestor: the leaf may say max while its service has a limit.
    cgroup = Path("/proc/self/cgroup")
    if cgroup.exists():
        for line in cgroup.read_text().splitlines():
            if line.startswith("0::"):
                root = Path("/sys/fs/cgroup")
                leaf = root / line[3:].lstrip("/")
                for folder in (leaf, *leaf.parents):
                    if folder != root and root not in folder.parents:
                        break
                    try:
                        limit = (folder / "memory.max").read_text().strip()
                        if limit != "max":
                            remaining = max(0, int(limit) - int((folder / "memory.current").read_text()))
                            headroom = remaining if headroom is None else min(headroom, remaining)
                    except (OSError, ValueError, TypeError):
                        pass
    temperatures = {}
    for sensor in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            name = sensor.with_name("type").read_text().strip()
            temperatures[name] = int(sensor.read_text()) / 1000.
        except (OSError, ValueError, TypeError):
            pass
    return {"unix_time": time.time(), "disk_free_bytes": free,
            "memory_available_bytes": available, "memory_headroom_bytes": headroom,
            "temperature_c": temperatures}


def resource_errors(resources, cache, split, queue_bytes, *, running=False):
    errors = []
    memory_needed = 384 * 1024**2 if running else 2*cache + queue_bytes + 768*1024**2
    disk_needed = max(1024**3, 2*cache + queue_bytes) if running else max(2*1024**3, 2*split + 2*cache)
    if resources["disk_free_bytes"] < disk_needed:
        errors.append("磁盘剩余空间不足，请更换目录或释放空间")
    available = resources.get("memory_headroom_bytes")
    if available is None:
        errors.append("无法读取可用内存，不能确认录制资源预算")
    elif available < memory_needed:
        errors.append("内存余量不足，请降低写缓存和压缩队列或停止其他任务")
    return errors


def optional_rtk_warnings(selected, stats, types, now, max_silence=2.):
    warnings = []
    for topic in selected:
        if topic not in OPTIONAL_RTK_TOPICS:
            continue
        row = stats.get(topic)
        if row is None or not row.count:
            warnings.append(f"卫星定位未同步到数据：{topic}；其余传感器继续录制")
        elif now - row.last_received > max_silence:
            warnings.append(f"卫星定位数据中断：{topic}；其余传感器继续录制")
        if row and (row.non_increasing or row.invalid_stamps):
            warnings.append(f"卫星定位时间戳异常：{topic}；该段不能作为可信定位约束")
    return warnings


def topic_issues(stats, types, now, *, preflight=False, max_silence=2.):
    errors = []
    for topic, message_type in types.items():
        row = stats[topic]
        if topic in OPTIONAL_RTK_TOPICS and OPTIONAL_RTK_TOPICS[topic] != message_type:
            errors.append(f"卫星定位话题类型不匹配：{topic}")
            continue
        # Keep no-fix, missing-UTC and repeated RTK epochs as evidence without
        # preventing RGB-D/LIO capture. Sensor-stream protection remains strict.
        if OPTIONAL_RTK_TOPICS.get(topic) == message_type:
            continue
        static = message_type in STATIC_TYPES
        if row.count < (1 if static else 2):
            errors.append(f"尚未收到有效数据：{topic}")
        elif not static and now - row.last_received > max_silence:
            errors.append(f"话题断流：{topic}")
        if not static and row.invalid_stamps:
            errors.append(f"话题时间戳无效：{topic}")
        if not static and row.non_increasing:
            errors.append(f"话题时间戳重复或倒退：{topic}")
    return errors


def recording_health(stats, types, now, resources, cache, split, queue_bytes, *, max_silence=2.):
    """Observer delays cannot prove that the separate rosbag writer lost input.

    Keep startup checks strict and audit the actual saved messages after closing.
    During capture, report observer anomalies and keep recording usable streams;
    only resource/write failures should request an automatic flush.
    """
    return {
        "errors": resource_errors(resources, cache, split, queue_bytes, running=True),
        "warnings": topic_issues(stats, types, now, max_silence=max_silence),
    }


def optional_capture_topics(protocol):
    """Depth can augment RGB/LiDAR/IMU without being a capture prerequisite."""
    optional = set(OPTIONAL_RTK_TOPICS)
    required = set(protocol.get("required_roles", []))
    for role in ("depth", "aligned_depth"):
        name = protocol.get("roles", {}).get(role)
        if name and role not in required:
            base = name.rsplit("/", 1)[0]
            optional.update((name, base+"/camera_info", base+"/metadata"))
    return optional


def capture_preflight(stats, types, selected, protocol, now):
    errors, warnings = [], []
    optional = optional_capture_topics(protocol)
    for name in selected:
        issues = ([f"未发现话题：{name}"] if name not in types else
                  topic_issues({name: stats[name]}, {name: types[name]}, now, preflight=True,
                               max_silence=protocol.get("max_silence_s", 2.)))
        (warnings if name in optional else errors).extend(issues)
    return {"errors": errors, "warnings": warnings}

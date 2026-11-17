"""ROS bag discovery, metadata and process lifecycle support for the Qt app."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import time
from typing import Dict, Optional, Sequence, Tuple

import yaml
from PyQt5.QtCore import QObject, pyqtSignal

from ..ros_launch_process import RosLaunchProcess
from .image_archive import archive_routes

try:
    from rosbag2_interfaces.srv import Pause, Resume
except ImportError:  # Keep UI-only tests usable before rosbag2 is installed.
    Pause = None
    Resume = None


DERIVED_TOPIC_PREFIXES = (
    "/aft_mapped_to_init",
    "/dlio/",
    "/fast_livo/cloud_body",
    "/fast_livo/cloud_world",
    "/fast_livo/map",
    "/map",
    "/odom",
    "/plan",
    "/global_plan",
    "/scan",
    "/cmd_vel",
)

# 录制默认值：zstd 逐消息压缩、1 GB 分片、128 MiB 缓存；按实测消息大小限制压缩预算。
# 默认逐消息: 实测文件级压缩在 ~160 MB/s 码率下会积压待压缩分片, 停机要几分钟。
DEFAULT_COMPRESSION_MODE = "message"
DEFAULT_COMPRESSION_QUEUE_MESSAGE = 128   # Guard additionally enforces a 64 MiB byte budget.
DEFAULT_COMPRESSION_QUEUE_FILE = 2        # 文件级: 队列单位是"1GB 分片", 不能大
DEFAULT_COMPRESSION_THREADS_MESSAGE = 1  # Preserve record order across split boundaries.
DEFAULT_COMPRESSION_THREADS_FILE = 1
DEFAULT_COMPRESSION_FORMAT = "zstd"
DEFAULT_MAX_CACHE_SIZE = 128 * 1024**2
DEFAULT_MAX_BAG_SIZE = 1_000_000_000

RAW_SENSOR_TYPES = {
    "livox_ros_driver2/msg/CustomMsg",
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/NavSatFix",
    # RealSense 的深度->彩色外参: 体积很小, 但离线做 RGB-D / 点云融合时需要
    "realsense2_camera_msgs/msg/Extrinsics",
    "realsense2_camera_msgs/msg/Metadata",
}

# 默认不勾的话题: CustomMsg 与 PointCloud2 是同一份雷达数据的两种封装,
# 两个都录会让雷达部分体积接近翻倍, 因此默认只勾 FAST-LIVO2 直接使用的 CustomMsg。
DEFAULT_SKIP_TOPICS = ("/livox/lidar/pointcloud",)

# 回放建图/定位需要静态外参(相机-雷达杆臂、base_link->laser), 且 /tf_static 极小,
# 所以它虽然不在上面的原始传感器类型里, 也默认勾选。
DEFAULT_EXTRA_TOPICS = ("/tf_static", "/rtk/observation", "/rtk/fix",
                        "/phone_gnss/rtk_solution", "/phone_gnss/observation", "/phone_gnss/status", "/phone_gnss/motion_solution",
                        "/phone_gnss/absolute_observations", "/nav/phone_absolute_solution")

@dataclass(frozen=True)
class BagTopic:
    name: str
    message_type: str
    message_count: int = 0


BASE_SENSOR_TOPICS = (
    BagTopic("/livox/lidar", "livox_ros_driver2/msg/CustomMsg"),
    BagTopic("/livox/lidar/pointcloud", "sensor_msgs/msg/PointCloud2"),
    BagTopic("/livox/imu", "sensor_msgs/msg/Imu"),
    BagTopic("/camera/camera/color/image_raw", "sensor_msgs/msg/Image"),
    BagTopic("/camera/camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
    BagTopic("/camera/camera/depth/image_rect_raw", "sensor_msgs/msg/Image"),
    BagTopic("/camera/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo"),
    BagTopic(
        "/camera/camera/aligned_depth_to_color/image_raw",
        "sensor_msgs/msg/Image"),
    BagTopic(
        "/camera/camera/aligned_depth_to_color/camera_info",
        "sensor_msgs/msg/CameraInfo"),
    BagTopic("/camera/camera/gyro/sample", "sensor_msgs/msg/Imu"),
    BagTopic("/camera/camera/accel/sample", "sensor_msgs/msg/Imu"),
    BagTopic("/camera/camera/imu", "sensor_msgs/msg/Imu"),
    BagTopic("/camera/camera/depth/color/points", "sensor_msgs/msg/PointCloud2"),
)


@dataclass(frozen=True)
class BagMetadata:
    path: Path
    storage_id: str
    duration_ns: int
    starting_time_ns: int
    message_count: int
    topics: Tuple[BagTopic, ...]
    recording_profile: str = 'full'

    @property
    def duration_seconds(self) -> float:
        return self.duration_ns / 1_000_000_000.0


@dataclass(frozen=True)
class PlaybackRequest:
    bag_path: Path
    topics: Tuple[str, ...]
    remaps: Tuple[Tuple[str, str], ...] = ()
    rate: float = 1.0
    clock_hz: float = 50.0
    loop: bool = False
    start_paused: bool = False

    @property
    def effective_topics(self) -> Dict[str, str]:
        remap = dict(self.remaps)
        return {topic: remap.get(topic, topic) for topic in self.topics}


def metadata_file(path: Path | str) -> Path:
    source = Path(path).expanduser()
    if source.is_file() and source.name == "metadata.yaml":
        return source
    if source.is_file():
        return source.parent / "metadata.yaml"
    return source / "metadata.yaml"


def read_bag_metadata(path: Path | str) -> BagMetadata:
    """Read ROS 2 bag metadata without opening the potentially large data file."""
    metadata_path = metadata_file(path)
    if not metadata_path.is_file():
        raise ValueError(f"没有找到 rosbag metadata.yaml：{metadata_path}")
    try:
        document = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"读取 rosbag 元数据失败：{exc}") from exc
    info = document.get("rosbag2_bagfile_information", document)
    if not isinstance(info, dict):
        raise ValueError("rosbag metadata.yaml 格式无效")

    topics = []
    for row in info.get("topics_with_message_count", ()) or ():
        topic = row.get("topic_metadata", {}) if isinstance(row, dict) else {}
        name = str(topic.get("name", "")).strip()
        message_type = str(topic.get("type", "")).strip()
        if name and message_type:
            topics.append(BagTopic(
                name=name,
                message_type=message_type,
                message_count=int(row.get("message_count", 0) or 0),
            ))
    # Present the original sensor contract to the UI. Playback restores these
    # images; the on-disk metadata deliberately retains CompressedImage.
    routes = {r['stored_topic']: r for r in archive_routes(metadata_path.parent)}
    topics = [BagTopic(routes[t.name]['source_topic'], 'sensor_msgs/msg/Image', t.message_count)
              if t.name in routes and t.message_type == 'sensor_msgs/msg/CompressedImage' else t
              for t in topics]
    topics.sort(key=lambda value: value.name)
    manifest_path = metadata_path.parent / 'nav_manifest.json'
    profile = (json.loads(manifest_path.read_text(encoding='utf-8')).get('recording_profile', 'full')
               if manifest_path.is_file() else 'full')
    if profile not in ('full', 'keyframes'):
        raise ValueError('不支持的采集数据格式')

    duration = info.get("duration", {}) or {}
    starting_time = info.get("starting_time", {}) or {}
    return BagMetadata(
        path=metadata_path.parent.resolve(),
        storage_id=str(info.get("storage_identifier", "") or ""),
        duration_ns=int(duration.get("nanoseconds", 0) or 0),
        starting_time_ns=int(
            starting_time.get("nanoseconds_since_epoch", 0) or 0),
        message_count=int(info.get("message_count", 0) or 0),
        topics=tuple(topics),
        recording_profile=profile,
    )


def previous_capture_warning():
    receipt = Path.home()/".local/state/nav/last_capture.json"
    try:
        value = json.loads(receipt.read_text(encoding="utf-8"))
        if value.get('state') == 'stored':
            # An old offline task must never overwrite the receipt belonging to
            # a new live capture. Resolve its final quality from its own file.
            manifest = Path(value.get('path', ''))/'nav_manifest.json'
            if manifest.is_file():
                state = json.loads(manifest.read_text(encoding='utf-8')).get('state')
                if state == 'complete':return ''
                if state == 'incomplete':
                    return f"上次录制有质量提示，请检查数据：{value.get('path', '')}"
            return f"上次采集数据已保存，等待后台校验：{value.get('path', '')}"
        if value.get("state") not in ("complete", "cancelled"):
            return f"上次录制未通过验收，请检查数据：{value.get('path', '')}"
    except (OSError, ValueError, TypeError):
        pass
    return ""


def default_record_selected(topic: BagTopic) -> bool:
    """Select raw Livox/RealSense inputs, not every diagnostic topic.

    默认勾选 = FAST-LIVO2 建图 + 深度录制所需的原始输入:
      雷达 CustomMsg、IMU、彩色图/CameraInfo、深度图/CameraInfo、
      对齐深度图/CameraInfo、深度->彩色外参、/tf_static。
    """
    if topic.name in DEFAULT_SKIP_TOPICS:
        return False
    if topic.name in DEFAULT_EXTRA_TOPICS:
        return True
    # image_transport advertises lazy JPEG/depth/theora transports even when
    # nobody needs them. Subscribing activates the encoder on the camera's
    # frame callback; JPEG cannot represent its 16UC1 depth and may publish
    # empty messages. Record original images once and compress the bag on disk.
    # Keep CompressedImage in RAW_SENSOR_TYPES for explicit legacy playback.
    if topic.message_type == "sensor_msgs/msg/CompressedImage":
        return False
    if topic.message_type not in RAW_SENSOR_TYPES:
        return False
    return topic.name.startswith(("/livox/", "/camera/"))


def safe_algorithm_topic(topic: BagTopic) -> bool:
    """Return whether a recorded topic is safe as an algorithm input."""
    if topic.name == "/tf_static":
        return True
    if topic.name in ("/rtk/observation", "/phone_gnss/rtk_solution", "/phone_gnss/motion_solution", "/phone_gnss/absolute_observations") and topic.message_type == "std_msgs/msg/String":
        return True
    if topic.name == "/tf" or topic.name.startswith(DERIVED_TOPIC_PREFIXES):
        return False
    return topic.message_type in RAW_SENSOR_TYPES


def build_record_arguments(
    output_path: Path | str,
    topics: Sequence[str],
    storage_id: str = "sqlite3",
    compression_mode: str = DEFAULT_COMPRESSION_MODE,
    compression_format: str = DEFAULT_COMPRESSION_FORMAT,
    max_cache_size: int = DEFAULT_MAX_CACHE_SIZE,
    max_bag_size: int = DEFAULT_MAX_BAG_SIZE,
    compression_queue_size: int = 0,
    compression_threads: int = 0,
) -> list[str]:
    """拼出 ros2 bag record 命令。

    默认带 zstd 逐消息压缩、128 MiB 写缓存、单个 db3 分片上限 1GB:
    深度/彩色图像不做压缩时 10 分钟就是几十 GB, 缓存太小又会在高码率下丢包。
    传 compression_mode="none" / max_cache_size=0 / max_bag_size=0 可逐项关掉。
    """
    selected = [str(topic).strip() for topic in topics if str(topic).strip()]
    if not selected:
        raise ValueError("至少选择一个录制话题")
    arguments = [
        "bag", "record", "--output", str(Path(output_path).expanduser()),
        "--storage", storage_id or "sqlite3",
    ]
    mode = str(compression_mode or "none").strip().lower()
    if mode and mode != "none":
        arguments += [
            "--compression-mode", mode,
            "--compression-format",
            str(compression_format or "zstd").strip().lower() or "zstd",
        ]
        # rosbag2 的 --compression-queue-size 默认是 1(条/片), 槽位占满时后来的
        # 消息会被直接丢弃 —— 高频话题(IMU 200Hz)受损最重。这里给逐消息一个足够
        # 有界队列，再由 record_guard 按最大消息大小收紧；文件级的队列单位是分片。
        queue = int(compression_queue_size) if compression_queue_size else (
            DEFAULT_COMPRESSION_QUEUE_MESSAGE if mode == "message"
            else DEFAULT_COMPRESSION_QUEUE_FILE)
        threads = int(compression_threads) if compression_threads else (
            DEFAULT_COMPRESSION_THREADS_MESSAGE if mode == "message"
            else DEFAULT_COMPRESSION_THREADS_FILE)
        arguments += [
            "--compression-queue-size", str(queue),
            "--compression-threads", str(threads),
        ]
    try:
        cache_size = int(max_cache_size)
    except (TypeError, ValueError):
        cache_size = 0
    if cache_size > 0:
        arguments += ["--max-cache-size", str(cache_size)]
    try:
        bag_size = int(max_bag_size)
    except (TypeError, ValueError):
        bag_size = 0
    if bag_size > 0:
        arguments += ["--max-bag-size", str(bag_size)]
    return [*arguments, *selected]


def build_play_arguments(request: PlaybackRequest) -> list[str]:
    if request.rate <= 0:
        raise ValueError("回放倍率必须大于 0")
    if request.clock_hz <= 0:
        raise ValueError("/clock 频率必须大于 0")
    routes = [dict(r, output_topic=dict(request.remaps).get(r['source_topic'], r['source_topic']))
              for r in archive_routes(metadata_file(request.bag_path).parent)
              if not request.topics or r['source_topic'] in request.topics]
    stored = {r['source_topic']: r['stored_topic'] for r in routes}
    arguments = [
        "bag", "play", str(Path(request.bag_path).expanduser()),
        "--clock", f"{request.clock_hz:g}",
        "--rate", f"{request.rate:g}",
        "--disable-keyboard-controls",
    ]
    if request.start_paused:
        arguments.append("--start-paused")
    if request.loop:
        arguments.append("--loop")
    if request.topics:
        arguments.extend(["--topics", *(stored.get(t, t) for t in request.topics)])
    remaps = [
        f"{source}:={target}" for source, target in request.remaps
        if source and target and source != target and source not in stored
    ]
    if remaps:
        arguments.extend(["--remap", *remaps])
    if routes:
        return ['run', 'nav_app', 'image_archive_play', '--routes', json.dumps(routes), '--', *arguments]
    return arguments


class RosbagController(QObject):
    """Own recorder/player processes and the player's pause services."""

    log_received = pyqtSignal(str, str)
    state_changed = pyqtSignal(str, str)
    record_finished = pyqtSignal(object)

    def __init__(self, node=None, parent=None) -> None:
        super().__init__(parent)
        self.node = node
        # 停止录制时 rosbag2 还要把缓存写盘、把排队的分片压缩完并写出
        # metadata.yaml。高码率 + zstd 文件级压缩下这一步可能要几分钟(实测 10 分钟
        # 47GB 的包, 停机瞬间还有 36 个分片排着), 12s 就 SIGKILL 会让整包作废,
        # 所以这里给足 15 分钟, 只有真正卡死才会被终止。
        self.record_process = RosLaunchProcess(
            "rosbag 录制", self, graceful_timeout_ms=900_000,
            terminate_timeout_ms=30_000)
        self.play_process = RosLaunchProcess("rosbag 回放", self, graceful_timeout_ms=15_000)
        self.record_process.log_received.connect(self._record_log)
        self.play_process.log_received.connect(
            lambda text: self.log_received.emit("play", text))
        self.record_process.state_changed.connect(self._record_state)
        self.play_process.state_changed.connect(
            lambda state: self.state_changed.emit("play", state))
        self._record_log_tail = ""
        self.last_record_error = ""
        self.last_record_state = ""
        self.last_record_progress = ""
        self.last_record_warning = ""
        self.last_record_warnings = []
        self.admission_guard = None
        self.stop_guard = None
        self._record_output: Optional[Path] = None
        self._record_topics: Tuple[str, ...] = ()
        self._record_arguments: Tuple[str, ...] = ()
        self._pause_client = None
        self._resume_client = None
        if node is not None and hasattr(node, "create_client"):
            try:
                if Pause is not None:
                    self._pause_client = node.create_client(
                        Pause, "/rosbag2_player/pause")
                if Resume is not None:
                    self._resume_client = node.create_client(
                        Resume, "/rosbag2_player/resume")
            except (RuntimeError, TypeError) as exc:
                from .log_store import log_store
                log_store().write("recording", "services", f"暂停/继续服务初始化失败：{exc}")
                self._pause_client = None
                self._resume_client = None

    @property
    def recording(self) -> bool:
        return self.record_process.active

    @property
    def record_output(self):
        return self._record_output

    def capture_topics(self, include_depth=True, include_satellite=False):
        """Use the guard's protocol, including depth, for the one-button capture."""
        configured = os.environ.get("NAV_CAPTURE_CONFIG")
        if configured:
            source = Path(configured)
        else:
            from ament_index_python.packages import get_package_share_directory
            source = Path(get_package_share_directory("nav_app")) / "config/capture.yaml"
        try:
            protocol = yaml.safe_load(source.read_text(encoding="utf-8"))
        except yaml.YAMLError as exc:
            raise ValueError("采集配置格式有误，请检查 capture.yaml") from exc
        roles = protocol["roles"]
        if not include_depth and set(protocol["required_roles"]) & {"depth", "aligned_depth"}:
            raise ValueError("当前采集规范要求深度，请在录包设置中开启深度")
        deferred_depth = protocol.get('depth_alignment') == 'offline'
        depth_roles = (("depth",) if deferred_depth else ("depth", "aligned_depth")) if include_depth else ()
        required = [roles[key] for key in dict.fromkeys([*protocol["required_roles"], *depth_roles])]
        required += [roles[key].rsplit("/", 1)[0] + "/camera_info" for key in depth_roles]
        if include_depth and deferred_depth:
            required.append(roles.get('depth_to_color', '/camera/camera/extrinsics/depth_to_color'))
        # The ROS graph also contains subscription-only topics. In particular,
        # the APP subscribes to RTK even when no receiver is running. Only add
        # optional discovered streams with a publisher; explicitly required
        # camera/LiDAR/IMU streams still go through the recorder's data checks.
        available = [row.name for row in self.available_topics()
                     if default_record_selected(row) and self.node is not None
                     and self.node.get_publishers_info_by_topic(row.name)]
        if deferred_depth:
            # A legacy viewer may advertise aligned depth. Do not subscribe
            # through automatic discovery and reactivate camera-side work.
            prefix = roles['aligned_depth'].rsplit('/', 1)[0] + '/'
            available = [name for name in available if not name.startswith(prefix)]
        if include_satellite:
            if '/phone_gnss/rtk_solution' in available and '/rtk/observation' not in available:
                required.append('/phone_gnss/rtk_solution')
            else:
                required += ['/rtk/observation','/rtk/fix']
        if not include_depth:
            prefixes = [roles[key].rsplit("/", 1)[0] + "/" for key in ("depth", "aligned_depth") if key in roles]
            available = [name for name in available if not any(name.startswith(p) for p in prefixes)]
        selected = [*required, *available]
        if not include_satellite:
            selected = [name for name in selected if name not in ('/rtk/fix', '/rtk/observation')]
        return tuple(dict.fromkeys(selected))

    @property
    def playing(self) -> bool:
        return self.play_process.active

    def available_topics(self) -> Tuple[BagTopic, ...]:
        if self.node is None or not hasattr(
                self.node, "get_topic_names_and_types"):
            return ()
        rows = []
        for name, message_types in self.node.get_topic_names_and_types():
            for message_type in message_types:
                rows.append(BagTopic(str(name), str(message_type)))
        return tuple(sorted(rows, key=lambda value: (
            value.name, value.message_type)))

    def start_recording(
        self, output_parent: Path | str, bag_name: str,
        topics: Sequence[str], storage_id: str = "sqlite3",
        compression_mode: str = DEFAULT_COMPRESSION_MODE,
        compression_format: str = DEFAULT_COMPRESSION_FORMAT,
        max_cache_size: int = DEFAULT_MAX_CACHE_SIZE,
        max_bag_size: int = DEFAULT_MAX_BAG_SIZE,
        unique_if_exists: bool = False,
        compression_queue_size: int = 0,
        compression_threads: int = 0,
        require_mapping_parameters: bool = False,
        image_storage: str = 'raw',
        recording_mode: str = 'full',
        keyframe_interval_s: float = 1.0,
        keyframe_selection: str = 'quality',
    ) -> Tuple[bool, str]:
        guard = getattr(self, 'start_guard', None)
        if guard:
            allowed, reason = guard()
            if not allowed:
                return False, reason
        if self.admission_guard is not None:
            allowed, reason = self.admission_guard('recording')
            if not allowed:
                return False, reason
        if recording_mode != 'full':
            return False, '仅支持完整传感器录包' 
        if image_storage not in ('raw', 'jpeg95'):
            return False, '图像保存格式无效'
        if getattr(self, 'library_delete_in_progress', False):
            return False, "正在删除录包，请完成后再录制"
        if self.playing:
            return False, "正在回放 rosbag，不能同时开始录制"
        if self.recording:
            return False, "rosbag 已经在录制"
        name = str(bag_name).strip()
        if not name or Path(name).name != name or name in (".", ".."):
            return False, "请输入不含路径分隔符的 rosbag 名称"
        raw_parent = str(output_parent).strip()
        if not raw_parent:
            return False, "请选择 rosbag 保存目录"
        parent_path = Path(raw_parent).expanduser()
        try:
            parent_path.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return False, f"无法创建录制目录：{exc}"
        if not parent_path.is_dir():
            return False, f"录制路径不是目录：{parent_path}"
        target = parent_path / name
        if target.exists():
            if not unique_if_exists:
                return False, (
                    f"目标已存在，不会覆盖：{target}\n"
                    "（勾选“同名目录自动加时间戳”可改为自动改名）")
            target = parent_path / (
                f"{name}_{time.strftime('%Y%m%d_%H%M%S')}")
            if target.exists():
                return False, f"目标已存在，不会覆盖：{target}"
        try:
            arguments = build_record_arguments(
                target, topics, storage_id,
                compression_mode=compression_mode,
                compression_format=compression_format,
                max_cache_size=max_cache_size,
                max_bag_size=max_bag_size,
                compression_queue_size=compression_queue_size,
                compression_threads=compression_threads)
        except ValueError as exc:
            return False, str(exc)
        self.last_record_error = ""
        self.last_record_state = "checking"
        self.last_record_progress = ""
        self.last_record_warning = ""
        self.last_record_warnings = []
        self._record_output = target
        self._record_topics = tuple(topics)
        self._record_arguments = tuple(arguments)
        self._record_log_tail = ""
        guard_options = ['--require-mapping-parameters'] if require_mapping_parameters else []
        guard_options += ['--image-storage', image_storage]
        if not self.record_process.start_launch(["run", "nav_app", "reliable_record", *guard_options, "--", *arguments]):
            self._record_output = None
            return False, "录制检查进程未能启动"
        return True, f"正在检查数据和存储空间：{target}"

    def stop_recording(self) -> None:
        if self.stop_guard is not None:
            allowed, reason = self.stop_guard('recording')
            if not allowed:
                self.log_received.emit('record', reason+'\n')
                return
        self.record_process.stop_launch()

    def start_playback(self, request: PlaybackRequest) -> Tuple[bool, str]:
        guard = getattr(self, 'start_guard', None)
        if guard:
            allowed, reason = guard()
            if not allowed:
                return False, reason
        if self.admission_guard is not None:
            allowed, reason = self.admission_guard('playback')
            if not allowed:
                return False, reason
        if getattr(self, 'library_delete_in_progress', False):
            return False, "正在删除录包，请完成后再回放"
        if self.recording:
            return False, "正在录制 rosbag，不能同时开始回放"
        if self.playing:
            return False, "rosbag 已经在回放"
        try:
            metadata = read_bag_metadata(request.bag_path)
            arguments = build_play_arguments(request)
        except ValueError as exc:
            return False, str(exc)
        if not request.topics:
            return False, "至少选择一个回放话题"
        known = {topic.name for topic in metadata.topics}
        missing = [topic for topic in request.topics if topic not in known]
        if missing:
            return False, "bag 中不存在这些话题：" + ", ".join(missing)
        if not self.play_process.start_launch(arguments):
            return False, "rosbag 回放进程未能启动"
        return True, f"正在回放：{metadata.path}"

    def stop_playback(self) -> None:
        self.play_process.stop_launch()

    def pause_playback(self) -> Tuple[bool, str]:
        return self._call_player_service(self._pause_client, Pause, "暂停")

    def resume_playback(self) -> Tuple[bool, str]:
        return self._call_player_service(self._resume_client, Resume, "继续")

    @staticmethod
    def _call_player_service(client, service_type, action: str):
        if client is None or service_type is None:
            return False, f"当前环境不支持 rosbag {action}服务"
        if not client.service_is_ready():
            return False, f"rosbag {action}服务尚未就绪"
        try:
            client.call_async(service_type.Request())
        except RuntimeError as exc:
            return False, f"请求 rosbag {action}失败：{exc}"
        return True, f"已请求{action}回放"

    def shutdown(self) -> None:
        self.record_process.shutdown()
        self.play_process.shutdown()

    def _record_log(self, text: str) -> None:
        self.log_received.emit("record", text)
        self._record_log_tail += text
        lines = self._record_log_tail.split("\n")
        self._record_log_tail = lines.pop()[-4096:]
        for line in lines:
            if line.startswith("NAV_RECORD_STATE "):
                self.last_record_state = line.split(" ", 1)[1].strip()
                # Older keyframe recorders used the manifest's "recording"
                # spelling on this wire; all recording modes expose running.
                if self.last_record_state=='recording':self.last_record_state='running'
                self.state_changed.emit("record", self.last_record_state)
            elif line.startswith("NAV_RECORD_PROGRESS "):
                self.last_record_progress = line.split(" ", 1)[1].strip()
            elif line.startswith("NAV_RECORD_WARNING "):
                self.last_record_warning = line.split(" ", 1)[1].strip()

    def _record_state(self, state: str) -> None:
        if state not in ("stopped", "error"):
            self.state_changed.emit("record", "checking" if state == "running" else state)
            return
        if self._record_output is None or self.record_process.active:
            return
        output = self._record_output
        self._record_output = None
        self._record_topics = ()
        self._record_arguments = ()
        try:
            manifest = json.loads((output / "nav_manifest.json").read_text(encoding="utf-8"))
            if manifest.get("state") == "cancelled":
                self.last_record_state = "cancelled"
                self.state_changed.emit("record", "cancelled")
                self.record_finished.emit(None)
                return
            if manifest.get('state') == 'stored' and manifest.get('storage_flush_confirmed'):
                metadata = read_bag_metadata(output)
                self.last_record_state = 'stored'
                self.last_record_warnings = list(manifest.get('storage_errors', []))
                self.state_changed.emit('record', 'stored')
                self.record_finished.emit(metadata)
                return
            if manifest.get("state") != "complete":
                raise ValueError("；".join(manifest.get("errors", [])) or "录制未通过完整性检查")
            metadata = read_bag_metadata(output)
            from .capture_quality import capture_quality_warnings
            if metadata.recording_profile == 'keyframes':
                self.last_record_warnings = list(manifest.get('audit', {}).get('warnings', []))
            else:
                self.last_record_warnings = capture_quality_warnings(manifest.get("audit", {}), manifest.get("capture_protocol", {}))
                if manifest.get("observation_warnings"):
                    self.last_record_warnings.append("采集中出现过监测异常；已保存的数据通过文件校验")
            self.last_record_state = "complete"
            self.state_changed.emit("record", "complete")
            self.record_finished.emit(metadata)
        except (OSError, ValueError) as exc:
            self.last_record_state = "incomplete"
            self.last_record_error = str(exc)
            self.log_received.emit("record", f"数据未通过验收：{exc}\n")
            self.state_changed.emit("record", "incomplete")
            self.record_finished.emit(None)

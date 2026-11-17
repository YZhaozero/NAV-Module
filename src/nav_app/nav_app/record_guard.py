"""Supervise rosbag2 without blocking the Qt/ROS UI thread.

SIGINT requests a flush; the guard persists a durable close receipt and exits.
Whole-bag readback is a separate, cancellable offline task.
JPEG capture isolates the recorder's signals until the image FIFO has drained.
"""
from __future__ import annotations
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import yaml

from .features.capture_quality import (
    StreamStats, STATIC_TYPES, atomic_json, resource_snapshot, resource_errors,
    topic_issues, optional_rtk_warnings, recording_history_depth,
    recording_health, capture_preflight, header_stamp, OPTIONAL_RTK_TOPICS,
    STAMPLESS_DIAGNOSTIC_TOPICS)
from .features.bag_cache import BagCacheWriter
from .features.capture_evidence import bound_record_buffers, snapshot_evidence, recorder_fault
from .features.image_archive import ARCHIVE_MAX_BYTES, ARCHIVE_MAX_FRAMES


class RecorderStop:
    """One stop signal per recorder, including the normal whole-group stop.

    POSIX launchers signal the entire owned group. Its recorder has already
    received that signal; forwarding it again can interrupt CLI cleanup.
    Internal resource/write errors still initiate SIGINT themselves.
    """
    def __init__(self, event, child_in_group=True):
        self.event = event
        self.child_in_group = child_in_group
        self.group_signal_received = False
        self.requested = False

    def group_signal(self, _signum, _frame):
        self.group_signal_received = True
        self.event.set()

    def request(self, child):
        if self.requested: return
        self.requested = True
        if child.poll() is None and (not self.group_signal_received or not self.child_in_group):
            child.send_signal(signal.SIGINT)


def state(value):
    print("NAV_RECORD_STATE " + value, flush=True)


def option(args, name, default=None):
    return args[args.index(name)+1] if name in args else default


def release_image_observers(node, subscriptions, types):
    """Keep image preflight evidence without a second full-payload DDS reader.

    rosbag remains subscribed at the original QoS/resolution/rate. Small camera
    metadata, IMU, LiDAR and diagnostics remain continuously observed. Metadata
    is not a substitute for image delivery: only audit_bag counts saved frames.
    Return exactly the subscriptions released so failed removals remain watched.
    """
    released = set()
    for name, subscription in list(subscriptions.items()):
        if types[name] not in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
            continue
        if node.destroy_subscription(subscription):
            del subscriptions[name]
            released.add(name)
    return released


def observed_topic_report(stats, baseline_counts, preflight_only):
    return {name: {**row.document(),
                  "scope": "preflight_only" if name in preflight_only else "continuous_observer",
                  "messages_since_start": None if name in preflight_only else row.count-baseline_counts[name]}
            for name, row in stats.items()}


def snapshot_parameters(node, names, *, service_timeout=2., call_timeout=1., executor=None):
    from rcl_interfaces.srv import ListParameters, GetParameters
    import rclpy
    from rosidl_runtime_py.convert import message_to_ordereddict
    result = {}
    clients = []
    def discover(client):
        # This node also consumes reliable sensor streams. Blocking in
        # wait_for_service while an optional localizer is absent fills its
        # small DDS history and backpressures the camera during capture start.
        deadline = time.monotonic() + service_timeout
        while not client.service_is_ready():
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                return False
            if executor is None:
                rclpy.spin_once(node, timeout_sec=min(.05, remaining))
            else:
                executor.spin_once(timeout_sec=min(.05, remaining))
        return True
    try:
        for name in names:
            listing = node.create_client(ListParameters, name + "/list_parameters")
            getter = node.create_client(GetParameters, name + "/get_parameters")
            clients += [listing, getter]
            # A newly created client has not necessarily discovered an already
            # running server. Give DDS discovery a bounded opportunity to finish.
            if not discover(listing) or not discover(getter):
                result[name] = {"available": False, "reason": "parameter service unavailable"}
                continue
            future = listing.call_async(ListParameters.Request())
            if executor is None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout)
            else:
                executor.spin_until_future_complete(future, timeout_sec=call_timeout)
            if not future.done() or future.result() is None:
                result[name] = {"available": False, "reason": "list timed out"}
                continue
            # Explicit sensor/algorithm nodes only; exclude credentials even if added later.
            keys = [k for k in future.result().result.names
                    if not any(word in k.lower() for word in ("password", "secret", "token", "serial_no"))]
            request = GetParameters.Request(names=keys)
            future = getter.call_async(request)
            if executor is None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=call_timeout)
            else:
                executor.spin_until_future_complete(future, timeout_sec=call_timeout)
            if not future.done() or future.result() is None:
                result[name] = {"available": False, "reason": "get timed out"}
                continue
            result[name] = {"available": True, "parameters": {
                key: message_to_ordereddict(value) for key, value in zip(keys, future.result().values)}}
    finally:
        for client in clients:
            node.destroy_client(client)
    return result


def snapshot_config_files(parameters):
    files = {}
    def scrub(value):
        if isinstance(value, dict):
            return {k: ("<redacted>" if any(t in str(k).lower() for t in ("password", "secret", "token")) else scrub(v)) for k,v in value.items()}
        if isinstance(value, list):
            return [scrub(v) for v in value]
        return value
    for node, snapshot in parameters.items():
        for name, value in snapshot.get("parameters", {}).items():
            if name not in ("config_path", "user_config_path", "params_file", "camera_params_file"):
                continue
            filename = value.get("string_value", "")
            if not filename:
                continue
            path = Path(filename).expanduser()
            try:
                if path.suffix.lower() not in (".yaml", ".yml", ".json") or path.stat().st_size > 1024**2:
                    raise ValueError("not a bounded sensor configuration")
                raw = path.read_bytes()
                files[node+":"+name] = {"path":str(path), "sha256":hashlib.sha256(raw).hexdigest(),
                    "configuration":scrub(yaml.safe_load(raw)), "credentials_redacted":True}
            except (OSError, ValueError, yaml.YAMLError) as exc:
                files[node+":"+name] = {"path":str(path), "unavailable":str(exc)}
    return files


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ament_index_python.packages import get_package_share_directory
    import rosbag2_py

    args = sys.argv[1:]
    require_mapping, image_storage = False, 'raw'
    recording_mode, keyframe_interval, keyframe_selection = 'full', 1., 'quality'
    while args and args[0] != '--':
        if args[0] == '--require-mapping-parameters':
            require_mapping = True; args = args[1:]
        elif args[0] == '--image-storage' and len(args) > 1:
            image_storage = args[1]; args = args[2:]
        elif args[0] == '--recording-mode' and len(args) > 1:
            recording_mode = args[1]; args = args[2:]
        elif args[0] == '--keyframe-interval' and len(args) > 1:
            try:keyframe_interval = float(args[1])
            except ValueError:return 2
            args = args[2:]
        elif args[0] == '--keyframe-selection' and len(args) > 1:
            keyframe_selection = args[1]; args = args[2:]
        else:
            break
    if image_storage not in ('raw', 'jpeg95'):
        print('图像保存格式无效', flush=True)
        return 2
    if args and args[0] == "--":
        args = args[1:]
    if args[:2] != ["bag", "record"] or "--output" not in args:
        print("Expected bag record arguments", flush=True)
        return 2
    if recording_mode != 'full':
        print('录制模式无效', flush=True)
        return 2
    output = Path(option(args, "--output")).expanduser()
    if output.exists():
        print("目标目录已存在，拒绝覆盖", flush=True)
        return 2
    import fcntl
    state_dir = Path.home()/".local/state/nav"
    state_dir.mkdir(parents=True, exist_ok=True)
    lock = (state_dir/"capture.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("另一项录制仍在运行或校验，请等待结束", flush=True)
        return 2
    receipt = state_dir/"last_capture.json"
    atomic_json(receipt, {"path": str(output), "state": "checking", "unix_time": time.time()})
    def report_state(value):
        # Keep recovery/diagnostic readers in step with the UI's stdout events.
        # A ten-minute recording previously left this receipt at "checking".
        atomic_json(receipt, {"path": str(output), "state": value, "unix_time": time.time()})
        state(value)
    stop = threading.Event()
    recorder_stop = RecorderStop(stop, child_in_group=image_storage == 'raw')
    signal.signal(signal.SIGINT, recorder_stop.group_signal)
    signal.signal(signal.SIGTERM, recorder_stop.group_signal)
    protocol_path = Path(os.environ.get("NAV_CAPTURE_CONFIG", "")) if os.environ.get("NAV_CAPTURE_CONFIG") else Path(get_package_share_directory("nav_app")) / "config/capture.yaml"
    errors, subscriptions, stats, types, calibration = [], [], {}, {}, {}
    sensor_subscriptions = {}
    manifest = {"schema_version": 2, "state": "checking", "started_unix": time.time(),
                "record_arguments": args, "errors": errors, "algorithm": "raw_sensor_capture",
                "camera_model": "raw images with recorded CameraInfo; no implicit undistortion",
                "time_policy": "raw header stamps preserved; compensation only from parameter snapshot"}
    child = None
    bag_cache = None
    node = None
    telemetry = None
    qos_path = None
    cancelled = False
    from nav_rtk.parser.evidence import EvidenceWriter
    timeline = EvidenceWriter(name="capture")
    timeline("capture_start", output=str(output), arguments=args)
    network_checked = 0.
    executor = None
    image_encoder = None
    stored_selected = None
    try:
        protocol = yaml.safe_load(protocol_path.read_text(encoding="utf-8"))
        manifest["capture_protocol"] = protocol
        # build_record_arguments places topics last; known ROS CLI flags all have one value.
        selected = []
        index = 2
        while index < len(args):
            if args[index].startswith("--"):
                index += 2
            else:
                selected.append(args[index]); index += 1
        if not selected:
            raise ValueError("至少选择一个录制话题")
        manifest["selected_topics"] = selected
        missing = [protocol["roles"][role] for role in protocol["required_roles"] if protocol["roles"][role] not in selected]
        if missing:
            raise ValueError("完整采集缺少必需话题：" + ", ".join(missing))
        storage = option(args, "--storage", "sqlite3")
        if storage not in rosbag2_py.get_registered_writers():
            raise ValueError(f"未安装 {storage} 录包插件，请选择已安装的存储格式")
        report_state("checking")
        # Disable rclpy signal handlers: shutdown must not interrupt rosbag flushing.
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("nav_capture_probe")
        # Keep one registered wait set through preflight, recording and parameter
        # snapshots. The free spin_once(node) adds/removes the whole node for each
        # sample, repeatedly invalidating all subscription/guard-condition state.
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        archive_publisher = None
        if image_storage == 'jpeg95':
            from sensor_msgs.msg import CompressedImage
            source = protocol['roles']['rgb']
            stored = source + '/image_archive/jpeg'
            if node.get_publishers_info_by_topic(stored):
                raise ValueError('已有彩色归档任务正在运行')
            # Advertise during preflight. Creating this only at recorder launch
            # delayed DDS discovery and lost the first ~1.5s of archived RGB.
            archive_publisher = node.create_publisher(CompressedImage, stored,
                QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE))
        # Compact derived outputs are evidence for comparison, not replay inputs.
        # Never subscribe to growing Path/map/cloud outputs just to log them.
        diagnostic_topics = {
            "/fast_livo/diagnostics":"std_msgs/msg/String",
            "/fast_livo/input_diagnostics":"std_msgs/msg/String",
            "/aft_mapped_to_init":"nav_msgs/msg/Odometry",
            "/tf":"tf2_msgs/msg/TFMessage",
            "/localizer/localizer_node/quality":"std_msgs/msg/String",
            "/nav/outdoor/localization":"std_msgs/msg/String",
            "/nav/navigation/localization":"std_msgs/msg/String",
            "/rtk/status":"std_msgs/msg/String",
        }
        diagnostic_counts = {name:0 for name in diagnostic_topics}
        def receive_diagnostic(data, name, message_type):
            diagnostic_counts[name] += 1
            if len(data) <= 65536:
                import base64
                timeline("ros_diagnostic",topic=name,message_type=message_type,
                         encoding="cdr_base64",base64=base64.b64encode(data).decode("ascii"))
            else:
                timeline("diagnostic_oversize",topic=name,bytes=len(data))
        for name,message_type in diagnostic_topics.items():
            subscriptions.append(node.create_subscription(get_message(message_type),name,
                lambda data,n=name,t=message_type:receive_diagnostic(data,n,t),
                QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE),raw=True))
        manifest["diagnostic_topics"] = diagnostic_topics
        deadline = time.monotonic() + 8.

        def receive(data, name, message_type, cls):
            stats[name].add(data, message_type)
            timeline("sensor_observed",topic=name,header_ns=header_stamp(data,message_type),bytes=len(data))
            if message_type.endswith(("/CameraInfo", "/TFMessage", "/Extrinsics", "/Metadata")):
                # Keep only the latest small calibration message, never image payloads.
                calibration[name] = message_to_ordereddict(deserialize_message(data, cls))

        while time.monotonic() < deadline and not stop.is_set():
            graph = dict(node.get_topic_names_and_types())
            for name in selected:
                if name in types or name not in graph:
                    continue
                choices = graph[name]
                if len(choices) != 1:
                    raise ValueError(f"话题类型不唯一：{name}")
                message_type = choices[0]
                cls = get_message(message_type)
                infos = node.get_publishers_info_by_topic(name)
                if not infos:
                    continue
                transient = all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
                # A best-effort observer can lose fragmented multi-megabyte RGB
                # while the reliable recorder receives every frame. That caused
                # a false stream-loss stop in a real combined-load capture. Match
                # the offered reliability just as the recorder does, with a small
                # bounded image history and a longer (cheap) IMU history.
                reliable = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
                qos = QoSProfile(depth=1000 if message_type == "sensor_msgs/msg/Imu" else 8,
                    reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
                stats[name] = StreamStats(timestamp_required=STAMPLESS_DIAGNOSTIC_TOPICS.get(name) != message_type)
                types[name] = message_type
                sensor_subscriptions[name] = node.create_subscription(cls, name,
                    lambda data, n=name, t=message_type, c=cls: receive(data,n,t,c), qos, raw=True)
            executor.spin_once(timeout_sec=.05)
            if len(types) == len(selected) and not topic_issues(stats, types, time.monotonic(), preflight=True):
                break
        if stop.is_set():
            cancelled = True
            raise ValueError("已取消录制准备")
        missing = sorted(set(selected)-set(types))
        manifest["optional_rtk_missing"] = [name for name in missing if name in OPTIONAL_RTK_TOPICS]
        now = time.monotonic()
        preflight_health = capture_preflight(stats, types, selected, protocol, now)
        errors.extend(preflight_health["errors"])
        manifest["observation_warnings"] = list(preflight_health["warnings"])
        manifest['warnings'] = optional_rtk_warnings(selected, stats, types, now)
        for warning in manifest['warnings']:
            print(warning, flush=True)
        info_topic = protocol["roles"]["camera_info"]
        info = calibration.get(info_topic, {})
        camera_selected = protocol['roles']['rgb'] in selected or info_topic in selected
        if protocol['roles']['rgb'] in selected and info_topic not in selected:
            errors.append('录制 RGB 时必须同时选择对应 CameraInfo')
        if camera_selected and (not info.get("width") or not info.get("height") or len(info.get("k", [])) != 9 or info["k"][0] <= 0):
            errors.append("相机内参无效或尚未收到")
        manifest["requested_record_arguments"] = list(args)
        storage_config = None
        if storage == "mcap" and "--storage-config-file" not in args and "--storage-preset-profile" not in args:
            storage_config = Path(get_package_share_directory("nav_app"))/"config/capture_mcap.yaml"
            raw_config = storage_config.read_bytes()
            manifest["storage_configuration"] = {"path": str(storage_config),
                "sha256": hashlib.sha256(raw_config).hexdigest(),
                "configuration": yaml.safe_load(raw_config)}
        args, budgets = bound_record_buffers(args,max((row.largest_message for row in stats.values()),default=0),
                                            storage_config_path=storage_config)
        manifest["buffer_budget"] = budgets
        manifest["record_arguments"] = args
        if budgets["compression_queue_estimate_bytes"] > budgets["compression_budget_bytes"]:
            errors.append("单帧超过64 MiB压缩缓冲预算，请降低采集分辨率")
        cache = int(option(args, "--max-cache-size", 0))
        split = int(option(args, "--max-bag-size", 0))
        queue = int(option(args, "--compression-queue-size", 0))
        if split > 0 and option(args,"--compression-mode") == "message" and int(option(args,"--compression-threads",1)) > 1:
            errors.append("分片消息压缩需要单线程以保持跨分片帧顺序，请将压缩线程设为 1")
        queue_bytes = queue*max((row.largest_message for row in stats.values()), default=0) if option(args,"--compression-mode") == "message" else 0
        # Bound the DDS receive history as well as the rosbag cache/compressor queue.
        # Missing streams are already reported above. Preserve that diagnostic
        # instead of raising a KeyError while estimating their queue budget.
        qos_depths = {name: recording_history_depth(types[name]) for name in selected if name in types}
        queue_bytes += sum(qos_depths[name]*row.largest_message for name, row in stats.items())
        # Include this observer's DDS history in the same resource preflight.
        queue_bytes += sum((1000 if types[name] == "sensor_msgs/msg/Imu" else 8)*row.largest_message
                           for name, row in stats.items())
        queue_bytes += len(diagnostic_topics)*32*65536 + 512*1024
        if image_storage == 'jpeg95':
            # Original-image DDS histories above conservatively cover the
            # archive recorder and encoder subscription. Add FIFO and publisher.
            rgb_stats = stats.get(protocol['roles']['rgb'])
            queue_bytes += ARCHIVE_MAX_BYTES + ARCHIVE_MAX_FRAMES * (rgb_stats.largest_message if rgb_stats else 0)
        resources = resource_snapshot(output.parent)
        errors.extend(resource_errors(resources, cache, split, queue_bytes))
        manifest["preflight"] = {"resources": resources, "estimated_queue_bytes": queue_bytes,
                                 "topics": {n: s.document() for n,s in stats.items()}}
        if errors:
            raise ValueError("；".join(errors))
        preflight_only = release_image_observers(node, sensor_subscriptions, types)
        live_types = {name: kind for name, kind in types.items() if name not in preflight_only}
        manifest["observation_policy"] = {
            "preflight_only_topics": sorted(preflight_only),
            "continuous_topics": sorted(live_types),
            "saved_message_counts_source": "nav_audit.json",
            "image_delivery_checked": "preflight and post_record_audit",
        }
        manifest["parameters_start"] = snapshot_parameters(node, protocol.get("parameter_nodes", []), executor=executor)
        if require_mapping:
            manifest['required_parameter_nodes'] = ['/laserMapping']
            if not manifest['parameters_start'].get('/laserMapping', {}).get('parameters'):
                # The handheld starts the estimator and recorder concurrently.
                # Camera discovery/estimator initialization can finish after the
                # sensor-only preflight. Never start a mapping bag without its
                # original algorithm snapshot and discover that only at modeling.
                manifest['parameters_start'].update(snapshot_parameters(
                    node, ['/laserMapping'], service_timeout=15., call_timeout=3., executor=executor))
            if not manifest['parameters_start'].get('/laserMapping', {}).get('parameters'):
                raise ValueError('建图参数尚未就绪，未开始录包；请检查建图是否正常启动')
        manifest["config_files"] = snapshot_config_files(manifest["parameters_start"])
        manifest["calibration_start"] = dict(calibration)
        manifest["software"] = {"ros_distro": os.environ.get("ROS_DISTRO"), "app_module": str(Path(__file__).resolve())}
        from ament_index_python.packages import get_package_prefix
        artifacts=[Path(__file__).resolve()]
        for package,relative in (("fast_livo","lib/liblaser_mapping.so"),
                                 ("livox_ros_driver2","lib/livox_ros_driver2/livox_ros_driver2_node")):
            try:artifacts.append(Path(get_package_prefix(package))/relative)
            except LookupError as exc:
                manifest.setdefault("software_warnings", []).append(f"可选制品未安装：{exc}")
        manifest["software"]["artifact_sha256"]={}
        for artifact in artifacts:
            try:
                digest=hashlib.sha256()
                with artifact.open("rb") as stream:
                    for block in iter(lambda:stream.read(65536),b""):digest.update(block)
                manifest["software"]["artifact_sha256"][str(artifact)]=digest.hexdigest()
            except OSError as exc:
                manifest["software"]["artifact_sha256"][str(artifact)]={"error":str(exc)}
        manifest["clock_snapshot"]={"wall_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns(),
                                    "timezone":time.strftime("%z"),"kernel":os.uname().release}
        try:
            root = Path(__file__).resolve().parents[2]
            manifest["software"]["git_commit"] = subprocess.check_output(["git","-C",str(root),"rev-parse","HEAD"], stderr=subprocess.DEVNULL, timeout=2, text=True).strip()
            manifest["software"]["git_dirty"] = bool(subprocess.check_output(["git","-C",str(root),"status","--porcelain","--untracked-files=no"], stderr=subprocess.DEVNULL, timeout=2, text=True).strip())
        except (OSError, subprocess.SubprocessError):
            manifest["software"]["git_commit"] = None
        if stop.is_set():
            cancelled = True
            raise ValueError("已取消录制准备")
        stored_selected = list(selected)
        if image_storage == 'jpeg95':
            from copy import deepcopy
            from .features.image_archive import ImageArchiveEncoder
            source = protocol['roles']['rgb']
            if types.get(source) != 'sensor_msgs/msg/Image':
                raise ValueError('长时采集需要原始彩色图像输入')
            stored = source + '/image_archive/jpeg'
            stored_selected = [stored if n == source else n for n in selected]
            args = [stored if a == source else a for a in args]
            manifest['source_selected_topics'] = list(selected)
            manifest['selected_topics'] = stored_selected
            manifest['source_capture_protocol'] = deepcopy(protocol)
            manifest['capture_protocol'] = deepcopy(protocol)
            manifest['capture_protocol']['roles']['rgb'] = stored
            manifest['image_archive'] = dict(schema=1, profile='jpeg95', lossy=True,
                images=[dict(source_topic=source, stored_topic=stored, codec='jpeg', quality=95)])
            manifest['camera_model'] = 'JPEG95 RGB; original dimensions, header, CameraInfo and extrinsics preserved'
        qos_profiles = {}
        for name in selected:
            if name not in types:
                continue
            infos = node.get_publishers_info_by_topic(name)
            reliable = infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
            transient = types[name] in STATIC_TYPES and infos and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
            qos_profiles[name] = {"history":"keep_last", "depth":qos_depths[name],
                "reliability":"reliable" if reliable else "best_effort",
                "durability":"transient_local" if transient else "volatile"}
        if image_storage == 'jpeg95':
            qos_profiles.pop(source, None)
            qos_profiles[stored] = dict(history='keep_last', depth=64,
                reliability='reliable', durability='volatile')
        qos_path = state_dir/f"capture-qos-{os.getpid()}-{time.time_ns()}.yaml"
        qos_path.write_text(yaml.safe_dump(qos_profiles), encoding="utf-8")
        args[2:2] = ["--qos-profile-overrides-path",str(qos_path)]
        manifest["qos_profiles"] = qos_profiles
        manifest['record_arguments'] = args
        command = ([sys.executable, '-m', 'nav_app.record_child', *args]
                   if image_storage == 'jpeg95' else ['ros2', *args])
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1, start_new_session=image_storage == 'jpeg95')
        bag_cache = BagCacheWriter(output).start()
        def forward():
            for line in child.stdout:
                print(line.rstrip(), flush=True)
                if recorder_fault(line):
                    errors.append("录包进程报告丢消息或写入异常：" + line.strip()[:300])
                    stop.set()
        reader_thread = threading.Thread(target=forward, daemon=True)
        reader_thread.start()
        if image_storage == 'jpeg95':
            from .features.image_archive import pending_recorder_endpoints
            offered_topics=[name for name in stored_selected if name==stored or name in types]
            readiness_deadline=time.monotonic()+20.
            pending=list(offered_topics)
            while child.poll() is None and not stop.is_set() and time.monotonic()<readiness_deadline:
                executor.spin_once(timeout_sec=.05)
                pending=pending_recorder_endpoints(node,archive_publisher,offered_topics)
                if not pending:break
            if stop.is_set():
                cancelled=True
                raise ValueError('已取消录制准备，已有片段保留')
            if pending or child.poll() is not None:
                raise ValueError('录制接收端尚未就绪：'+'、'.join(pending or ['录包进程已退出']))
            manifest['image_archive']['recorder_ready_unix']=time.time()
            manifest['image_archive']['readiness_topics']=offered_topics
            # Subscribe to the original images only after every offered selected
            # recorder endpoint and the reliable JPEG match exist. Publishing
            # into the cold discovery window silently lost startup frames.
            image_encoder=ImageArchiveEncoder(node,source,stored,archive_publisher)
        report_state("running")
        manifest["state"] = "recording"
        last_check, requested_stop, flush_started = 0., False, 0.
        previous_observation_warnings = []
        if manifest["observation_warnings"]:
            print("NAV_RECORD_WARNING 部分辅助数据尚未就绪，正在录制已有数据", flush=True)
        baseline_counts = {n:s.count for n,s in stats.items()}
        while child.poll() is None:
            executor.spin_once(timeout_sec=.02)
            now = time.monotonic()
            if now-last_check >= .5 and not requested_stop:
                last_check = now
                resources = resource_snapshot(output.parent)
                for warning in optional_rtk_warnings(selected, stats, types, now):
                    if warning not in manifest['warnings']:
                        manifest['warnings'].append(warning)
                        print(warning, flush=True)
                timeline("resources", **resources)
                if now-network_checked >= 1.:
                    counters={}
                    for filename in ("/proc/net/udp","/proc/net/snmp"):
                        try:
                            with open(filename) as stream:
                                counters[filename]=stream.read(65536)
                        except OSError as exc:
                            counters[filename]=type(exc).__name__
                    timeline("network_counters", counters=counters)
                    network_checked=now
                health = recording_health(stats, live_types, now, resources, cache, split, queue_bytes,
                                          max_silence=protocol.get("max_silence_s",2.))
                observed_warnings = health["warnings"]
                if observed_warnings != previous_observation_warnings:
                    timeline("observation_health", warnings=observed_warnings,
                             action="continue_recording")
                    for warning in observed_warnings:
                        if warning not in manifest["observation_warnings"]:
                            manifest["observation_warnings"].append(warning)
                    print("NAV_RECORD_WARNING " + ("部分数据监测延迟或异常，录制继续，保存后核对数据"
                          if observed_warnings else ""), flush=True)
                    previous_observation_warnings = list(observed_warnings)
                runtime_errors = health["errors"]
                if bag_cache.error:
                    runtime_errors.append("录包分片同步失败：" + bag_cache.error)
                if image_encoder is not None:
                    image_status = image_encoder.worker.snapshot()
                    manifest['image_archive']['statistics'] = image_status
                    if image_status['error']:
                        runtime_errors.append('彩色图像归档失败：' + image_status['error'])
                    if image_status['dropped'] and not manifest.get('image_overload_warned'):
                        manifest['image_overload_warned'] = True
                        print('NAV_RECORD_WARNING 彩色图像归档跟不上输入，部分帧未保存；建图继续', flush=True)
                if runtime_errors:
                    errors.extend(runtime_errors)
                    stop.set()
                if output.is_dir():
                    if telemetry is None:
                        telemetry = (output/"nav_resources.jsonl").open("a",encoding="utf-8")
                        atomic_json(output/"nav_manifest.json", manifest)
                    telemetry.write(json.dumps(resources)+"\n"); telemetry.flush()
            if stop.is_set() and not requested_stop:
                requested_stop = True
                flush_started = now
                report_state("flushing")
                if image_encoder is not None:
                    image_status = image_encoder.close()
                    manifest['image_archive']['statistics'] = image_status
                    manifest['image_archive']['recording_window'] = image_status['recording_window']
                    if image_status['error'] or image_status['dropped']:
                        errors.append('彩色图像归档不完整：' + json.dumps(image_status, ensure_ascii=False))
                    image_encoder = None
                recorder_stop.request(child)
            if requested_stop and now-flush_started > 870:
                errors.append("录包收尾超时，输出未通过验收")
                child.kill()
                break
        child.wait(timeout=10)
        if image_encoder is not None:
            manifest['image_archive']['statistics'] = image_encoder.close()
            manifest['image_archive']['recording_window'] = manifest['image_archive']['statistics']['recording_window']
            image_encoder = None
        manifest["file_cache"] = bag_cache.close()
        bag_cache = None
        if manifest["file_cache"]["error"]:
            errors.append("录包分片同步失败：" + manifest["file_cache"]["error"])
        reader_thread.join(timeout=2)
        if child.returncode not in (0, -signal.SIGINT):
            errors.append(f"录包进程异常退出：{child.returncode}")
        if not requested_stop and not stop.is_set():
            errors.append("录制未经过正常停止流程")
        manifest["observed_topics"] = observed_topic_report(stats, baseline_counts, preflight_only)
        manifest["calibration_end"] = calibration
        manifest["parameters_end"] = snapshot_parameters(node, protocol.get("parameter_nodes", []), executor=executor)
        # rosbag's performance-oriented SQLite settings may leave dirty page-cache
        # data after close. Confirm every split and metadata reached the filesystem.
        metadata_path = output/"metadata.yaml"
        metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
        for filename in ["metadata.yaml", *metadata["relative_file_paths"]]:
            path = (output/filename).resolve()
            if output.resolve() not in path.parents:
                raise ValueError("录包分片路径越界")
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
        manifest["storage_flush_confirmed"] = True
        manifest.update(storage_state='stored', audit_state='pending')
    except (OSError, RuntimeError, ValueError, TypeError, KeyError, MemoryError) as exc:
        import traceback
        traceback.print_exc()
        if str(exc) not in errors and str(exc) != "；".join(errors):
            errors.append(str(exc))
    finally:
        if image_encoder is not None:
            try:
                manifest['image_archive']['statistics'] = image_encoder.close()
                manifest['image_archive']['recording_window'] = manifest['image_archive']['statistics']['recording_window']
            except (OSError, RuntimeError, ValueError, TypeError, KeyError, MemoryError) as exc:
                errors.append('彩色图像归档收尾失败：' + str(exc))
        if child and child.poll() is None:
            recorder_stop.request(child)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
        if telemetry:
            telemetry.close()
        if bag_cache is not None:
            manifest["file_cache"] = bag_cache.close()
            if manifest["file_cache"]["error"]:
                errors.append("录包分片同步失败：" + manifest["file_cache"]["error"])
        if qos_path is not None:
            if output.is_dir():
                (output/"nav_qos.yaml").write_bytes(qos_path.read_bytes())
            qos_path.unlink(missing_ok=True)
        if executor is not None:
            if node:
                executor.remove_node(node)
            executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        timeline("capture_end", errors=list(dict.fromkeys(errors)))
        timeline.close()
        manifest["timeline_writer"] = timeline.snapshot()
        manifest["diagnostic_counts"] = locals().get("diagnostic_counts",{})
        manifest["storage_errors"] = list(dict.fromkeys(errors))
        # Even a degraded capture can have durably closed files. Preserve both
        # facts; only the offline readback may assign the final quality state.
        manifest["state"] = ("cancelled" if cancelled else
                             "stored" if manifest.get('storage_flush_confirmed') else "incomplete")
        manifest["ended_unix"] = time.time()
        manifest["errors"] = list(dict.fromkeys(errors))
        # Persist failure evidence even if rosbag2 exited before creating metadata.
        output.mkdir(parents=True, exist_ok=True)
        try:
            manifest['diagnostic_snapshot'] = snapshot_evidence(output,manifest['started_unix'])
            manifest['diagnostics_state'] = 'pending'
        except (OSError, RuntimeError, ValueError, TypeError, KeyError, MemoryError) as exc:
            manifest['diagnostic_snapshot'] = {'schema_version':1,'snapshot_unix':time.time(),'errors':[str(exc)],'files':[]}
            manifest['diagnostics_state'] = 'pending'
        # Sensor data integrity and diagnostic completeness are separate facts.
        # A missing optional RTK receiver never invents data or blocks indoor capture.
        manifest['diagnostics_complete'] = False
        for issue in manifest['diagnostic_snapshot']['errors']:
            print("诊断记录不完整："+issue,flush=True)
        atomic_json(output/"nav_manifest.json", manifest)
        for error in manifest["errors"]:
            print(error, flush=True)
        report_state(manifest["state"])
        lock.close()
    return 2 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())

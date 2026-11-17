"""Localizer maintenance tools used by the navigation desktop app.

This module deliberately keeps process execution and ROS service calls out of
the Qt page.  Additional localization backends can provide another adapter
without changing the page layout.
"""

from __future__ import annotations

import hashlib
import inspect
import json
import math
from pathlib import Path
import struct
import threading
import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

from ..ros_launch_process import RosLaunchProcess
from .capture_quality import atomic_json
from .localization_health import LocalizationHealth

try:
    from std_srvs.srv import Trigger
except ImportError:  # Keep the desktop UI importable before ROS is sourced.
    Trigger = None

try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
except ImportError:  # Keep the desktop UI importable before ROS is sourced.
    PoseWithCovarianceStamped = None

try:
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
except ImportError:  # Keep the desktop UI importable before ROS is sourced.
    Parameter = ParameterType = ParameterValue = SetParameters = None


DEFAULT_GLOBAL_RELOCALIZE_SERVICE = (
    "/localizer/localizer_node/global_relocalize"
)
DEFAULT_CANCEL_RELOCALIZE_SERVICE = (
    "/localizer/localizer_node/cancel_relocalization"
)
DEFAULT_RELOCALIZATION_TIMEOUT_S = 120
RELOCALIZATION_TIMEOUT_CHOICES = (60, 120, 300, 600)

RELOCALIZATION_STAGE_LABELS = {
    "waiting_input": "等待同步点云和里程计",
    "preprocessing": "正在预处理点云",
    "scan_context_search": "正在搜索 Scan Context 候选",
    "scan_context_candidates": "正在验证全局候选位置",
    "scan_context_matching": "正在验证全局候选位置",
    "correlative_search": "正在按点云形状搜索地图位置",
    "global_registration": "正在执行全局特征匹配",
    "global_features_source": "正在提取当前点云特征",
    "global_features_target": "正在提取地图特征",
    "map_support_check": "正在检查候选位置的地图支撑",
    "coarse_gicp": "正在执行粗匹配",
    "refine_gicp": "正在执行精匹配",
    "verifying_fresh_scan": "匹配已通过，等待新点云复核",
    "complete": "本次匹配已结束",
    "idle": "等待开始重定位",
}

RELOCALIZATION_REASON_LABELS = {
    "INPUT_REJECTED": "定位输入不符合要求",
    "INSUFFICIENT_POINTS": "有效点云数量不足",
    "SC_NO_CANDIDATE": "Scan Context 没有找到候选位置",
    "SC_CANDIDATES_REJECTED": "全局候选均未通过点云匹配",
    "MAP_SUPPORT_LOW": "候选位置在地图中的点云支撑不足",
    "GLOBAL_GEOMETRY_AMBIGUOUS": "多个位置形状相似，请换个视角后重试定位",
    "GLOBAL_REGISTRATION_FAILED": "全局特征匹配失败",
    "GLOBAL_REGISTRATION_DISABLED": "没有先验且全局匹配未启用",
    "MAP_EMPTY": "定位地图点云为空",
    "COARSE_NOT_CONVERGED": "粗匹配未收敛",
    "COARSE_SCORE_HIGH": "粗匹配分数超过阈值",
    "REFINE_NOT_CONVERGED": "精匹配未收敛",
    "REFINE_SCORE_HIGH": "精匹配分数超过阈值",
    "RESULT_STALE": "匹配完成时使用的点云已经超过本次时限",
    "FRESH_INPUT_TIMEOUT": "已匹配到地图，但新点云持续延迟，未能确认当前位置",
    "RESULT_REJECTED": "匹配结果未通过几何安全检查",
    "WAITING_FRESH_CONFIRMATION": "正在等待一帧新点云复核结果",
    "ODOM_EPOCH_CHANGED": "里程计坐标系或时间序列发生变化",
    "WORKER_EXCEPTION": "点云匹配线程发生异常",
    "REQUEST_CANCELLED": "本次重定位已取消",
    "MATCH_FAILED": "点云匹配未通过",
}

INPUT_ERROR_LABELS = {
    "cloud_odom_frame_mismatch": "点云坐标系与里程计机身坐标系不一致",
    "invalid_cloud_layout": "点云字段或内存布局无效",
    "invalid_odometry_quaternion": "里程计姿态四元数无效",
    "invalid_odometry_position": "里程计位置无效",
    "invalid_cloud_conversion": "点云转换失败",
    "insufficient_finite_points": "点云中的有效点数量不足",
}


def validate_relocalization_timeout(value) -> int:
    try:
        timeout = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("重定位时限必须是 30–600 秒") from exc
    if timeout < 30 or timeout > 600:
        raise ValueError("重定位时限必须是 30–600 秒")
    return timeout


def call_with_optional_timeout(callback, *args, timeout_s: int):
    """Keep test/custom adapters compatible while extending the ROS adapter."""
    try:
        parameters = inspect.signature(callback).parameters.values()
        accepts_timeout = any(
            parameter.name == "timeout_s"
            or parameter.kind == inspect.Parameter.VAR_KEYWORD
            for parameter in parameters)
    except (TypeError, ValueError):
        accepts_timeout = True
    if accepts_timeout:
        return callback(*args, timeout_s=timeout_s)
    return callback(*args)


def describe_relocalization_progress(row) -> str:
    input_error = str(row.get("input_error") or "")
    if input_error:
        return INPUT_ERROR_LABELS.get(input_error, f"定位输入异常：{input_error}")
    reason = str(row.get("reason_code") or "")
    stage = str(row.get("stage") or "")
    if reason and reason != "WAITING_FRESH_CONFIRMATION":
        return RELOCALIZATION_REASON_LABELS.get(reason, f"匹配异常：{reason}")
    return RELOCALIZATION_STAGE_LABELS.get(
        stage, "正在等待定位服务返回进度")


def format_relocalization_failure(row, prefix: str) -> str:
    detail = describe_relocalization_progress(row)
    reason = str(row.get("reason_code") or "")
    if reason in ('RESULT_STALE', 'FRESH_INPUT_TIMEOUT'):
        age = row.get('data_age_s')
        if isinstance(age, (int, float)) and age > 1:
            detail = f'定位输入延迟{age:.1f}秒，需要更新的点云确认当前位置'
        return f"{prefix}：{detail}"
    if reason == "COARSE_SCORE_HIGH":
        score, threshold = row.get("rough_fitness"), row.get("rough_threshold")
    elif reason in ('REFINE_SCORE_HIGH', 'MATCH_FAILED', 'RESULT_REJECTED'):
        score, threshold = row.get("refine_fitness"), row.get("refine_threshold")
    else:
        return f"{prefix}：{detail}"
    if (
        isinstance(score, (int, float)) and score >= 0
        and isinstance(threshold, (int, float)) and threshold >= 0
    ):
        detail += f"（实际 {score:.3f}，阈值 {threshold:.3f}）"
    return f"{prefix}：{detail}"


class RelocalizationDiagnosticsRecorder:
    """Persist a small, structured timeline for the current field attempt."""

    QUALITY_FIELDS = (
        "instance_id", "map_sha256", "map_frame", "odom_frame",
        "input_error", "data_age_s", "operation", "stage", "reason_code",
        "operation_elapsed_s", "stage_elapsed_s", "progress_sequence",
        "input_points", "rough_points", "refine_points", "candidate_count",
        "supported_candidates", "map_support_ratio", "rough_converged",
        "rough_fitness", "rough_threshold", "refine_converged",
        "refine_fitness", "refine_threshold", "inlier_ratio", "rmse",
        "match_in_progress", "verifying_initial_pose", "fitness", "failures",
        "request_id", "manual_request_id", "published_stamp_ns",
        "relocalization_timeout_s",
    )

    def __init__(self, directory=None):
        self.directory = Path(
            directory or Path.home() / ".local/state/nav/diagnostics")
        self.current = None
        self._last_signature = None
        self._last_sample = 0.0

    def begin(self, mode: str, timeout_s: int, **context) -> str:
        attempt_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{time.time_ns()%1_000_000_000:09d}"
        self.current = dict(
            schema_version=1,
            attempt_id=attempt_id,
            mode=str(mode),
            timeout_s=int(timeout_s),
            started_unix_time=time.time(),
            context=context,
            result=None,
        )
        self._last_signature = None
        self._last_sample = 0.0
        self._write_event("started", self.current)
        self._save_latest()
        return attempt_id

    def quality(self, row) -> None:
        if self.current is None or not isinstance(row, dict):
            return
        now = time.monotonic()
        signature = (
            row.get("progress_sequence"), row.get("stage"),
            row.get("reason_code"), row.get("input_error"),
            row.get("match_in_progress"), row.get("verifying_initial_pose"),
        )
        if signature == self._last_signature and now - self._last_sample < 1.0:
            return
        self._last_signature = signature
        self._last_sample = now
        sample = {key: row.get(key) for key in self.QUALITY_FIELDS if key in row}
        self._write_event("quality", sample)
        self.current["latest_quality"] = sample
        self._save_latest()

    def finish(self, success: bool, message: str) -> None:
        if self.current is None:
            return
        result = dict(
            success=bool(success), message=str(message),
            finished_unix_time=time.time())
        self.current["result"] = result
        self._write_event("finished", result)
        self._save_latest()
        self.current = None

    def _write_event(self, event: str, payload) -> None:
        try:
            row = dict(
                unix_time=time.time(), event=event,
                attempt_id=(self.current or payload).get("attempt_id", ""),
                payload=payload)
            from .log_store import log_store
            data = json.dumps(row, ensure_ascii=False, allow_nan=False)
            log_store().write("localization", "relocalization", data)
            def save():
                self.directory.mkdir(parents=True, exist_ok=True)
                target = self.directory / "relocalization-attempts.jsonl"
                if target.exists() and target.stat().st_size > 2 * 1024**2:
                    target.replace(self.directory / "relocalization-attempts.previous.jsonl")
                with target.open("a", encoding="utf-8") as stream:
                    stream.write(data + "\n")
            log_store().submit(save, len(data) * 4 + 128)
        except (OSError, TypeError, ValueError):
            pass

    def _save_latest(self) -> None:
        if self.current is None:
            return
        try:
            data = json.dumps(self.current, ensure_ascii=False, allow_nan=False)
            from .log_store import log_store
            def save():
                self.directory.mkdir(parents=True, exist_ok=True)
                atomic_json(self.directory / "relocalization-latest.json", json.loads(data))
            log_store().submit(save, len(data) * 4 + 128)
        except (OSError, TypeError, ValueError):
            pass

_SCAN_CONTEXT_VALIDATION_CACHE = {}
_SCAN_CONTEXT_HEADER_BYTES = 112
_SCAN_CONTEXT_MAX_BYTES = 256 * 1024 * 1024
_SCAN_CONTEXT_RINGS = 20
_SCAN_CONTEXT_SECTORS = 60


def initial_pose_from_points(
    first,
    second,
    minimum_distance: float = 0.30,
) -> Tuple[float, float, float]:
    """Convert a position point and a forward-direction point to x/y/yaw."""
    try:
        position = np.asarray(first, dtype=float).reshape(-1)
        direction = np.asarray(second, dtype=float).reshape(-1)
        threshold = float(minimum_distance)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("请选择有效的位置点和方向点") from exc
    if (
        len(position) < 2
        or len(direction) < 2
        or not np.all(np.isfinite(position[:2]))
        or not np.all(np.isfinite(direction[:2]))
        or not math.isfinite(threshold)
        or threshold <= 0.0
    ):
        raise ValueError("请选择有效的位置点和方向点")
    delta = direction[:2] - position[:2]
    if float(np.linalg.norm(delta)) < threshold:
        raise ValueError(f"方向点离位置点至少需要 {threshold:.2f} 米")
    return (
        float(position[0]),
        float(position[1]),
        math.atan2(float(delta[1]), float(delta[0])),
    )


def scan_context_path(pcd_path: str) -> Path:
    """Return the file produced by ``localizer generate_sc_tool``."""
    return Path(str(Path(pcd_path).expanduser().resolve()) + ".sc.v2")


def validate_scan_context_source(pcd_path: str) -> Tuple[bool, str]:
    """Validate an SC source map without mutating it."""
    raw_path = str(pcd_path).strip()
    if not raw_path:
        return False, "请先在“地图”页选择定位 PCD 地图"
    path = Path(raw_path).expanduser()
    if path.suffix.lower() != ".pcd":
        return False, "Scan Context 只能从 PCD 点云地图生成"
    if not path.is_file():
        return False, f"定位地图不存在：{path}"
    return True, str(path.resolve())


def validate_scan_context_cache(
    pcd_path: str,
    cache_path: str,
    expected_radius: float | None = None,
    expected_grid: float | None = None,
    cancelled=None,
    progress=None,
) -> Tuple[bool, str]:
    """Verify that a v2 SC cache is complete and belongs to ``pcd_path``."""
    def report(done, total):
        if cancelled is not None and cancelled():
            raise ValueError("已取消地图核对")
        if progress is not None:
            progress(done, total)
    valid, value = validate_scan_context_source(pcd_path)
    if not valid:
        return False, value
    cache = Path(str(cache_path).strip()).expanduser()
    if not cache.is_file():
        return False, f"SC 地图不存在：{cache}"
    if not str(cache).endswith(".sc.v2"):
        return False, "SC 地图必须是 .sc.v2 文件"
    try:
        pcd = Path(value)
        pcd_stat = pcd.stat()
        cache_stat = cache.stat()
        total = pcd_stat.st_size + cache_stat.st_size
        report(0, total)
        cache_key = (
            str(pcd), pcd_stat.st_size, pcd_stat.st_mtime_ns,
            str(cache.resolve()), cache_stat.st_size, cache_stat.st_mtime_ns,
        )
        cached = _SCAN_CONTEXT_VALIDATION_CACHE.get(cache_key)
        if cached is not None:
            resolved, radius, grid = cached
            return _validate_scan_context_parameters(
                resolved, radius, grid, expected_radius, expected_grid)
        size = cache_stat.st_size
        if size < _SCAN_CONTEXT_HEADER_BYTES or size > _SCAN_CONTEXT_MAX_BYTES:
            return False, "SC 地图大小异常或文件未写完整"
        with cache.open("rb") as stream:
            header = stream.read(_SCAN_CONTEXT_HEADER_BYTES)
            if len(header) != _SCAN_CONTEXT_HEADER_BYTES:
                return False, "SC 地图头不完整"
            magic, version, endian = struct.unpack_from("<8sII", header, 0)
            if magic != b"GSSCC002" or version != 2 or endian != 0x01020304:
                return False, "SC 地图版本或字节序不受支持"
            identity = header[16:48]
            radius, grid, rings, sectors, entries = struct.unpack_from(
                "<ddIIQ", header, 48)
            checksum = header[80:112]
            if (
                rings != _SCAN_CONTEXT_RINGS
                or sectors != _SCAN_CONTEXT_SECTORS
                or not entries
                or not math.isfinite(radius)
                or not math.isfinite(grid)
                or radius <= 0
                or grid <= 0
            ):
                return False, "SC 地图结构或生成参数不受支持"
            entry_bytes = 20 + 8 * rings * sectors
            if size - _SCAN_CONTEXT_HEADER_BYTES != entries * entry_bytes:
                return False, "SC 地图结构不完整"
            payload_digest = hashlib.sha256()
            for _index in range(entries):
                if _index % 64 == 0:
                    report(_SCAN_CONTEXT_HEADER_BYTES + _index * entry_bytes, total)
                entry = stream.read(entry_bytes)
                if len(entry) != entry_bytes:
                    return False, "SC 地图条目未写完整"
                payload_digest.update(entry)
                pose = struct.unpack_from("<fff", entry, 0)
                rows, columns = struct.unpack_from("<ii", entry, 12)
                descriptor = np.frombuffer(entry, dtype="<f8", offset=20)
                if (
                    rows != rings
                    or columns != sectors
                    or not all(math.isfinite(value) for value in pose)
                    or not np.all(np.isfinite(descriptor))
                ):
                    return False, "SC 地图条目结构或数值无效"
        if payload_digest.digest() != checksum:
            return False, "SC 地图内容校验失败"
        digest = hashlib.sha256()
        with pcd.open("rb") as stream:
            processed = cache_stat.st_size
            while True:
                report(processed, total)
                block = stream.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
                processed += len(block)
        if digest.digest() != identity:
            return False, "SC 地图与所选 PCD 不匹配"
        if (pcd.stat().st_size, pcd.stat().st_mtime_ns, cache.stat().st_size, cache.stat().st_mtime_ns) != (
                pcd_stat.st_size, pcd_stat.st_mtime_ns, cache_stat.st_size, cache_stat.st_mtime_ns):
            return False, "地图在核对期间发生变化，请重新选择地图"
        report(total, total)
    except (OSError, struct.error, ValueError) as exc:
        return False, f"SC 地图无法校验：{exc}"
    resolved = str(cache.resolve())
    if len(_SCAN_CONTEXT_VALIDATION_CACHE) >= 64:
        _SCAN_CONTEXT_VALIDATION_CACHE.clear()
    _SCAN_CONTEXT_VALIDATION_CACHE[cache_key] = (resolved, radius, grid)
    return _validate_scan_context_parameters(
        resolved, radius, grid, expected_radius, expected_grid)


def _validate_scan_context_parameters(
    resolved: str,
    radius: float,
    grid: float,
    expected_radius: float | None,
    expected_grid: float | None,
) -> Tuple[bool, str]:
    if expected_radius is not None and radius != float(expected_radius):
        return False, (
            f"SC 最大半径为 {radius:g} m，当前定位配置需要 "
            f"{float(expected_radius):g} m")
    if expected_grid is not None and grid != float(expected_grid):
        return False, (
            f"SC 网格分辨率为 {grid:g} m，当前定位配置需要 "
            f"{float(expected_grid):g} m")
    return True, resolved


class LocalizationRosAdapter:
    """ROS-facing implementation of localizer runtime commands."""

    def __init__(self, node, *, quality_node=None) -> None:
        self.node = node
        self._trigger_clients: Dict[str, object] = {}
        from std_msgs.msg import String
        self.health = LocalizationHealth()
        self._pending_results = []
        self._pending_initial_poses = []
        self._pending_cancellations = []
        self._initial_pose_lock = threading.RLock()
        self._manual_reservation_serial = 0
        self._manual_reservation = None
        self._global_reservation_serial = 0
        self._global_reservation = None
        self._last_initial_pose_stamp_ns = 0
        self.diagnostics_hook = None
        self._desired_relocalization_timeout_s = (
            DEFAULT_RELOCALIZATION_TIMEOUT_S)
        self._timeout_future = None
        self._timeout_error = ""
        self._timeout_client = (
            node.create_client(
                SetParameters,
                "/localizer/localizer_node/set_parameters")
            if SetParameters is not None else None)
        self._cancel_client = (
            node.create_client(
                Trigger, DEFAULT_CANCEL_RELOCALIZE_SERVICE)
            if Trigger is not None else None)
        self._quality_sub = (quality_node or node).create_subscription(
            String, "/localizer/localizer_node/quality",
            self._quality_received, 1)
        self._initial_pose_pub = (
            node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            if PoseWithCovarianceStamped is not None else None)
        self._result_timer = node.create_timer(.2, self._poll_results)

    def _quality_received(self, message) -> None:
        self.health.update(message.data)
        if self.diagnostics_hook is not None and self.health.sample:
            self.diagnostics_hook(dict(self.health.sample))

    def request_relocalization_timeout(
        self, timeout_s: int,
    ) -> Tuple[bool, str]:
        try:
            timeout = validate_relocalization_timeout(timeout_s)
        except ValueError as exc:
            return False, str(exc)
        self._desired_relocalization_timeout_s = timeout
        self._timeout_error = ""
        self._poll_timeout_parameter()
        ready, message = self.relocalization_timeout_readiness(timeout)
        return True, message if not ready else f"重定位时限已设为 {timeout} 秒"

    def relocalization_timeout_readiness(
        self, timeout_s: int,
    ) -> Tuple[bool, str]:
        try:
            timeout = validate_relocalization_timeout(timeout_s)
        except ValueError as exc:
            return False, str(exc)
        current = self.health.sample.get("relocalization_timeout_s")
        if isinstance(current, (int, float)) and abs(float(current)-timeout) < .5:
            return True, f"重定位时限已设为 {timeout} 秒"
        timeout_error = getattr(self, "_timeout_error", "")
        if timeout_error:
            return False, timeout_error
        if timeout == DEFAULT_RELOCALIZATION_TIMEOUT_S and current is None:
            # Compatibility with the previously installed Localizer, whose
            # only supported value was the hard-coded 120 seconds.
            return True, "当前 Localizer 使用默认 120 秒时限"
        return False, f"正在把 Localizer 重定位时限设为 {timeout} 秒"

    def _poll_timeout_parameter(self) -> None:
        client = getattr(self, "_timeout_client", None)
        if client is None:
            return
        future = getattr(self, "_timeout_future", None)
        if future is not None:
            if not future.done():
                return
            self._timeout_future = None
            try:
                response = future.result()
                results = getattr(response, "results", []) if response else []
                if not results or not all(result.successful for result in results):
                    reason = next(
                        (result.reason for result in results
                         if not result.successful and result.reason), "")
                    self._timeout_error = (
                        f"Localizer 拒绝重定位时限：{reason or '未知原因'}")
                    return
                # Wait for the next quality heartbeat to confirm that the
                # server-side value actually changed before dispatching again.
                return
            except Exception as exc:
                self._timeout_error = f"设置 Localizer 重定位时限失败：{exc}"
                return
        current = self.health.sample.get("relocalization_timeout_s")
        desired = getattr(
            self, "_desired_relocalization_timeout_s",
            DEFAULT_RELOCALIZATION_TIMEOUT_S)
        if isinstance(current, (int, float)) and abs(float(current)-desired) < .5:
            self._timeout_error = ""
            return
        if not client.service_is_ready():
            return
        request = SetParameters.Request()
        value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(desired))
        request.parameters = [Parameter(
            name="relocalization_timeout_s", value=value)]
        try:
            self._timeout_future = client.call_async(request)
        except (RuntimeError, TypeError, ValueError) as exc:
            self._timeout_error = f"设置 Localizer 重定位时限失败：{exc}"

    def _cancel_localizer_operation(self, request, *, manual=False) -> None:
        """Retire one result and keep new commands behind its cancel receipt.

        Trigger has no request identity fields. Wait for the original request
        acknowledgement and check the current owner before sending it. In
        particular, a late global-service reply must not outlive its cancel.
        """
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            if not any(item["request"] is request for item in self._pending_cancellations):
                self._pending_cancellations.append(dict(
                    request=request, manual=manual, future=None, next_attempt=0.,
                    message="正在确认上次定位请求已取消，请稍候"))
            self._poll_cancellations()

    def _cancellation_message(self) -> str:
        pending = getattr(self, "_pending_cancellations", ())
        return pending[0]["message"] if pending else ""

    def _poll_cancellations(self) -> None:
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            remaining = []
            for item in self._pending_cancellations:
                request = item["request"]
                row = getattr(getattr(self, "health", None), "sample", {})
                received = getattr(getattr(self, "health", None), "received", 0.)
                fresh = (bool(row.get("instance_id")) and
                         0 <= time.monotonic() - received <= 1.5 and
                         -.1 <= (self.node.get_clock().now().nanoseconds -
                                  row.get("published_stamp_ns", 0)) / 1e9 <= 1.5)
                if not item["manual"] and request.get("request_id") is None:
                    original = request.get("future")
                    if original is None or not original.done():
                        if fresh and request.get("instance_id") and row["instance_id"] != request["instance_id"]:
                            continue  # The old node exited before acknowledging.
                        item["message"] = "正在等待上次定位请求回执，再确认取消"
                        remaining.append(item)
                        continue
                    try:
                        response = original.result()
                        if response is None or not response.success:
                            continue  # The node never queued this request.
                        parts = response.message.split(":", 2)
                        if len(parts) != 3 or parts[0] != "queued":
                            raise ValueError("missing request identity")
                        request["request_id"] = int(parts[1])
                        request["instance_id"] = parts[2]
                    except Exception:
                        if fresh and request.get("instance_id") and row["instance_id"] != request["instance_id"]:
                            continue
                        item["message"] = "无法确认上次定位请求，请恢复或重启定位服务"
                        remaining.append(item)
                        continue
                if fresh and request.get("instance_id") and row["instance_id"] != request["instance_id"]:
                    continue  # Never send an old cancellation to a new node.
                future = item["future"]
                if future is not None:
                    if future.done():
                        try:
                            response = future.result()
                            if response is not None and (response.success or
                                    response.message == "no relocalization is pending"):
                                continue
                            item["message"] = "定位服务未确认取消，请稍候或重启定位服务"
                        except Exception:
                            item["message"] = "取消定位连接中断，请恢复定位服务"
                        item["future"] = None
                        item["next_attempt"] = time.monotonic() + 1.
                    remaining.append(item)
                    continue
                if not fresh:
                    item["message"] = "取消尚未确认，请恢复定位服务连接"
                    remaining.append(item)
                    continue
                if item["manual"]:
                    stamp = request.get("pose_stamp_ns")
                    if row.get("manual_pose_stamp_ns", 0) > (stamp or 0):
                        continue  # Superseded by a newer manual command.
                    if stamp is None or row.get("manual_pose_stamp_ns") != stamp:
                        item["message"] = "正在等待上次位置请求回执，再确认取消"
                        remaining.append(item)
                        continue
                    if row.get("request_id", 0) > request.get("baseline_global_request_id", row.get("request_id", 0)):
                        continue
                    if (row.get("completed_manual_pose_stamp_ns") == stamp and
                            row.get("completed_manual_request_id") == row.get("manual_request_id")):
                        continue  # Completion is not undone by cancelling a wait.
                elif (row.get("request_id", 0) > request["request_id"] or
                      row.get("manual_request_id", 0) > request.get("baseline_manual_request_id", row.get("manual_request_id", 0))):
                    continue
                elif row.get("request_id", 0) != request["request_id"]:
                    remaining.append(item)
                    continue
                elif row.get("completed_request_id") == request["request_id"]:
                    continue  # Do not cancel ordinary tracking after completion.
                client = getattr(self, "_cancel_client", None)
                if client is None or not client.service_is_ready():
                    item["message"] = "取消服务未连接，尚未停止上次定位；请恢复定位服务"
                elif time.monotonic() >= item["next_attempt"]:
                    try:
                        item["future"] = client.call_async(Trigger.Request())
                        item["message"] = "已请求取消定位，正在等待服务确认"
                    except (RuntimeError, TypeError, ValueError):
                        item["message"] = "取消定位发送失败，请恢复定位服务连接"
                        item["next_attempt"] = time.monotonic() + 1.
                remaining.append(item)
            self._pending_cancellations = remaining

    def initial_pose_readiness(
        self,
        frame_id: str,
        expected_map_sha256: str = "",
    ) -> Tuple[bool, str]:
        """Require a live Localizer which has loaded the selected map."""
        if self._cancellation_message():
            return False, self._cancellation_message()
        frame_id = str(frame_id).strip()
        if not frame_id:
            return False, "地图坐标系不能为空"
        if PoseWithCovarianceStamped is None or self._initial_pose_pub is None:
            return False, "未找到 geometry_msgs，请构建并 source ROS 工作空间"
        if self._initial_pose_pub.get_subscription_count() <= 0:
            return False, "正在等待定位服务加载地图"
        row = self.health.sample
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds
        fresh = (
            bool(row.get("instance_id"))
            and 0 <= now - self.health.received <= 1.5
            and -.1 <= (ros_now - row.get("published_stamp_ns", 0)) / 1e9 <= 1.5
        )
        if not fresh:
            return False, "正在等待定位服务状态"
        if row.get("map_frame") != frame_id:
            return False, "定位服务加载的地图坐标系与当前地图不一致"
        expected_map_sha256 = str(expected_map_sha256).strip().lower()
        if (
            len(expected_map_sha256) != 64
            or any(character not in "0123456789abcdef"
                   for character in expected_map_sha256)
        ):
            return False, "当前点云地图身份尚未核对，不能提交重定位"
        if row.get("map_sha256") != expected_map_sha256:
            return False, "定位服务加载的点云不是当前所选地图"
        if row.get("match_in_progress") is True:
            return False, "定位算法仍在计算上一次匹配，请等待收尾"
        return True, "定位服务已就绪，可以提交两点先验"

    def publish_rtk_initial_pose(self, message, instance_id: str) -> bool:
        """Publish an RTK seed only while no user localization owns the node."""
        if self._initial_pose_pub is None:
            return False
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            if self._pending_cancellations:
                return False
            reservation = self._manual_reservation
            if reservation and reservation[1] == str(instance_id):
                return False
            reservation = self._global_reservation
            if reservation and (
                not reservation[1] or reservation[1] == str(instance_id)
            ):
                return False
            try:
                self._initial_pose_pub.publish(message)
            except (RuntimeError, TypeError, ValueError):
                return False
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            if stamp is not None:
                self._last_initial_pose_stamp_ns = max(
                    self._last_initial_pose_stamp_ns,
                    int(getattr(stamp, "sec", 0)) * 1_000_000_000
                    + int(getattr(stamp, "nanosec", 0)),
                )
        return True

    def initial_pose_subscription_count(self) -> int:
        if self._initial_pose_pub is None:
            return 0
        return int(self._initial_pose_pub.get_subscription_count())

    @property
    def initial_pose_publisher(self):
        return self._initial_pose_pub

    def _ensure_initial_pose_state(self) -> None:
        if not hasattr(self, "_initial_pose_lock"):
            self._initial_pose_lock = threading.RLock()
        if not hasattr(self, "_manual_reservation_serial"):
            self._manual_reservation_serial = 0
        if not hasattr(self, "_manual_reservation"):
            self._manual_reservation = None
        if not hasattr(self, "_global_reservation_serial"):
            self._global_reservation_serial = 0
        if not hasattr(self, "_global_reservation"):
            self._global_reservation = None
        if not hasattr(self, "_last_initial_pose_stamp_ns"):
            self._last_initial_pose_stamp_ns = 0
        if not hasattr(self, "_pending_initial_poses"):
            self._pending_initial_poses = []
        if not hasattr(self, "_pending_results"):
            self._pending_results = []
        if not hasattr(self, "_pending_cancellations"):
            self._pending_cancellations = []

    def _release_manual_reservation(self, token) -> None:
        lock = getattr(self, "_initial_pose_lock", None)
        if lock is None:
            return
        with lock:
            if self._manual_reservation and self._manual_reservation[0] == token:
                self._manual_reservation = None

    def _release_global_reservation(self, token) -> None:
        if token is None:
            return
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            if self._global_reservation and self._global_reservation[0] == token:
                self._global_reservation = None

    def cancel_initial_pose_requests(
        self, reason: str = "定位服务已停止，请重新提交位置和方向",
    ) -> bool:
        """Stop tracking manual results after the owning stack disappears."""
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            pending = self._pending_initial_poses
            self._pending_initial_poses = []
            for request in pending:
                if (
                    self._manual_reservation
                    and self._manual_reservation[0] == request.get("token")
                ):
                    self._manual_reservation = None
        for request in pending:
            self._cancel_localizer_operation(request, manual=True)
            request["done"](False, str(reason) + "；" + self._cancellation_message()
                            if self._cancellation_message() else str(reason))
        return bool(pending)

    def cancel_global_relocalization_requests(
        self, reason: str = "定位服务已停止，请重新发起自动匹配",
    ) -> bool:
        """Forget queued global results and release RTK publication gating."""
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            pending = self._pending_results
            self._pending_results = []
            self._global_reservation = None
        for request in pending:
            self._cancel_localizer_operation(request)
            request["done"](False, str(reason) + "；" + self._cancellation_message()
                            if self._cancellation_message() else str(reason))
        return bool(pending)

    def publish_initial_pose(
        self,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        expected_map_sha256: str,
        done: Callable[[bool, str], None],
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        try:
            timeout_s = validate_relocalization_timeout(timeout_s)
        except ValueError as exc:
            return False, str(exc)
        timeout_ready, timeout_message = (
            self.relocalization_timeout_readiness(timeout_s))
        if not timeout_ready:
            return False, timeout_message
        ready, message = self.initial_pose_readiness(
            frame_id, expected_map_sha256)
        if not ready:
            return False, message
        values = (float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in values):
            return False, "初始位置或方向不是有效数值"
        row = self.health.sample
        stamp = self.node.get_clock().now()
        sent_ns = stamp.nanoseconds
        message = PoseWithCovarianceStamped()
        message.header.frame_id = str(frame_id).strip()
        message.pose.pose.position.x = values[0]
        message.pose.pose.position.y = values[1]
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(values[2] * 0.5)
        message.pose.pose.orientation.w = math.cos(values[2] * 0.5)
        # Localizer currently consumes only the pose.  Keep meaningful
        # covariance for tools (and a possible future AMCL backend).
        message.pose.covariance[0] = 1.0
        message.pose.covariance[7] = 1.0
        message.pose.covariance[14] = 4.0
        message.pose.covariance[21] = math.radians(10.0) ** 2
        message.pose.covariance[28] = math.radians(10.0) ** 2
        message.pose.covariance[35] = math.radians(15.0) ** 2
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            stamp_floor = max(
                self._last_initial_pose_stamp_ns,
                int(row.get("manual_pose_stamp_ns", 0)),
            )
            pose_stamp_ns = max(sent_ns, stamp_floor + 1)
            stamp_message = stamp.to_msg()
            if pose_stamp_ns != stamp.nanoseconds:
                stamp_message.sec = pose_stamp_ns // 1_000_000_000
                stamp_message.nanosec = pose_stamp_ns % 1_000_000_000
            message.header.stamp = stamp_message
            self._manual_reservation_serial += 1
            token = self._manual_reservation_serial
            self._manual_reservation = (token, str(row.get("instance_id")))
            try:
                self._initial_pose_pub.publish(message)
            except (RuntimeError, TypeError, ValueError) as exc:
                self._manual_reservation = None
                return False, f"初始位姿发送失败：{exc}"
            self._last_initial_pose_stamp_ns = pose_stamp_ns
            self._pending_initial_poses.append(dict(
                done=done,
                deadline=time.monotonic() + timeout_s + 10.0,
                started_monotonic=time.monotonic(),
                timeout_s=timeout_s,
                started_ns=sent_ns,
                pose_stamp_ns=pose_stamp_ns,
                token=token,
                instance_id=row.get("instance_id"),
                map_sha256=str(expected_map_sha256).strip().lower(),
                match_sequence=int(row.get("match_sequence", 0)),
                baseline_manual_request_id=int(
                    row.get("manual_request_id", 0)),
                baseline_global_request_id=int(row.get("request_id", 0)),
                manual_request_id=None,
                acknowledged=False,
            ))
        return True, (
            f"已提交大概位置和方向，最长匹配 {timeout_s} 秒")

    def trigger_global_relocalization(
        self,
        service_name: str,
        done: Callable[[bool, str], None],
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        try:
            timeout_s = validate_relocalization_timeout(timeout_s)
        except ValueError as exc:
            return False, str(exc)
        timeout_ready, timeout_message = (
            self.relocalization_timeout_readiness(timeout_s))
        if not timeout_ready:
            return False, timeout_message
        service_name = str(service_name).strip()
        if not service_name:
            return False, "全局重定位服务名不能为空"
        if Trigger is None:
            return False, "未找到 std_srvs/Trigger，请构建并 source ROS 工作空间"
        client = self._trigger_clients.get(service_name)
        if client is None:
            try:
                client = self.node.create_client(Trigger, service_name)
            except (RuntimeError, TypeError, ValueError) as exc:
                return False, f"创建全局重定位客户端失败：{exc}"
            self._trigger_clients[service_name] = client
        if (
            not client.service_is_ready()
            and not client.wait_for_service(timeout_sec=0.1)
        ):
            return False, f"服务尚未就绪：{service_name}"
        self._ensure_initial_pose_state()
        with self._initial_pose_lock:
            if self._pending_cancellations:
                return False, self._cancellation_message()
            if self._manual_reservation is not None:
                return False, "两点重定位正在匹配，请稍候"
            if self._global_reservation is not None:
                return False, "全局重定位请求正在执行，请稍候"
            self._global_reservation_serial += 1
            token = self._global_reservation_serial
            instance_id = str(self.health.sample.get("instance_id") or "")
            self._global_reservation = (token, instance_id)
            started_ns = self.node.get_clock().now().nanoseconds
            try:
                future = client.call_async(Trigger.Request())
            except (RuntimeError, TypeError, ValueError) as exc:
                self._global_reservation = None
                return False, f"调用全局重定位服务失败：{exc}"
            self._pending_results.append(dict(
                future=future,
                done=done,
                request_id=None,
                deadline=time.monotonic() + timeout_s + 10.0,
                started_monotonic=time.monotonic(),
                timeout_s=timeout_s,
                started_ns=started_ns,
                instance_id=instance_id,
                baseline_manual_request_id=int(self.health.sample.get("manual_request_id", 0)),
                token=token,
            ))
        return True, f"已请求全局重定位，最长匹配 {timeout_s} 秒"

    def _timeout_message(self, request, manual: bool) -> str:
        timeout = int(request.get(
            "timeout_s", DEFAULT_RELOCALIZATION_TIMEOUT_S))
        row = self.health.sample
        prefix = f"重定位超过 {timeout} 秒"
        input_error = str(row.get("input_error") or "")
        if input_error:
            reason = INPUT_ERROR_LABELS.get(
                input_error, f"定位输入异常 {input_error}")
            return f"{prefix}：{reason}"
        data_age = row.get("data_age_s")
        if isinstance(data_age, (int, float)) and data_age > 1.0:
            return f"{prefix}：最近同步点云已有 {data_age:.1f} 秒未更新"
        if row.get("verifying_initial_pose") is True:
            return f"{prefix}：匹配已得到候选结果，但一直未收到新鲜点云复核"
        if row.get("match_in_progress") is True:
            stage = RELOCALIZATION_STAGE_LABELS.get(
                str(row.get("stage") or ""), "点云匹配")
            return f"{prefix}：算法仍在{stage}，已取消本次结果"
        reason = str(row.get("reason_code") or "")
        if reason:
            return f"{prefix}：{RELOCALIZATION_REASON_LABELS.get(reason, reason)}"
        if manual and not request.get("acknowledged"):
            return f"{prefix}：Localizer 未确认收到人工初始位姿"
        if not manual and request.get("request_id") is None:
            return f"{prefix}：全局重定位服务未返回请求编号"
        return f"{prefix}：未收到可验证的匹配终态"

    def _poll_results(self):
        self._ensure_initial_pose_state()
        self._poll_cancellations()
        self._poll_timeout_parameter()

        def finish(request, success, message):
            self._release_global_reservation(request.get("token"))
            request["done"](success, message)

        # The current ROS event pump runs on the Qt thread. Keeping the whole
        # snapshot under this lock also makes cancel/requeue atomic if that
        # executor becomes multithreaded later.
        with self._initial_pose_lock:
            pending, self._pending_results = self._pending_results, []
            for request in pending:
                if time.monotonic() > request["deadline"]:
                    self._cancel_localizer_operation(request)
                    finish(
                        request, False,
                        self._timeout_message(request, manual=False))
                    continue
                if (
                    request["request_id"] is None
                    and request["future"].done()
                ):
                    try:
                        response = request["future"].result()
                        if response is None or not response.success:
                            finish(request, False, "全局重定位请求失败")
                            continue
                        if not response.message.startswith("queued:"):
                            finish(
                                request, False,
                                "服务已接收请求，但未提供可验证的匹配结果",
                            )
                            continue
                        parts = response.message.split(":", 2)
                        request["request_id"] = int(parts[1])
                        if len(parts) == 3:
                            request["instance_id"] = parts[2]
                            reservation = self._global_reservation
                            if (
                                reservation
                                and reservation[0] == request.get("token")
                            ):
                                self._global_reservation = (
                                    reservation[0], parts[2])
                    except Exception as exc:
                        finish(
                            request, False,
                            f"全局重定位服务调用失败：{exc}")
                        continue
                result = None
                row = self.health.sample
                ros_now = self.node.get_clock().now().nanoseconds
                fresh = (
                    0 <= time.monotonic() - self.health.received <= 1.5
                    and -.1 <= (
                        ros_now - row.get("published_stamp_ns", 0)
                    ) / 1e9 <= 1.5
                )
                if (
                    request["request_id"] is not None
                    and fresh
                    and row.get("published_stamp_ns", 0)
                    >= request["started_ns"]
                ):
                    if (
                        request.get("instance_id")
                        and row.get("instance_id") != request["instance_id"]
                    ):
                        finish(
                            request, False,
                            "定位服务已重启，请重新发起定位")
                        continue
                    result = self.health.request_result(request["request_id"])
                    if result is True and not self.health.valid(ros_now):
                        # A completed computation is not yet a usable pose.
                        result = None
                if result is not None:
                    finish(
                        request,
                        result,
                        "全局重定位匹配成功" if result else
                        format_relocalization_failure(
                            row, "全局重定位匹配失败"),
                    )
                else:
                    self._pending_results.append(request)
        self._poll_initial_pose_results()

    def _poll_initial_pose_results(self):
        lock = getattr(self, "_initial_pose_lock", None)
        if lock is None:  # Compatibility for small adapter unit-test doubles.
            pending = getattr(self, "_pending_initial_poses", [])
            self._pending_initial_poses = []
        else:
            with lock:
                pending = self._pending_initial_poses
                self._pending_initial_poses = []

        def requeue(request):
            if lock is None:
                self._pending_initial_poses.append(request)
            else:
                with lock:
                    self._pending_initial_poses.append(request)

        def finish(request, success, message):
            self._release_manual_reservation(request.get("token"))
            request["done"](success, message)

        for request in pending:
            if time.monotonic() > request["deadline"]:
                self._cancel_localizer_operation(request, manual=True)
                finish(
                    request, False,
                    self._timeout_message(request, manual=True))
                continue
            row = self.health.sample
            ros_now = self.node.get_clock().now().nanoseconds
            fresh = (
                0 <= time.monotonic() - self.health.received <= 1.5
                and -.1 <= (ros_now - row.get("published_stamp_ns", 0)) / 1e9 <= 1.5
                and row.get("published_stamp_ns", 0) >= request["started_ns"]
            )
            if not fresh:
                requeue(request)
                continue
            if row.get("instance_id") != request.get("instance_id"):
                finish(request, False, "定位服务已重启，请重新提交位置和方向")
                continue
            expected_hash = request.get("map_sha256")
            if expected_hash and row.get("map_sha256") != expected_hash:
                finish(request, False, "定位服务地图已变化，请重新进入定位页面")
                continue

            # Current Localizer versions acknowledge the exact message stamp
            # and expose a separate completion ID.  This prevents a periodic
            # pre-request quality frame (or another match) from being mistaken
            # for the result of this manual prior.
            if "manual_request_id" in row:
                active_id = int(row.get("manual_request_id", 0))
                active_stamp = int(row.get("manual_pose_stamp_ns", 0))
                request_id = request.get("manual_request_id")
                if request_id is None:
                    if (
                        active_id > request["baseline_manual_request_id"]
                        and active_stamp == request["pose_stamp_ns"]
                    ):
                        request_id = active_id
                        request["manual_request_id"] = request_id
                        request["acknowledged"] = True
                    elif (
                        active_id > request["baseline_manual_request_id"]
                        and active_stamp > request["pose_stamp_ns"]
                    ):
                        finish(
                            request, False,
                            "初始位姿已被另一请求替代，请重新提交",
                        )
                        continue
                    else:
                        requeue(request)
                        continue
                if (
                    active_id > request_id
                    and active_stamp != request["pose_stamp_ns"]
                ):
                    finish(
                        request, False,
                        "初始位姿已被另一请求替代，请重新提交",
                    )
                    continue
                if (
                    int(row.get("completed_manual_request_id", 0))
                    != request_id
                    or int(row.get("completed_manual_pose_stamp_ns", 0))
                    != request["pose_stamp_ns"]
                ):
                    requeue(request)
                    continue
                succeeded = row.get("manual_request_succeeded") is True
                if succeeded and not self.health.valid(ros_now):
                    requeue(request)
                    continue
                finish(
                    request,
                    succeeded,
                    "点云匹配通过，已进入连续跟踪"
                    if succeeded else
                    format_relocalization_failure(
                        row, "本次先验未通过点云确认"),
                )
                continue

            # Compatibility with an older installed Localizer.  Do not accept
            # a terminal state until this request has first produced observable
            # matching activity.
            if row.get("match_in_progress") is True:
                request["acknowledged"] = True
            if not request.get("acknowledged"):
                requeue(request)
                continue
            if (
                row.get("valid") is True
                and row.get("match_sequence", 0) > request["match_sequence"]
                and self.health.valid(ros_now)
            ):
                finish(request, True, "点云匹配通过，已进入连续跟踪")
                continue
            if (
                row.get("failures", 0) >= 1
                and row.get("match_in_progress") is False
                and row.get("initial_pose_required") is True
                and row.get("match_sequence", 0) <= request["match_sequence"]
            ):
                finish(request, False, "本次先验未通过点云确认，请重新选点")
                continue
            requeue(request)


class UnavailableLocalizationRosAdapter:
    """Fallback used by UI tests and non-ROS previews."""

    def trigger_global_relocalization(
        self,
        _service_name: str,
        _done: Callable[[bool, str], None],
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        return False, "当前节点未启用定位 ROS 接口"

    def initial_pose_readiness(
        self, _frame_id: str, _expected_map_sha256: str = "",
    ) -> Tuple[bool, str]:
        return False, "当前节点未启用定位 ROS 接口"

    def publish_initial_pose(
        self,
        _frame_id: str,
        _x: float,
        _y: float,
        _yaw: float,
        _expected_map_sha256: str,
        _done: Callable[[bool, str], None],
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        return False, "当前节点未启用定位 ROS 接口"

    def cancel_initial_pose_requests(self, _reason: str = "") -> bool:
        return False

    def cancel_global_relocalization_requests(self, _reason: str = "") -> bool:
        return False

    def request_relocalization_timeout(
        self, _timeout_s: int,
    ) -> Tuple[bool, str]:
        if int(_timeout_s) == DEFAULT_RELOCALIZATION_TIMEOUT_S:
            return True, "使用默认 120 秒重定位时限"
        return False, "当前节点未启用定位 ROS 接口"

    def relocalization_timeout_readiness(
        self, _timeout_s: int,
    ) -> Tuple[bool, str]:
        if int(_timeout_s) == DEFAULT_RELOCALIZATION_TIMEOUT_S:
            return True, "使用默认 120 秒重定位时限"
        return False, "当前节点未启用定位 ROS 接口"


class LocalizationToolsController(QObject):
    """Own SC generation and global-relocalization operation state."""

    log_received = pyqtSignal(str)
    operation_state_changed = pyqtSignal(str, str)
    status_changed = pyqtSignal(str)

    def __init__(self, ros_adapter, parent=None) -> None:
        super().__init__(parent)
        self.ros_adapter = ros_adapter
        self.admission_guard = None
        self.task_submit = None
        self.task_cancel = None
        self._sc_job_id = None
        self._sc_request_token = 0
        self.sc_process = RosLaunchProcess("Scan Context 地图生成", self)
        self.sc_process.log_received.connect(self.log_received.emit)
        self.sc_process.state_changed.connect(self._on_sc_state_changed)
        self._expected_sc_path: Optional[Path] = None
        self._sc_cancel_requested = False
        self._global_relocalize_pending = False
        self._manual_relocalize_pending = False
        self._manual_relocalize_state = "idle"
        self._active_relocalization_started = None
        self._active_relocalization_timeout_s = (
            DEFAULT_RELOCALIZATION_TIMEOUT_S)
        self.diagnostics = RelocalizationDiagnosticsRecorder()
        if hasattr(self.ros_adapter, "diagnostics_hook"):
            self.ros_adapter.diagnostics_hook = self.diagnostics.quality
        self._shutting_down = False
        self.status = "可生成 SC 地图或触发全局重定位"

    @property
    def sc_generation_active(self) -> bool:
        return bool(self._sc_job_id or self.sc_process.active)

    @property
    def global_relocalize_pending(self) -> bool:
        return self._global_relocalize_pending

    @property
    def manual_relocalize_pending(self) -> bool:
        return self._manual_relocalize_pending

    @property
    def manual_relocalize_state(self) -> str:
        return self._manual_relocalize_state

    @property
    def active_relocalization_elapsed_s(self) -> float:
        if self._active_relocalization_started is None:
            return 0.0
        return max(0.0, time.monotonic()-self._active_relocalization_started)

    @property
    def active_relocalization_timeout_s(self) -> int:
        return self._active_relocalization_timeout_s

    def relocalization_progress(self) -> str:
        row = getattr(
            getattr(self.ros_adapter, "health", None), "sample", {})
        expected = (
            "manual" if self._manual_relocalize_pending else
            ("global" if self._global_relocalize_pending else ""))
        if expected and row.get("operation") not in (expected, "verification"):
            return "已提交请求，等待 Localizer 确认"
        return describe_relocalization_progress(row)

    def relocalization_metrics(self) -> str:
        row = getattr(
            getattr(self.ros_adapter, "health", None), "sample", {})
        values = []
        candidates = row.get("candidate_count")
        if isinstance(candidates, (int, float)) and candidates > 0:
            values.append(
                f"候选 {int(row.get('supported_candidates', 0))}/{int(candidates)}")
        rough = row.get("rough_fitness")
        if isinstance(rough, (int, float)) and rough >= 0:
            values.append(
                f"粗匹配 {rough:.3f}/{float(row.get('rough_threshold', -1)):.3f}")
        refine = row.get("refine_fitness")
        if isinstance(refine, (int, float)) and refine >= 0:
            values.append(
                f"精匹配 {refine:.3f}/{float(row.get('refine_threshold', -1)):.3f}")
        points = row.get("refine_points")
        if isinstance(points, (int, float)) and points > 0:
            values.append(f"有效点 {int(points)}")
        return " · ".join(values)

    def request_relocalization_timeout(
        self, timeout_s: int,
    ) -> Tuple[bool, str]:
        if self._manual_relocalize_pending or self._global_relocalize_pending:
            return False, "匹配进行中不能修改时限"
        request = getattr(
            self.ros_adapter, "request_relocalization_timeout", None)
        if request is None:
            return False, "当前定位后端不支持修改时限"
        accepted, message = request(timeout_s)
        self._set_status(message)
        return accepted, message

    def relocalization_timeout_readiness(
        self, timeout_s: int,
    ) -> Tuple[bool, str]:
        readiness = getattr(
            self.ros_adapter, "relocalization_timeout_readiness", None)
        if readiness is None:
            return (
                timeout_s == DEFAULT_RELOCALIZATION_TIMEOUT_S,
                "当前定位后端只支持默认 120 秒时限")
        return readiness(timeout_s)

    def reset_manual_relocalize_state(self) -> None:
        if not self._manual_relocalize_pending:
            self._manual_relocalize_state = "idle"

    def cancel_manual_relocalization(
        self, message: str = "定位服务已停止，请重新提交位置和方向",
    ) -> bool:
        if not self._manual_relocalize_pending:
            return False
        cancel = getattr(self.ros_adapter, "cancel_initial_pose_requests", None)
        cancelled = bool(cancel(message)) if cancel is not None else False
        if not cancelled and self._manual_relocalize_pending:
            self._on_manual_relocalize_done(False, message)
        return True

    def cancel_global_relocalization(
        self, message: str = "定位服务已停止，请重新发起自动匹配",
    ) -> bool:
        if not self._global_relocalize_pending:
            return False
        cancel = getattr(
            self.ros_adapter, "cancel_global_relocalization_requests", None)
        cancelled = bool(cancel(message)) if cancel is not None else False
        if not cancelled and self._global_relocalize_pending:
            self._on_global_relocalize_done(False, message)
        return True

    def initial_pose_readiness(
        self, frame_id: str, expected_map_sha256: str = "",
    ) -> Tuple[bool, str]:
        if self._manual_relocalize_pending:
            return False, "两点重定位正在匹配，请稍候"
        if self._global_relocalize_pending:
            return False, "自动重定位正在匹配，请稍候"
        return self.ros_adapter.initial_pose_readiness(
            frame_id, expected_map_sha256)

    def submit_initial_pose(
        self,
        frame_id: str,
        first,
        second,
        expected_map_sha256: str = "",
        minimum_distance: float = 0.30,
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        if self._manual_relocalize_pending:
            return False, "两点重定位正在匹配，请稍候"
        if self._global_relocalize_pending:
            return False, "自动重定位正在匹配，请稍候"
        try:
            timeout_s = validate_relocalization_timeout(timeout_s)
            x, y, yaw = initial_pose_from_points(
                first, second, minimum_distance)
        except ValueError as exc:
            message = str(exc)
            self._set_status(message)
            return False, message
        self._manual_relocalize_pending = True
        self._manual_relocalize_state = "pending"
        self._active_relocalization_started = time.monotonic()
        self._active_relocalization_timeout_s = timeout_s
        self.diagnostics.begin(
            "manual", timeout_s,
            frame_id=str(frame_id), map_sha256=str(expected_map_sha256),
            position=[x, y], direction=[float(second[0]), float(second[1])],
            yaw=yaw)
        self.operation_state_changed.emit("manual_relocalize", "starting")
        accepted, message = call_with_optional_timeout(
            self.ros_adapter.publish_initial_pose,
            frame_id, x, y, yaw, expected_map_sha256,
            self._on_manual_relocalize_done, timeout_s=timeout_s)
        if not accepted:
            self._manual_relocalize_pending = False
            self._manual_relocalize_state = "error"
            self.operation_state_changed.emit("manual_relocalize", "error")
            self._set_status(message)
            self._log(message)
            self.diagnostics.finish(False, message)
            self._active_relocalization_started = None
            return False, message
        self._set_status(message)
        self._log(message)
        return True, message

    def generate_scan_context(self, pcd_path: str) -> Tuple[bool, str]:
        if self.sc_generation_active:
            return False, "Scan Context 地图正在生成，请稍候"
        valid, value = validate_scan_context_source(pcd_path)
        if not valid:
            self._set_status(value)
            self._log(value)
            return False, value
        resolved_path = value
        self._expected_sc_path = scan_context_path(resolved_path)
        self._sc_cancel_requested = False
        if self.task_submit is not None:
            self._sc_request_token += 1
            token = self._sc_request_token
            finished = False
            def completed(value, error):
                nonlocal finished
                if finished or self._shutting_down or token != self._sc_request_token:
                    return
                finished = True
                self._sc_job_id = None
                if error:
                    cancelled = self._sc_cancel_requested
                    self._sc_cancel_requested = False
                    message = '定位索引生成已取消，原始点云已保留' if cancelled else str(error)
                    self.operation_state_changed.emit('scan_context', 'cancelled' if cancelled else 'error')
                else:
                    path = Path((value or {}).get('path') or self._expected_sc_path)
                    if path.is_file() and path.stat().st_size > 0:
                        self._expected_sc_path = path
                        message = f'定位索引已生成：{path}；登记时将核对点云身份'
                        self.operation_state_changed.emit('scan_context', 'success')
                    else:
                        message = '处理已结束，但没有找到定位索引，请查看处理详情'
                        self.operation_state_changed.emit('scan_context', 'error')
                    self._sc_cancel_requested = False
                self._set_status(message)
                self._log(message)
            try:
                job_id = self.task_submit('scan_context', {'cloud': resolved_path}, completed)
            except (OSError, ValueError, RuntimeError) as exc:
                message = str(exc) or '定位索引任务未能加入队列'
                self._set_status(message)
                return False, message
            if finished:
                return True, self.status
            self._sc_job_id = job_id
            if not job_id:
                return False, '定位索引任务未能加入队列'
            message = '定位索引任务已加入队列，可在处理进度中查看'
            self._set_status(message)
            self.operation_state_changed.emit('scan_context', 'queued')
            return True, message
        if self.admission_guard is not None:
            allowed, reason = self.admission_guard('scan_context')
            if not allowed:
                self._set_status(reason)
                return False, reason
        message = f"开始为 {Path(resolved_path).name} 生成 SC 地图"
        self._set_status(message)
        self._log(message)
        started = self.sc_process.start_launch([
            "run", "localizer", "generate_sc_tool", resolved_path,
        ])
        if not started:
            message = "Scan Context 生成工具未能启动"
            self._set_status(message)
            self._log(message)
            return False, message
        return True, message

    def stop_scan_context(self) -> Tuple[bool, str]:
        if self._sc_job_id:
            if self.task_cancel is None:
                return False, '请从地图处理进度中停止此任务'
            self._sc_cancel_requested = True
            try:
                accepted = self.task_cancel(self._sc_job_id)
            except (OSError, ValueError, RuntimeError) as exc:
                self._sc_cancel_requested = False
                return False, str(exc)
            if accepted is False:
                self._sc_cancel_requested = False
                return False, '任务状态已变化，请查看处理进度'
            if not self._sc_job_id:
                return True, self.status
            message = '正在结束定位索引任务，原始点云会保留'
            self._set_status(message)
            self.operation_state_changed.emit('scan_context', 'stopping')
            return True, message
        if not self.sc_process.active:
            return False, "当前没有正在生成的 SC 地图"
        self._sc_cancel_requested = True
        message = "正在停止 SC 地图生成…"
        self._set_status(message)
        self._log(message)
        self.sc_process.stop_launch()
        return True, message

    def trigger_global_relocalization(
        self, service_name: str = DEFAULT_GLOBAL_RELOCALIZE_SERVICE,
        timeout_s: int = DEFAULT_RELOCALIZATION_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        if self._manual_relocalize_pending:
            return False, "两点重定位正在匹配，请稍候"
        if self._global_relocalize_pending:
            return False, "全局重定位请求正在执行，请稍候"
        service_name = str(service_name).strip()
        try:
            timeout_s = validate_relocalization_timeout(timeout_s)
        except ValueError as exc:
            return False, str(exc)
        self._manual_relocalize_state = "idle"
        self._global_relocalize_pending = True
        self._active_relocalization_started = time.monotonic()
        self._active_relocalization_timeout_s = timeout_s
        self.diagnostics.begin(
            "global", timeout_s, service_name=service_name)
        self.operation_state_changed.emit("global_relocalize", "starting")
        accepted, message = call_with_optional_timeout(
            self.ros_adapter.trigger_global_relocalization,
            service_name, self._on_global_relocalize_done,
            timeout_s=timeout_s)
        if not accepted:
            self._global_relocalize_pending = False
            self.operation_state_changed.emit("global_relocalize", "error")
            self._set_status(message)
            self._log(message)
            self.diagnostics.finish(False, message)
            self._active_relocalization_started = None
            return False, message
        self._set_status(message)
        self._log(message)
        return True, message

    def _on_sc_state_changed(self, state: str) -> None:
        if self._shutting_down:
            return
        if state in ("starting", "running", "stopping"):
            self.operation_state_changed.emit("scan_context", state)
            return
        if self._sc_cancel_requested and state in ("stopped", "error"):
            self._sc_cancel_requested = False
            message = "SC 地图生成已停止，未将本次任务标记为成功"
            self.operation_state_changed.emit("scan_context", "cancelled")
        elif state == "error":
            message = "SC 地图生成失败，请查看定位日志"
            self.operation_state_changed.emit("scan_context", "error")
        elif (
            state == "stopped"
            and self._expected_sc_path is not None
            and self._expected_sc_path.is_file()
            and self._expected_sc_path.stat().st_size > 0
        ):
            message = (
                f"SC 地图已生成：{self._expected_sc_path}；"
                "登记地图包时将核对文件结构和 PCD 身份")
            self.operation_state_changed.emit("scan_context", "success")
        else:
            message = "生成进程已结束，但没有找到有效的定位缓存"
            self.operation_state_changed.emit("scan_context", "error")
        self._set_status(message)
        self._log(message)

    def _on_global_relocalize_done(self, success: bool, message: str) -> None:
        if self._shutting_down:
            return
        self._global_relocalize_pending = False
        self._active_relocalization_started = None
        self.diagnostics.finish(success, message)
        state = "success" if success else "error"
        self._set_status(message)
        self.operation_state_changed.emit("global_relocalize", state)
        self._log(message)

    def _on_manual_relocalize_done(self, success: bool, message: str) -> None:
        if self._shutting_down:
            return
        self._manual_relocalize_pending = False
        self._active_relocalization_started = None
        self.diagnostics.finish(success, message)
        self._manual_relocalize_state = "matched" if success else "error"
        state = "success" if success else "error"
        self._set_status(message)
        self.operation_state_changed.emit("manual_relocalize", state)
        self._log(message)

    def _set_status(self, message: str) -> None:
        self.status = str(message)
        self.status_changed.emit(self.status)

    def _log(self, message: str) -> None:
        self.log_received.emit(str(message).rstrip("\n") + "\n")

    def shutdown(self) -> None:
        self._shutting_down = True
        self.sc_process.shutdown()

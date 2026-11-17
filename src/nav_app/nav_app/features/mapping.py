"""Mapping backends, ROS adapter and lifecycle controller.

The UI depends on :class:`MappingController` instead of knowing how DLIO is
launched, which ROS types it consumes, or how its map is saved. A new mapping
algorithm can be registered by adding a :class:`MappingBackend` and a matching
save adapter when required.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional
import time

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from ..map_processing import pointcloud2_to_pointcloud
from ..map_storage import unique_map_file
from ..ros_launch_process import RosLaunchProcess
from ..latest_worker import LatestWorker
from .map_save import MapSaveLifecycle

try:
    from direct_lidar_inertial_odometry.srv import SavePCD as DlioSavePCD
except ImportError:  # UI remains available before the DLIO package is built.
    DlioSavePCD = None

try:
    from fast_livo.srv import SaveMap as FastLivoSaveMap
except ImportError:
    FastLivoSaveMap = None


@dataclass(frozen=True)
class MappingBackend:
    """Declarative contract between one SLAM package and the desktop app."""

    key: str
    label: str
    description: str
    package: str
    launch_file: str
    launch_arguments: Dict[str, str]
    default_pointcloud_topic: str
    default_imu_topic: str
    default_map_topic: str
    save_adapter: str = ""
    save_service: str = ""
    pointcloud_type: str = "sensor_msgs/msg/PointCloud2"


MAPPING_BACKENDS = {
    "fast_livo": MappingBackend(
        key="fast_livo", label="FAST-LIVO2",
        description="激光雷达、IMU 与相机联合建图；显示完整几何，有相机颜色的区域同步上色",
        package="fast_livo", launch_file="navigation.launch.py",
        launch_arguments={"pointcloud": "pointcloud_topic", "imu": "imu_topic",
                          "rviz": "rviz", "use_sim_time": "use_sim_time",
                          "image": "image_topic", "vision": "img_en", "params": "params_file",
                          "camera_params": "camera_params_file", "lidar_config": "lidar_config_file"},
        default_pointcloud_topic="/livox/lidar", default_imu_topic="/livox/imu",
        default_map_topic="/fast_livo/map_preview", save_adapter="fast_livo_save_map",
        save_service="/fast_livo/save_map",
        pointcloud_type="livox_ros_driver2/msg/CustomMsg",
    ),
    "dlio": MappingBackend(
        key="dlio",
        label="DLIO",
        description="Direct LiDAR-Inertial Odometry（激光雷达 + IMU）",
        package="direct_lidar_inertial_odometry",
        launch_file="dlio.launch.py",
        launch_arguments={
            "pointcloud": "pointcloud_topic",
            "imu": "imu_topic",
            "rviz": "rviz",
            "use_sim_time": "use_sim_time",
        },
        default_pointcloud_topic="/livox/lidar/pointcloud",
        default_imu_topic="/livox/imu",
        default_map_topic="/dlio/map_node/map",
        save_adapter="dlio_save_pcd",
        save_service="/save_pcd",
    ),
}


MAPPING_BACKENDS = {key: MAPPING_BACKENDS[key] for key in ("dlio", "fast_livo")}
MAPPING_BACKENDS["woosh_dlio"] = MappingBackend(
    key="woosh_dlio", label="DLIO · WOOSH .3",
    description="复用 WOOSH .3 点云和配对 IMU；自动完成 SI 单位及安装坐标适配，仅建图",
    package="nav_app", launch_file="woosh_mapping.launch.py",
    launch_arguments={"pointcloud": "pointcloud_topic", "imu": "imu_topic",
                      "rviz": "rviz", "use_sim_time": "use_sim_time"},
    default_pointcloud_topic="/foo/livox/lidar_192_168_128_3",
    default_imu_topic="/foo/livox/imu_192_168_128_3",
    default_map_topic="/dlio/map_node/map",
    save_adapter="dlio_save_pcd", save_service="/save_pcd",
)

MAPPING_BACKENDS["woosh_fast_livo"] = MappingBackend(
    key="woosh_fast_livo", label="FAST-LIVO2 · WOOSH .3 + 相机1",
    description="WOOSH .3 PointCloud2、配对 IMU 和相机1融合；使用机器人外参及实时相机内参，仅建图",
    package="nav_app", launch_file="woosh_fast_livo.launch.py",
    launch_arguments={"pointcloud": "pointcloud_topic", "imu": "imu_topic",
                      "image": "image_topic", "vision": "img_en", "params": "params_file",
                      "camera_params": "camera_params_file", "rviz": "rviz", "use_sim_time": "use_sim_time"},
    default_pointcloud_topic="/foo/livox/lidar_192_168_128_3",
    default_imu_topic="/foo/livox/imu_192_168_128_3",
    default_map_topic="/fast_livo/map_preview", save_adapter="fast_livo_save_map",
    save_service="/fast_livo/save_map",
)


MAPPING_BACKENDS["woosh_native5"] = MappingBackend(
    key="woosh_native5", label="FAST-LIVO2 · 本机 .5 + 相机1",
    description="启动/复用本机 .5 驱动服务，融合配对 IMU 和 /foo 相机1；使用 .5 安装外参，仅建图",
    package="nav_app", launch_file="woosh_native5.launch.py",
    launch_arguments={"pointcloud": "pointcloud_topic", "imu": "imu_topic",
                      "image": "image_topic", "vision": "img_en", "params": "params_file",
                      "camera_params": "camera_params_file", "rviz": "rviz", "use_sim_time": "use_sim_time"},
    default_pointcloud_topic="/livox/lidar_192_168_128_5",
    default_imu_topic="/livox/imu_192_168_128_5",
    default_map_topic="/fast_livo/map_preview", save_adapter="fast_livo_save_map",
    save_service="/fast_livo/save_map",
)


def unique_map_path(
    directory: Path, map_name: str, now=None,
) -> Path:
    """Return a timestamped PCD path that never replaces an existing map."""
    return unique_map_file(directory, map_name, ".pcd", now)


class MappingRosAdapter(MapSaveLifecycle):
    """Own all ROS subscriptions and services required by mapping features."""

    def __init__(self, node) -> None:
        self.node = node
        self.init_save_lifecycle()
        self.cloud: Optional[np.ndarray] = None
        self.cloud_colors: Optional[np.ndarray] = None
        self.cloud_color_source = ""
        self.cloud_frame = ""
        self.cloud_topic = ""
        self.cloud_error = ""
        self.cloud_revision = 0
        self.cloud_received_at = 0.
        self.save_status = ""
        self.save_revision = 0
        self.save_in_progress = False
        self.last_saved_path: Optional[Path] = None
        self._pending_save_path: Optional[Path] = None
        self._cloud_subscription = None
        self._geometry_subscription = None
        self._rgb_received_at = None
        self._cloud_epoch = 0
        self._cloud_worker = LatestWorker(lambda sample: pointcloud2_to_pointcloud(sample[0],max_points=120_000))
        self._cloud_timer = node.create_timer(.1,self.poll_cloud)
        self._fast_livo_save_client = (
            node.create_client(FastLivoSaveMap, "/fast_livo/save_map")
            if FastLivoSaveMap is not None else None)
        self._dlio_save_client = (
            node.create_client(DlioSavePCD, "/save_pcd")
            if DlioSavePCD is not None else None
        )

    def available_topics(self):
        grouped = {"pointcloud": [], "custom": [], "imu": [], "image": []}
        for topic_name, topic_types in self.node.get_topic_names_and_types():
            if "sensor_msgs/msg/PointCloud2" in topic_types:
                grouped["pointcloud"].append(topic_name)
            if "livox_ros_driver2/msg/CustomMsg" in topic_types:
                grouped["custom"].append(topic_name)
            if "sensor_msgs/msg/Image" in topic_types:
                grouped["image"].append(topic_name)
            if "sensor_msgs/msg/Imu" in topic_types:
                grouped["imu"].append(topic_name)
        for topics in grouped.values():
            topics.sort()
        return grouped

    def validate_inputs(self, pointcloud_topic: str, imu_topic: str, backend=None):
        graph = dict(self.node.get_topic_names_and_types())
        expected = (
            (pointcloud_topic, backend.pointcloud_type if backend else "sensor_msgs/msg/PointCloud2", "雷达"),
            (imu_topic, "sensor_msgs/msg/Imu", "IMU"),
        )
        for topic, message_type, label in expected:
            published_types = graph.get(topic, [])
            if published_types and message_type not in published_types:
                actual = ", ".join(published_types)
                return False, (
                    f"{label}话题 {topic} 类型不兼容：{actual}；"
                    f"当前算法需要 {message_type}")
        return True, "输入话题类型检查通过"

    def start_preview(self, topic: str):
        topic = str(topic).strip()
        # Migrate the previously persisted RGB-only selection to full coverage.
        if topic == "/fast_livo/map_rgb":
            topic = "/fast_livo/map_preview"
        if not topic:
            return False, "地图输出话题不能为空"
        self.stop_preview()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        try:
            epoch = self._cloud_epoch
            self._cloud_subscription = self.node.create_subscription(
                PointCloud2, topic, lambda msg: self._on_cloud(msg, epoch), qos)
            if topic == "/fast_livo/map_preview":
                # Geometry exists even when no camera-visible points can be
                # coloured. Do not block preview/saving on the RGB product.
                self._geometry_subscription = self.node.create_subscription(
                    PointCloud2, "/fast_livo/map",
                    lambda msg: self._on_geometry(msg, epoch), qos)
        except (RuntimeError, ValueError) as exc:
            self.stop_preview()
            self.cloud_error = str(exc)
            return False, f"地图点云订阅失败：{exc}"
        self.cloud = None
        self.cloud_colors = None
        self.cloud_color_source = ""
        self.cloud_frame = ""
        self.cloud_revision += 1
        self.cloud_topic = topic
        self.cloud_error = ""
        return True, f"等待建图数据 {topic}"

    def stop_preview(self) -> None:
        self._cloud_epoch += 1
        if self._cloud_subscription is not None:
            self.node.destroy_subscription(self._cloud_subscription)
            self._cloud_subscription = None
        if self._geometry_subscription is not None:
            self.node.destroy_subscription(self._geometry_subscription)
            self._geometry_subscription = None
        self._rgb_received_at = None
        self.cloud_topic = ""
        self._cloud_worker.clear()

    def _on_cloud(self, msg: PointCloud2, epoch=None) -> None:
        if self._cloud_subscription is not None and (epoch is None or epoch == self._cloud_epoch):
            if not msg.width * msg.height:
                return
            self._rgb_received_at = time.monotonic()
            self._cloud_worker.submit((msg,time.monotonic(),self._cloud_epoch))

    def _on_geometry(self, msg: PointCloud2, epoch: int) -> None:
        if (self._geometry_subscription is None or epoch != self._cloud_epoch or
                not msg.width * msg.height):
            return
        if self._rgb_received_at is not None and time.monotonic() - self._rgb_received_at < 3.:
            return
        self._cloud_worker.submit((msg, time.monotonic(), epoch))

    def poll_cloud(self):
        result=self._cloud_worker.take()
        if result is None:return
        (msg,received,epoch),cloud,error=result
        if not self.cloud_topic or epoch != self._cloud_epoch:return
        if error:
            self.cloud_error=f"建图点云格式错误：{error}"
        else:
            self.cloud = cloud.points
            self.cloud_colors = cloud.colors
            self.cloud_color_source = cloud.color_source
            self.cloud_frame = msg.header.frame_id
            self.cloud_error = ""
            self.cloud_revision += 1
            self.cloud_received_at = received

    def close(self):
        self.cancel_pending_save("建图服务已关闭，保存回执未完成；已落盘的数据保留，可在恢复未完成地图中检查")
        self.stop_preview()
        self._cloud_worker.close()
        self.node.destroy_timer(self._cloud_timer)
        self.cloud=self.cloud_colors=None

    def save_map(
        self,
        backend: MappingBackend,
        save_path: str,
        map_name: str,
        leaf_size: float,
    ):
        service_type, client = (FastLivoSaveMap, self._fast_livo_save_client) if (
            backend.save_adapter == "fast_livo_save_map") else (DlioSavePCD, self._dlio_save_client)
        if service_type is None or client is None:
            return False, f"未找到 {backend.label} 地图保存接口，请先构建并 source 工作空间"
        if self.save_in_progress:
            return False, "上一次地图仍在保存，请稍候"
        if (
            backend.save_adapter != "fast_livo_save_map"
            and (self.cloud is None or len(self.cloud) == 0)
        ):
            return False, "当前还没有可保存的地图点云，请等待建图产生有效点云"
        raw_path = str(save_path).strip()
        if not raw_path:
            return False, "请选择地图保存目录"
        target = Path(raw_path).expanduser()
        try:
            target.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return False, f"无法创建保存目录：{exc}"
        if (
            not client.service_is_ready()
            and not client.wait_for_service(timeout_sec=0.25)
        ):
            return False, (
                f"保存服务 {backend.save_service} 尚未就绪，请先启动建图")
        output_path = unique_map_path(target.resolve(), map_name)
        request = service_type.Request()
        request.leaf_size = float(leaf_size)
        request.save_path = str(output_path)
        try:
            attempt_id = self.begin_save(output_path,
                native_commit=backend.save_adapter == 'fast_livo_save_map', legacy_root=Path.home() / ".local/state/nav/map-spool")
        except (OSError, ValueError) as exc:
            return False, f'无法准备地图保存：{exc}'
        try:
            future = client.call_async(request)
        except (RuntimeError, TypeError, ValueError) as exc:
            status = f"调用地图保存服务失败：{exc}"
            self.fail_save_call(attempt_id, status)
            return False, status
        self.watch_save(future, output_path, attempt_id)
        return True, self.save_status


class UnavailableMappingRosAdapter:
    """Inert adapter for previews/tests that do not construct a ROS node."""

    cloud = None
    cloud_colors = None
    cloud_color_source = ""
    cloud_frame = ""
    cloud_topic = ""
    cloud_error = ""
    cloud_revision = 0
    cloud_received_at = 0.
    save_status = ""
    save_revision = 0
    save_in_progress = False
    last_saved_path = None

    def available_topics(self):
        return {"pointcloud": [], "imu": []}

    def validate_inputs(self, _pointcloud_topic: str, _imu_topic: str, backend=None):
        return False, "当前节点未启用建图 ROS 接口"

    def start_preview(self, _topic: str):
        return False, "当前节点未启用建图 ROS 接口"

    def stop_preview(self) -> None:
        pass

    def cancel_pending_save(self, reason):
        if self.save_in_progress:
            self.save_in_progress = False
            self.save_status = reason
            self.save_revision += 1

    def mapping_exited(self, reason):
        self.cancel_pending_save(reason)

    def save_map(
        self, _backend, _save_path: str, _map_name: str, _leaf_size: float,
    ):
        return False, "当前节点未启用建图保存接口"


class MappingController(QObject):
    """UI-independent mapping use cases and process lifecycle."""

    log_received = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    def __init__(
        self,
        ros_adapter: MappingRosAdapter,
        backend_key: str = "dlio",
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.ros = ros_adapter
        self.backend_key = backend_key
        self.start_guard = None
        self.configure_clock = None
        self._recovery_path = None
        self.admission_guard = None
        self.process = RosLaunchProcess(
            f"{self.backend.label} 建图", self)
        self.process.log_received.connect(self.log_received.emit)
        self.process.state_changed.connect(self._on_process_state)

    @property
    def backend(self) -> MappingBackend:
        return MAPPING_BACKENDS[self.backend_key]

    def select_backend(self, backend_key: str) -> MappingBackend:
        if backend_key not in MAPPING_BACKENDS:
            raise KeyError(f"Unknown mapping backend: {backend_key}")
        if self.process.running:
            raise RuntimeError("建图运行时不能切换算法")
        self.backend_key = backend_key
        self.process.display_name = f"{self.backend.label} 建图"
        return self.backend

    def available_topics(self):
        return self.ros.available_topics()

    def build_launch_arguments(
        self,
        pointcloud_topic: str,
        imu_topic: str,
        open_rviz: bool,
        use_sim_time: bool,
        image_topic: str = "/camera/camera/color/image_raw",
        params_file: str = "", camera_params_file: str = "",
        lidar_config_file: str = "",
        vision_enabled: bool = True,
    ):
        backend = self.backend
        values = {
            "image": image_topic.strip(), "vision": str(int(vision_enabled)), "params": params_file.strip(),
            "camera_params": camera_params_file.strip(),
            "lidar_config": lidar_config_file.strip(),
            "pointcloud": pointcloud_topic.strip(),
            "imu": imu_topic.strip(),
            "rviz": "true" if open_rviz else "false",
            "use_sim_time": "true" if use_sim_time else "false",
        }
        arguments = ["launch", backend.package, backend.launch_file]
        for role, launch_name in backend.launch_arguments.items():
            if launch_name and values.get(role, ""):
                arguments.append(f"{launch_name}:={values[role]}")
        return arguments

    def start(
        self,
        pointcloud_topic: str,
        imu_topic: str,
        map_topic: str,
        open_rviz: bool = False,
        use_sim_time: bool = False,
        image_topic: str = "/camera/camera/color/image_raw",
        params_file: str = "", camera_params_file: str = "",
        lidar_config_file: str = "",
        vision_enabled: bool = True,
    ):
        if self.admission_guard is not None:
            allowed, reason = self.admission_guard('mapping')
            if not allowed:
                return False, reason
        if self.start_guard is not None:
            allowed, reason = self.start_guard()
            if not allowed:
                return False, reason
        for path in (params_file, camera_params_file):
            if self.backend.save_adapter == "fast_livo_save_map" and path.strip() and not Path(path).expanduser().is_file():
                return False, f"FAST-LIVO2 配置文件不存在：{path}"
        pointcloud_topic = pointcloud_topic.strip()
        imu_topic = imu_topic.strip()
        map_topic = map_topic.strip()
        if not pointcloud_topic or not imu_topic or not map_topic:
            return False, "雷达、IMU 和地图输出话题都不能为空"
        compatible, reason = self.ros.validate_inputs(
            pointcloud_topic, imu_topic, self.backend)
        if not compatible:
            self.log_received.emit(reason + "\n")
            return False, reason
        if self.configure_clock is not None:
            self.configure_clock(use_sim_time)
        success, status = self.ros.start_preview(map_topic)
        if not success:
            return False, status
        started = self.process.start_launch(self.build_launch_arguments(
            pointcloud_topic, imu_topic, open_rviz, use_sim_time,
            image_topic, params_file, camera_params_file, lidar_config_file, vision_enabled))
        if not started:
            self.ros.stop_preview()
            return False, "建图进程已经在运行"
        return True, status

    def recover_map(self, checkpoint: str, destination: str):
        if self.process.active:
            return False, "请先停止建图或等待恢复完成"
        if self.start_guard is not None:
            allowed, reason = self.start_guard()
            if not allowed:
                return False, reason
        directory = Path(destination).expanduser()
        try:
            directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return False, f"无法创建恢复目录：{exc}"
        output = unique_map_path(directory.resolve(), "recovered_map")
        self._recovery_path = output
        self.process.display_name = "地图恢复"
        started = self.process.start_launch([
            "run", "fast_livo", "recover_map", checkpoint, str(output)])
        if not started:
            self._recovery_path = None
            return False, "地图恢复进程未能启动"
        return True, f"正在恢复已落盘地图：{output}"

    def stop(self) -> None:
        self.process.stop_launch()

    def save_map(self, save_path: str, map_name: str, leaf_size: float):
        if self._recovery_path is not None:
            return False, "地图正在恢复，请等待完成"
        if self.admission_guard is not None:
            allowed, reason = self.admission_guard('map_snapshot')
            if not allowed:
                return False, reason
        return self.ros.save_map(
            self.backend, save_path, map_name, leaf_size)

    def shutdown(self) -> None:
        self.process.shutdown()
        self.ros.stop_preview()

    def _on_process_state(self, state: str) -> None:
        if self._recovery_path is not None and state in ("stopped", "error"):
            output, self._recovery_path = self._recovery_path, None
            if state == "stopped" and output.is_file():
                self.ros.last_saved_path = output
                self.ros.save_status = f"地图恢复完成：{output}"
            else:
                self.ros.save_status = "地图恢复失败，原始检查点已保留，请查看建图日志"
            self.ros.save_revision += 1
            self.log_received.emit(self.ros.save_status + "\n")
            self.process.display_name = f"{self.backend.label} 建图"
        if state in ("stopped", "error"):
            # NAV_FAILURE can be emitted while the launch group (including
            # the save writer) is still alive. Preserve its pending request
            # until final process exit, then reconcile ACK/commit evidence.
            if not self.process.active:
                self.ros.mapping_exited("建图进程已停止，未收到点云保存回执；已落盘的数据保留，可在恢复未完成地图中检查")
            self.ros.stop_preview()
        self.state_changed.emit(state)

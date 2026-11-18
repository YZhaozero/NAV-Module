"""Ordered lifecycle control for the complete navigation runtime stack."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
import time
from typing import Callable, Dict, List, Sequence, Tuple

from PyQt5.QtCore import QObject, QProcess, QTimer, pyqtSignal

from ..ros_launch_process import RosLaunchProcess
from .log_store import log_store
from .navigation_backends import (
    LOCALIZATION_BACKENDS,
    NAVIGATION_BACKENDS,
    launch_bool,
)


COMPONENT_LABELS = {
    "lidar": "雷达驱动",
    "camera": "相机驱动",
    "dlio": "里程计",
    "laserscan": "点云转 LaserScan",
    "localizer": "定位后端",
    "nav2": "导航后端",
}

START_ORDER = ("lidar", "camera", "dlio", "laserscan", "localizer", "nav2")

LOCALIZER_TUNING_DEFAULTS = {
    "rough_score_thresh": 1.0,
    "refine_score_thresh": 0.1,
    "rough_scan_resolution": 0.2,
    "refine_scan_resolution": 0.05,
    "rough_map_resolution": 0.15,
    "refine_map_resolution": 0.15,
    "rough_max_iteration": 30,
    "refine_max_iteration": 100,
}


@dataclass
class NavigationStackConfig:
    """User-editable inputs required to launch the navigation stack."""

    navigation_map: str
    localization_map: str
    localization_backend: str = "pointcloud_localizer"
    navigation_backend: str = "nav2"
    lidar_arguments: Sequence[str] = field(default_factory=tuple)
    camera_arguments: Sequence[str] = field(default_factory=tuple)
    sensor_source: str = "live"
    lidar_driver_mode: str = "profile"
    camera_driver_mode: str = "profile"
    scan_source: str = "profile"
    external_odom_topic: str = "/odom"
    use_sim_time: bool = False
    autostart: bool = True
    robot_profile: str = ""
    odometry_backend: str = "dlio"
    fast_livo_lidar_topic: str = "/livox/lidar"
    fast_livo_image_topic: str = "/camera/camera/color/image_raw"
    woosh_fast_livo_profile: str = "livox3"
    fast_livo_params: str = ""
    fast_livo_camera_params: str = ""
    fast_livo_vision: bool = True
    dlio_rviz: bool = False
    dlio_pointcloud_topic: str = "/livox/lidar/pointcloud"
    dlio_imu_topic: str = "/livox/imu"
    laser_cloud_topic: str = "/livox/lidar/pointcloud"
    scan_topic: str = "/scan"
    laser_target_frame: str = "livox_frame"
    min_height: float = -1.0
    max_height: float = 0.1
    range_min: float = 0.45
    range_max: float = 10.0
    localizer_cloud_topic: str = "/dlio/odom_node/pointcloud/deskewed"
    localizer_odom_topic: str = "/dlio/odom_node/odom"
    localizer_config: str = ""
    localizer_tuning_enabled: bool = False
    rough_score_thresh: float = 1.0
    refine_score_thresh: float = 0.1
    rough_scan_resolution: float = 0.2
    refine_scan_resolution: float = 0.05
    rough_map_resolution: float = 0.15
    refine_map_resolution: float = 0.15
    rough_max_iteration: int = 30
    refine_max_iteration: int = 100
    scan_context_map: str = ""
    map_gravity_aligned: bool = False
    nav2_params: str = ""
    nav2_rviz: bool = False
    custom_localization_package: str = ""
    custom_localization_launch: str = ""
    custom_localization_arguments: str = ""
    custom_navigation_package: str = ""
    custom_navigation_launch: str = ""
    custom_navigation_arguments: str = ""
    scan_navi_mode: int = 1
    scan_sensor_type: str = "lidar"
    scan_body_pose_topic: str = "/dlio/odom_node/odom"
    scan_sensor_pose_topic: str = "/dlio/odom_node/odom"
    scan_cloud_topic: str = "/livox/lidar/pointcloud"
    scan_depth_topic: str = "/camera/aligned_depth_to_color/image_raw"
    scan_goal_topic: str = "/move_base_simple/goal"
    scan_initial_path_topic: str = "/initial_path"
    scan_cmd_vel_topic: str = "/cmd_vel"
    scan_start_controller: bool = True
    scan_cloud_is_world: bool = False
    scan_need_extrinsic: bool = False
    scan_planner_params: str = ""
    scan_controller_params: str = ""
    scan_keypoints_file: str = ""
    scan_reference_path_file: str = ""
    startup_interval_ms: int = 1000
    sensor_timeout_s: float = 12.0


def navigation_input_modes(config):
    """Resolve saved explicit choices or the existing robot preset."""
    woosh = config.robot_profile == "woosh"
    return dict(lidar=("external" if woosh else "nav") if config.lidar_driver_mode == "profile" else config.lidar_driver_mode,
                camera=("external" if woosh else "nav") if config.camera_driver_mode == "profile" else config.camera_driver_mode,
                scan=("external" if woosh else "convert") if config.scan_source == "profile" else config.scan_source)


def validate_navigation_stack(config: NavigationStackConfig, *, cancelled=None, progress=None) -> Tuple[bool, str]:
    """Validate files and coupled numeric parameters before any process starts."""
    modes = navigation_input_modes(config)
    for key, choices in (("lidar", ("nav", "external")),
                         ("camera", ("nav", "external", "off")),
                         ("scan", ("external", "convert"))):
        if modes[key] not in choices:
            return False, f"未知接入选项：{key}"
    if config.navigation_backend in ("nav2", "walking") and not config.scan_topic.strip():
        return False, "Nav2 的 LaserScan 话题不能为空"
    if config.odometry_backend == "external" and not config.external_odom_topic.strip():
        return False, "已有里程计话题不能为空"
    if config.odometry_backend == "fast_livo" and config.fast_livo_vision and modes["camera"] == "off":
        return False, "FAST-LIVO 视觉已启用，请选择相机输入或关闭视觉"
    if config.robot_profile not in ("", "woosh"):
        return False, "未知机器人接入配置"
    if config.robot_profile == "woosh":
        if config.woosh_fast_livo_profile not in ("livox3", "native5"):
            return False, "未知 WOOSH 雷达安装配置"
        if config.woosh_fast_livo_profile == "native5" and config.odometry_backend == "dlio":
            return False, "原生 .5 安装预设请使用 FAST-LIVO2；DLIO 尚未接入这套安装参数"
    if config.sensor_source not in ("live", "bag"):
        return False, "未知传感器数据源"
    if config.sensor_source == "bag" and not config.use_sim_time:
        return False, "rosbag 数据源必须启用 /clock 仿真时间"
    if config.odometry_backend not in ("dlio", "fast_livo", "external"):
        return False, "未知里程计后端"
    if config.odometry_backend == "fast_livo":
        from pathlib import Path
        for path in (config.fast_livo_params, config.fast_livo_camera_params):
            if path.strip() and not Path(path).expanduser().is_file():
                return False, f"FAST-LIVO2 配置文件不存在：{path}"
        if not config.fast_livo_lidar_topic.strip() or not config.fast_livo_image_topic.strip():
            return False, "FAST-LIVO2 雷达与相机话题不能为空"
    localization = LOCALIZATION_BACKENDS.get(config.localization_backend)
    if localization is None:
        return False, f"未知定位后端：{config.localization_backend}"
    navigation = NAVIGATION_BACKENDS.get(config.navigation_backend)
    if navigation is None:
        return False, f"未知导航后端：{config.navigation_backend}"
    options = dict(cancelled=cancelled, progress=progress) if config.localization_backend == "pointcloud_localizer" else {}
    valid, message = localization.validate(config, **options)
    if not valid:
        return valid, message
    if (
        config.localization_backend == "pointcloud_localizer"
        and config.localizer_tuning_enabled
    ):
        positive_numbers = {
            "粗匹配评分阈值": config.rough_score_thresh,
            "精匹配评分阈值": config.refine_score_thresh,
            "粗匹配点云分辨率": config.rough_scan_resolution,
            "精匹配点云分辨率": config.refine_scan_resolution,
            "粗匹配地图分辨率": config.rough_map_resolution,
            "精匹配地图分辨率": config.refine_map_resolution,
        }
        for label, value in positive_numbers.items():
            try:
                valid_number = math.isfinite(float(value)) and float(value) > 0
            except (TypeError, ValueError, OverflowError):
                valid_number = False
            if not valid_number:
                return False, f"{label}必须是大于 0 的有效数字"
        for label, value in (
            ("粗匹配最大迭代次数", config.rough_max_iteration),
            ("精匹配最大迭代次数", config.refine_max_iteration),
        ):
            if type(value) is not int or value < 1:
                return False, f"{label}必须是正整数"
    valid, message = navigation.validate(config)
    if not valid:
        return valid, message
    if modes["scan"] == "convert" and config.min_height >= config.max_height:
        return False, "LaserScan 最低高度必须小于最高高度"
    if modes["scan"] == "convert" and (config.range_min < 0.0 or config.range_min >= config.range_max):
        return False, "LaserScan 最小量程必须小于最大量程"
    if config.sensor_source == "live":
        if (modes["lidar"] == "nav" and not config.lidar_arguments
                and not (config.robot_profile == "woosh" and config.woosh_fast_livo_profile == "native5")):
            return False, "雷达启动参数不完整"
        if modes["camera"] == "nav" and not config.camera_arguments:
            return False, "相机启动参数不完整"
    return True, "启动参数检查通过"


def build_navigation_launches(
    config: NavigationStackConfig,
) -> Dict[str, List[str]]:
    """Build component commands; native .5 acquisition uses its shared service."""
    native5 = config.robot_profile == "woosh" and config.woosh_fast_livo_profile == "native5"
    if config.odometry_backend == "fast_livo":
        # Switch dependent contracts together: a world cloud is not a body cloud.
        config = replace(config,
            localizer_cloud_topic="/fast_livo/cloud_body",
            localizer_odom_topic="/aft_mapped_to_init",
            laser_cloud_topic="/nav/livox5/points_filtered" if native5 else "/fast_livo/cloud_body",
            laser_target_frame="base_footprint" if native5 else "base_link",
            scan_body_pose_topic="/aft_mapped_to_init",
            scan_sensor_pose_topic="/aft_mapped_to_init",
            scan_cloud_topic="/fast_livo/cloud_body", scan_cloud_is_world=False,
            scan_need_extrinsic=False)
    launches = {
        "lidar": list(config.lidar_arguments),
        "camera": list(config.camera_arguments),
        "dlio": [
            "launch", "direct_lidar_inertial_odometry", "dlio.launch.py",
            f"rviz:={launch_bool(config.dlio_rviz)}",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
            f"pointcloud_topic:={config.dlio_pointcloud_topic.strip()}",
            f"imu_topic:={config.dlio_imu_topic.strip()}",
        ],
        "laserscan": [
            "launch", "pointcloud_to_laserscan",
            "pointcloud_to_laserscan_launch.py",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
            f"cloud_topic:={config.laser_cloud_topic.strip()}",
            f"scan_topic:={config.scan_topic.strip()}",
            f"target_frame:={config.laser_target_frame.strip()}",
            f"min_height:={config.min_height:g}",
            f"max_height:={config.max_height:g}",
            f"range_min:={config.range_min:g}",
            f"range_max:={config.range_max:g}",
        ],
    }
    if native5:
        launches["lidar"] = ["systemctl", "--user", "start", "nav-livox5.service"]
        launches["laserscan"][1:3] = ["nav_app", "woosh_livox5_scan.launch.py"]
        launches["laserscan"].append("scan_time:=0.1")
    if config.sensor_source == "bag" or (native5 and config.use_sim_time):
        launches.pop("lidar", None)
        launches.pop("camera", None)
    if config.odometry_backend == "fast_livo":
        launches["dlio"] = [
            "launch", "fast_livo", "navigation.launch.py",
            f"pointcloud_topic:={config.fast_livo_lidar_topic}",
            f"imu_topic:={config.dlio_imu_topic}", f"image_topic:={config.fast_livo_image_topic}",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
            f"img_en:={int(config.fast_livo_vision)}", f"rviz:={launch_bool(config.dlio_rviz)}",
        ]
        for argument, value in (("params_file", config.fast_livo_params),
                                ("camera_params_file", config.fast_livo_camera_params)):
            if value.strip(): launches["dlio"].append(f"{argument}:={value.strip()}")
        # Driver format 4 publishes BOTH CustomMsg and PointCloud2 in this workspace.
        if config.sensor_source == "live" and config.robot_profile != "woosh":
            driver_path = next((a.split(":=", 1)[1] for a in config.lidar_arguments
                                if a.startswith("user_config_path:=")), "")
            if driver_path:
                launches["dlio"].append(f"lidar_config_file:={driver_path}")
            launches["lidar"] = [
                a for a in launches["lidar"]
                if not a.startswith("xfer_format:=")]
            launches["lidar"].append("xfer_format:=4")
            launches["camera"] = [
                a for a in launches["camera"]
                if not a.startswith("color_profile:=")]
            launches["camera"].append("color_profile:=1280x720x15")
    if config.robot_profile == "woosh":
        fast_launch = "woosh_native5.launch.py" if config.woosh_fast_livo_profile == "native5" else "woosh_fast_livo.launch.py"
        if config.odometry_backend == "dlio":
            config = replace(config, localizer_cloud_topic="/dlio/odom_node/pointcloud/body",
                             localizer_odom_topic="/dlio/odom_node/scan_odom")
        launches["dlio"] = [
            "launch", "nav_app",
            fast_launch if config.odometry_backend == "fast_livo" else "woosh_mapping.launch.py",
            f"pointcloud_topic:={(config.fast_livo_lidar_topic if config.odometry_backend == 'fast_livo' else config.dlio_pointcloud_topic).strip()}",
            f"imu_topic:={config.dlio_imu_topic.strip()}",
            f"rviz:={launch_bool(config.dlio_rviz)}",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
        ]
        if config.odometry_backend == "fast_livo":
            launches["dlio"] += [f"image_topic:={config.fast_livo_image_topic}",
                                 f"img_en:={int(config.fast_livo_vision)}",
                                 "imu_rate_odom:=true"]
            if config.woosh_fast_livo_profile == "native5":
                launches["dlio"].append("start_driver:=false")
            for argument, value in (("params_file", config.fast_livo_params),
                                    ("camera_params_file", config.fast_livo_camera_params)):
                if value.strip():
                    launches["dlio"].append(f"{argument}:={value.strip()}")
    modes = navigation_input_modes(config)
    for key in ("lidar", "camera"):
        if modes[key] != "nav":
            launches.pop(key, None)
    if modes["scan"] == "external":
        launches.pop("laserscan", None)
    if config.odometry_backend == "external":
        launches.pop("dlio", None)
    localization = LOCALIZATION_BACKENDS[config.localization_backend]
    localization_launch = localization.launch_arguments(config)
    if localization_launch:
        launches["localizer"] = localization_launch
    navigation = NAVIGATION_BACKENDS[config.navigation_backend]
    navigation_launch = navigation.launch_arguments(config)
    if navigation_launch:
        launches["nav2"] = navigation_launch
    return launches


class NavigationStackController(QObject):
    """Start dependent launch files in order and stop managed processes safely."""

    log_received = pyqtSignal(str, str)
    component_state_changed = pyqtSignal(str, str)
    overall_state_changed = pyqtSignal(str)
    input_waiting = pyqtSignal(str)

    def __init__(
        self,
        sensor_controller,
        sensor_active: Callable[[str], bool] | None = None,
        parent=None,
        component_ready=None,
    ) -> None:
        super().__init__(parent)
        self.component_ready = component_ready
        self.start_guard = None
        self.configure_navigation = None
        self.navigation_log = None
        self._readiness_components = {"dlio", "nav2"}
        self.sensor_controller = sensor_controller
        self.sensor_active = sensor_active or (lambda _key: False)
        self.processes: Dict[str, RosLaunchProcess] = {}
        self.states = {key: "stopped" for key in START_ORDER}
        self.overall_state = "stopped"
        self._generation = 0
        self._lidar_service_start = None
        self._owns_native5_service = False
        self._lidar_service_stop = QProcess(self)
        self._lidar_service_stop.setProcessChannelMode(QProcess.MergedChannels)
        self._lidar_service_stop.finished.connect(self._native5_stop_finished)
        self._lidar_service_stop.errorOccurred.connect(self._native5_stop_error)
        self._launches: Dict[str, List[str]] = {}
        self._queue: List[str] = []
        self._waiting_key = ""
        self._owned_components = set()
        self._stop_targets = set()
        self._stopping = False
        self._failed_component = ""
        self._startup_interval_ms = 1000
        self._sensor_timeout_s = 12.0
        self._sensor_wait_deadline = 0.0
        self._sensor_source = "live"
        self._required_sensors = {"lidar", "camera"}
        self._external_inputs = {}
        self.input_modes = {}
        self.active_config = None
        self._waiting_input_message = ""
        self.log_events = []

        for key in ("dlio", "laserscan", "localizer", "nav2"):
            process = RosLaunchProcess(COMPONENT_LABELS[key], self)
            process.log_received.connect(
                lambda text, component=key: self._record_log(
                    component, text))
            process.state_changed.connect(
                lambda state, component=key: self._handle_state(
                    component, state))
            self.processes[key] = process
        self.sensor_controller.log_received.connect(self._record_log)
        self.sensor_controller.state_changed.connect(self._handle_state)

    def start(self, config: NavigationStackConfig) -> Tuple[bool, str]:
        if self.start_guard is not None:
            allowed, reason = self.start_guard()
            if not allowed:
                return False, reason
        if self._queue or self._waiting_key or self.overall_state in (
            "starting", "running", "stopping",
        ):
            return False, "导航系统已经在启动或运行"
        if any(self._process_active(process) for process in self.processes.values()):
            return False, "仍有导航组件在运行，请先停止整套系统再重新启动"
        valid, message = validate_navigation_stack(config)
        if not valid:
            return False, message
        self._generation += 1
        if self.configure_navigation is not None:self.configure_navigation(config)
        self._sensor_source = config.sensor_source
        self.active_config = config
        self._waiting_input_message = ""
        self.input_modes = navigation_input_modes(config)
        self._required_sensors = set()
        if config.odometry_backend != "external" or self.input_modes["scan"] == "convert":
            self._required_sensors.add("lidar")
        if config.odometry_backend == "fast_livo" and config.fast_livo_vision:
            self._required_sensors.add("camera")
        self._external_inputs = {}
        if config.navigation_backend in ("nav2", "walking") and config.sensor_source == "live":
            self._external_inputs["scan"] = "LaserScan " + config.scan_topic
            if config.odometry_backend == "external":
                self._external_inputs["external_odom"] = "已有里程计 " + config.external_odom_topic + " 及 odom→base_link TF"
            if config.localization_backend == "external":
                self._external_inputs["external_localization"] = "已有定位 map→odom TF"
        # During replay the player is intentionally paused until every
        # subscriber is up. Waiting for odometry here would deadlock startup.
        self._readiness_components = (
            set() if config.sensor_source == "bag" or config.odometry_backend == "external" else {"dlio"})
        if config.localization_backend == "pointcloud_localizer":
            self._readiness_components.add("localizer")
        if config.sensor_source == "live" and config.navigation_backend in ("nav2", "walking"):
            self._readiness_components.add("nav2")
        self._launches = build_navigation_launches(config)
        self._queue = [key for key in START_ORDER if key in self._launches]
        self._waiting_key = ""
        self._owned_components.clear()
        self._stopping = False
        self._failed_component = ""
        self._startup_interval_ms = max(0, int(config.startup_interval_ms))
        self._sensor_timeout_s = max(1.0, float(config.sensor_timeout_s))
        for key in START_ORDER:
            if key not in self._launches:
                external = ((key in ("lidar", "camera") and self.input_modes[key] == "external")
                            or (key == "laserscan" and self.input_modes["scan"] == "external")
                            or (key == "dlio" and config.odometry_backend == "external")
                            or (key == "localizer" and config.localization_backend == "external"))
                self.states[key] = "external" if external else (
                    "nav2_managed" if key == "localizer" and config.localization_backend == "amcl" else "disabled")
                self.component_state_changed.emit(key, self.states[key])
        self._set_overall_state("starting")
        source_label = "rosbag" if config.sensor_source == "bag" else "实机传感器"
        self._record_log(
            "system", f"开始按依赖顺序启动导航系统（数据源：{source_label}）…\n")
        self._start_next()
        return True, "导航系统开始启动"

    def _component_running(self, key: str) -> bool:
        if key in ("lidar", "camera"):
            return bool(self.sensor_active(key))
        return self.processes[key].running

    def _later(self, delay, callback):
        generation = self._generation
        QTimer.singleShot(delay, lambda: callback() if generation == self._generation else None)

    def _start_next(self) -> None:
        if self._stopping or self.overall_state != "starting":
            return
        if not self._queue:
            self._waiting_key = ""
            self._record_log("system", "导航系统全部组件已启动。\n")
            self._set_overall_state("running")
            self._later(1000, self._monitor_sensor_health)
            return
        if self._queue[0] == "nav2" and self.component_ready is not None and self._external_inputs:
            self._poll_external_inputs(time.monotonic() + self._sensor_timeout_s)
            return
        self._launch_next_component()

    def _poll_external_inputs(self, deadline):
        if self._stopping or self.overall_state != "starting":
            return
        missing = [label for key, label in self._external_inputs.items() if not self.component_ready(key)]
        if not missing:
            self._waiting_input_message = ""
            self.input_waiting.emit("输入已就绪，正在启动 Nav2")
            self._launch_next_component()
            return
        message = "等待输入：" + "、".join(missing)
        if message != self._waiting_input_message:
            self._waiting_input_message = message
            self._record_log("system", message + "\n")
            self.input_waiting.emit(message)
        if self.states["nav2"] != "waiting":
            self.states["nav2"] = "waiting"
            self.component_state_changed.emit("nav2", "waiting")
        if time.monotonic() >= deadline and self.active_config.navigation_backend != "walking":
            self._record_log("system", message + "；输入超时，未启动 Nav2。\n")
            self._abort_for_failure("nav2")
            return
        self._later(200, lambda: self._poll_external_inputs(deadline))

    def _launch_next_component(self):
        key = self._queue.pop(0)
        if key == "lidar" and self._launches[key] == ["systemctl", "--user", "start", "nav-livox5.service"]:
            self._owns_native5_service = True
            self._waiting_key = key
            self._start_native5_service()
            return
        if self._component_running(key):
            self.states[key] = "running"
            self.component_state_changed.emit(key, "running")
            message = f"{COMPONENT_LABELS[key]}已有真实数据，跳过重复启动。\n"
            self._record_log("system", message)
            if key in ("lidar", "camera"):
                self._record_sensor_log(key, message)
            self._later(self._startup_interval_ms, self._start_next)
            return

        if key in ("lidar", "camera"):
            if self.sensor_controller.processes[key].running:
                if key not in self._required_sensors:
                    self.states[key] = "running"
                    self.component_state_changed.emit(key, "running")
                    self._later(self._startup_interval_ms, self._start_next)
                    return
                self._waiting_key = key
                self._begin_sensor_wait(key, started_here=False)
                return

        self._waiting_key = key
        self._record_log(
            "system", f"正在启动 {COMPONENT_LABELS[key]}…\n")
        if (key == "lidar" and self.active_config.robot_profile == "woosh"
                and self.active_config.woosh_fast_livo_profile == "native5"):
            self._start_native5_service()
            return
        if key in ("lidar", "camera"):
            started = self.sensor_controller.start(key, self._launches[key])
        else:
            started = self.processes[key].start_launch(self._launches[key])
        if started:
            self._owned_components.add(key)
            return
        if key == "camera" and key not in self._required_sensors:
            self._waiting_key = ""
            self.states[key] = "error"
            self.component_state_changed.emit(key, "error")
            self._record_log("system", "预览相机未启动，继续导航；当前算法不依赖相机。\n")
            self._later(self._startup_interval_ms, self._start_next)
            return
        self._waiting_key = ""
        self._queue.clear()
        self._record_log(
            "system", f"{COMPONENT_LABELS[key]}未能启动，启动流程中止。\n")
        self._abort_for_failure(key)

    def _start_native5_service(self) -> None:
        # systemctl starts/reuses the shared acquisition service, never a second SDK.
        process = QProcess(self)
        self._lidar_service_start = process
        generation = self._generation
        process.setProcessChannelMode(QProcess.MergedChannels)
        self.states["lidar"] = "starting"
        self.component_state_changed.emit("lidar", "starting")
        self._record_log("lidar", "正在启动 / 复用 nav-livox5.service…\n")

        def finished(code, status):
            if generation != self._generation or self._lidar_service_start is not process:
                return
            self._lidar_service_start = None
            self._record_log("lidar", bytes(process.readAllStandardOutput()).decode(errors="replace"))
            if code == 0 and status == QProcess.NormalExit:
                self._begin_sensor_wait("lidar", started_here=True)
            else:
                self._record_log("lidar", f".5 雷达服务启动失败（退出码 {code}），导航启动中止。\n")
                self._abort_for_failure("lidar")

        def failed(error):
            if error == QProcess.FailedToStart:
                if generation == self._generation and self._lidar_service_start is process:
                    self._record_log("lidar", f".5 雷达服务无法启动：{process.errorString()}\n")
                    self._lidar_service_start = None
                    self._abort_for_failure("lidar")
                process.deleteLater()

        def timed_out():
            if self._lidar_service_start is process:
                self._record_log("lidar", ".5 雷达服务启动超时，导航启动中止。\n")
                self._abort_for_failure("lidar")

        process.finished.connect(finished)
        process.finished.connect(process.deleteLater)
        process.errorOccurred.connect(failed)
        command = self._launches["lidar"]
        process.start(command[0], command[1:])
        self._later(15000, timed_out)

    def _poll_component_ready(self, key: str, deadline: float) -> None:
        if self._stopping or self.overall_state != "starting" or self._waiting_key != key:
            return
        if self.component_ready(key):
            self._waiting_key = ""
            self.states[key] = "running"
            self.component_state_changed.emit(key, "running")
            self._later(self._startup_interval_ms, self._start_next)
        elif time.monotonic() >= deadline and self.active_config.navigation_backend == "walking":
            message = ("等待地图定位，请点击重新定位；已启动的组件继续运行"
                       if key == "localizer" else f"等待{COMPONENT_LABELS[key]}有效数据；已启动的组件继续运行")
            if message != self._waiting_input_message:
                self._waiting_input_message = message
                self._record_log("system", message + "\n")
                self.input_waiting.emit(message)
            self._later(500, lambda: self._poll_component_ready(key, deadline))
        elif time.monotonic() >= deadline:
            self._queue.clear()
            self._waiting_key = ""
            self.states[key] = "error"
            self.component_state_changed.emit(key, "error")
            self._record_log("system", f"{COMPONENT_LABELS[key]}已启动，但未收到有效输出，启动中止。\n")
            self._abort_for_failure(key)
        else:
            self._later(100, lambda: self._poll_component_ready(key, deadline))

    def _handle_state(self, key: str, state: str) -> None:
        if (self.component_ready is not None and key in self._readiness_components
                and key == self._waiting_key and state == "running" and not self._stopping):
            self.states[key] = "waiting"
            self.component_state_changed.emit(key, "waiting")
            minimum_timeout = 120.0 if key in ("localizer", "nav2") else 30.0
            self._poll_component_ready(
                key,
                time.monotonic() + max(minimum_timeout, self._sensor_timeout_s),
            )
            return
        if key not in self.states:
            return
        if (key == "camera" and key not in self._required_sensors
                and state in ("error", "stopped") and not self._stopping):
            self.states[key] = state
            self.component_state_changed.emit(key, state)
            if self._waiting_key == key:
                self._waiting_key = ""
                self._later(self._startup_interval_ms, self._start_next)
            return
        if all((
            key == self._waiting_key,
            key in ("lidar", "camera"),
            key in self._required_sensors,
            state == "running",
        )):
            self._begin_sensor_wait(key, started_here=True)
            return
        self.states[key] = state
        self.component_state_changed.emit(key, state)
        if self._stopping:
            if state in ("stopped", "error"):
                self._finish_stop_if_ready()
            return
        if key == self._waiting_key and state == "running":
            self._waiting_key = ""
            self._later(self._startup_interval_ms, self._start_next)
        elif key == self._waiting_key and state in ("error", "stopped"):
            self._waiting_key = ""
            self._queue.clear()
            self._record_log(
                "system", f"{COMPONENT_LABELS[key]}启动失败，流程中止。\n")
            self._abort_for_failure(key)
        elif all((
            self.overall_state in ("starting", "running"),
            key in self._owned_components,
            state in ("error", "stopped"),
        )):
            self._queue.clear()
            self._waiting_key = ""
            self._record_log(
                "system", f"{COMPONENT_LABELS[key]}意外退出。\n")
            self._abort_for_failure(key)

    def _abort_for_failure(self, key: str) -> None:
        self.states[key] = "error"
        self.component_state_changed.emit(key, "error")
        self._failed_component = key
        self._queue.clear()
        self._waiting_key = ""
        self._set_overall_state("error")
        # Stop dependent navigation processes instead of continuing with stale
        # odometry. Drivers already owned by somebody else remain untouched.
        self.stop(owned_only=True)

    def _begin_sensor_wait(self, key: str, started_here: bool) -> None:
        self.states[key] = "waiting"
        self.component_state_changed.emit(key, "waiting")
        self._sensor_wait_deadline = time.monotonic() + self._sensor_timeout_s
        origin = "驱动已启动" if started_here else "检测到已有驱动进程"
        message = f"{COMPONENT_LABELS[key]}{origin}，等待真实传感器数据…\n"
        self._record_log("system", message)
        self._record_sensor_log(key, message)
        self._later(100, lambda: self._poll_sensor_data(key))

    def _poll_sensor_data(self, key: str) -> None:
        if any((
            self._stopping,
            self.overall_state != "starting",
            self._waiting_key != key,
        )):
            return
        if self.sensor_active(key):
            self._waiting_key = ""
            self.states[key] = "running"
            self.component_state_changed.emit(key, "running")
            message = f"{COMPONENT_LABELS[key]}已收到真实数据。\n"
            self._record_log("system", message)
            self._record_sensor_log(key, message)
            self._later(self._startup_interval_ms, self._start_next)
            return
        if time.monotonic() >= self._sensor_wait_deadline:
            self._waiting_key = ""
            self._queue.clear()
            self.states[key] = "error"
            self.component_state_changed.emit(key, "error")
            message = (
                f"{COMPONENT_LABELS[key]}在 {self._sensor_timeout_s:g} 秒内"
                "没有收到数据，请检查供电、网络和设备连接。\n")
            self._record_log("system", message)
            self._record_sensor_log(key, message)
            self._abort_for_failure(key)
            return
        self._later(200, lambda: self._poll_sensor_data(key))

    def _monitor_sensor_health(self) -> None:
        if self.overall_state != "running":
            return
        if self._sensor_source == "bag":
            return
        if self.active_config is not None and self.active_config.navigation_backend == "walking":
            checks = [(key, self.sensor_active(key), COMPONENT_LABELS[key]) for key in self._required_sensors]
            if self.component_ready is not None:
                checks += [(key, self.component_ready(key), COMPONENT_LABELS[key])
                           for key in self._readiness_components]
                checks += [(key, self.component_ready(key), label) for key, label in self._external_inputs.items()]
            missing = [label for key, ready, label in checks if not ready]
            for key, ready, label in checks:
                if key in self.states:
                    state = "running" if ready else "waiting"
                    if self.states[key] != state:
                        self.states[key] = state
                        self.component_state_changed.emit(key, state)
            message = "等待数据：" + "、".join(missing) + "；路线规划已暂停" if missing else "路线规划已就绪；使用手柄控制行走"
            if message != self._waiting_input_message:
                self._waiting_input_message = message
                self._record_log("system", message + "\n")
                self.input_waiting.emit(message)
            self._later(1000, self._monitor_sensor_health)
            return
        for key in self._required_sensors:
            if not self.sensor_active(key):
                self.states[key] = "error"
                self.component_state_changed.emit(key, "error")
                message = f"{COMPONENT_LABELS[key]}数据已中断。\n"
                self._record_log("system", message)
                self._record_sensor_log(key, message)
                self._abort_for_failure(key)
                return
        if self.component_ready is not None:
            if self.active_config is not None and self.active_config.odometry_backend != "external" and not self.component_ready("dlio"):
                self._record_log("system", "里程计输出已失效，停止受管导航组件。\n")
                self._abort_for_failure("dlio")
                return
            for key, label in self._external_inputs.items():
                if not self.component_ready(key):
                    self._record_log("system", f"{label} 已失效，停止受管导航组件。\n")
                    self._abort_for_failure("nav2")
                    return
        self._later(1000, self._monitor_sensor_health)

    def stop(self, owned_only: bool = False) -> None:
        if self._stopping:
            return
        if self.configure_navigation is not None:self.configure_navigation(None)
        self._generation += 1
        if self._lidar_service_start is not None:
            self._lidar_service_start.kill()
            self._lidar_service_start = None
        if not owned_only:
            self._failed_component = ""
        self._queue.clear()
        self._waiting_key = ""
        targets = set(self._owned_components)
        for key in START_ORDER:
            if (owned_only or key in ("lidar", "camera")) and key not in self._owned_components:
                continue
            process = self._component_process(key)
            if self._process_active(process):
                targets.add(key)
        self._stop_targets = targets
        if not targets and not self._owns_native5_service:
            self._stopping = False
            self._record_log("system", "没有仍在运行的受管组件需要停止。\n")
            self._set_overall_state("error" if self._failed_component else "stopped")
            return
        self._stopping = True
        self._set_overall_state("stopping")
        self._record_log("system", "正在按反向顺序停止导航系统…\n")
        for key in reversed(START_ORDER):
            if key not in targets or not self._process_active(self._component_process(key)):
                continue
            if key in ("lidar", "camera"):
                self.sensor_controller.stop(key)
            else:
                self.processes[key].stop_launch()
        self._finish_stop_if_ready()

    def _finish_stop_if_ready(self) -> None:
        if not self._stopping:
            return
        if any(
            self._process_active(process)
            for process in (
                self._component_process(key) for key in self._stop_targets)
        ):
            return
        if self._owns_native5_service:
            if self._lidar_service_stop.state() == QProcess.NotRunning:
                self._stop_targets.add("lidar")
                self._record_log("system", "导航算法已退出，正在关闭 .5 雷达驱动。\n")
                self._lidar_service_stop.start("systemctl", ["--user", "stop", "nav-livox5.service"])
            return
        self._owned_components.clear()
        stopped_targets = set(self._stop_targets)
        self._stop_targets.clear()
        self._stopping = False
        for key in stopped_targets:
            self.states[key] = "error" if key == self._failed_component else "stopped"
            self.component_state_changed.emit(key, self.states[key])
        self._record_log("system", "导航系统进程已全部停止。\n")
        self._set_overall_state("error" if self._failed_component else "stopped")

    def _native5_stop_error(self, error):
        if error == QProcess.FailedToStart:
            self._native5_stop_finished(-1, QProcess.CrashExit)

    def _native5_stop_finished(self, code, status):
        if code or status != QProcess.NormalExit:
            detail = bytes(self._lidar_service_stop.readAllStandardOutput()).decode(errors="replace").strip()
            self._record_log("system", f".5 雷达驱动停止失败：{detail or self._lidar_service_stop.errorString()}\n")
            self._stopping = False
            self.states["lidar"] = "error"
            self.component_state_changed.emit("lidar", "error")
            self._set_overall_state("error")
            return
        self._owns_native5_service = False
        self._record_log("system", ".5 雷达驱动已停止。\n")
        self._finish_stop_if_ready()

    def _component_process(self, key: str):
        if key in ("lidar", "camera"):
            return self.sensor_controller.processes[key]
        return self.processes[key]

    @staticmethod
    def _process_active(process) -> bool:
        """Support real process wrappers and lightweight test doubles."""
        return bool(getattr(process, "active", getattr(process, "running", False)))

    def _set_overall_state(self, state: str) -> None:
        self.overall_state = state
        self.overall_state_changed.emit(state)

    def _record_log(self, key: str, text: str) -> None:
        if not text:
            return
        if key == "system":
            log_store().write("navigation", "stack", text)
        if key == "nav2" and self.navigation_log is not None:
            self.navigation_log(text)
        self.log_events.append((key, text))
        if len(self.log_events) > 10000:
            del self.log_events[:2000]
        self.log_received.emit(key, text)

    def record_external_log(self, key: str, text: str) -> None:
        """Add a related feature log to this page's persistent log stream."""
        self._record_log(key, text)

    def _record_sensor_log(self, key: str, text: str) -> None:
        """Mirror diagnostics into the shared sensor page when supported."""
        append_log = getattr(self.sensor_controller, "append_log", None)
        if callable(append_log):
            append_log(key, text)
        else:
            self._record_log(key, text)

    def shutdown(self) -> None:
        self._stopping = False
        if self.configure_navigation is not None:self.configure_navigation(None)
        self._generation += 1
        if self._lidar_service_start is not None:
            self._lidar_service_start.kill()
            self._lidar_service_start = None
        self._queue.clear()
        self._waiting_key = ""
        for process in self.processes.values():
            process.shutdown()
        if self._owns_native5_service:
            if self._lidar_service_stop.state() == QProcess.NotRunning:
                self._lidar_service_stop.start("systemctl", ["--user", "stop", "nav-livox5.service"])
            if not self._lidar_service_stop.waitForFinished(15000) and self._owns_native5_service:
                self._record_log("system", ".5 雷达驱动未确认停止，请检查 nav-livox5.service。\n")

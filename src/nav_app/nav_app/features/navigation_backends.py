"""Pluggable localization and navigation launch backends."""

from __future__ import annotations

import math
from pathlib import Path
import shlex
from typing import Dict, List, Tuple

import yaml


def launch_bool(value: bool) -> str:
    return "true" if value else "false"


def existing_optional_file(path: str, label: str) -> Tuple[bool, str]:
    if path.strip() and not Path(path).expanduser().is_file():
        return False, f"{label}不存在：{path}"
    return True, ""


def expanded_or_empty(path: str) -> str:
    return str(Path(path).expanduser()) if path.strip() else ""


class LaunchBackend:
    """Interface implemented by a selectable ROS 2 launch backend."""

    key = ""
    label = ""
    description = ""

    def validate(self, _config) -> Tuple[bool, str]:
        return True, ""

    def launch_arguments(self, _config) -> List[str]:
        raise NotImplementedError


class PointCloudLocalizerBackend(LaunchBackend):
    key = "pointcloud_localizer"
    label = "点云 Localizer"
    description = "使用 PCD 地图、机身点云和里程计进行地图定位。"

    @staticmethod
    def _scan_context_parameters(config_path: str) -> Tuple[float, float]:
        # The packaged localizer.yaml is used when the UI field is empty.
        if not config_path.strip():
            return 80.0, 2.0
        path = Path(config_path.strip()).expanduser()
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise ValueError(f"Localizer 配置无法读取：{exc}") from exc
        if not isinstance(document, dict):
            raise ValueError("Localizer 配置顶层必须是键值表")
        try:
            radius = float(document.get("sc_max_radius", 40.0))
            grid = float(document.get("sc_grid_resolution", 2.0))
        except (TypeError, ValueError) as exc:
            raise ValueError("Localizer 的 SC 生成参数不是有效数字") from exc
        if not math.isfinite(radius) or not math.isfinite(grid) or radius <= 0 or grid <= 0:
            raise ValueError("Localizer 的 SC 生成参数必须是正数")
        return radius, grid

    def validate(self, config, *, cancelled=None, progress=None) -> Tuple[bool, str]:
        map_path = Path(config.localization_map).expanduser()
        if not map_path.is_file():
            return False, f"定位点云地图不存在：{map_path}"
        if map_path.suffix.lower() != ".pcd":
            return False, "点云 Localizer 的定位地图必须是 PCD 文件"
        if config.scan_context_map.strip():
            from .localization_tools import validate_scan_context_cache
            try:
                radius, grid = self._scan_context_parameters(
                    config.localizer_config)
            except ValueError as exc:
                return False, str(exc)
            valid, message = validate_scan_context_cache(
                str(map_path), config.scan_context_map.strip(),
                expected_radius=radius, expected_grid=grid,
                cancelled=cancelled, progress=progress)
            if not valid:
                return False, message
        return existing_optional_file(
            config.localizer_config, "Localizer 配置文件")

    def launch_arguments(self, config) -> List[str]:
        arguments = [
            "launch", "localizer", "localizer_launch.py",
            f"map:={Path(config.localization_map).expanduser().resolve()}",
            f"cloud_topic:={config.localizer_cloud_topic.strip()}",
            f"odom_topic:={config.localizer_odom_topic.strip()}",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
        ]
        if config.localizer_config.strip():
            arguments.append(
                "config_path:="
                f"{Path(config.localizer_config).expanduser().resolve()}")
        if config.localizer_tuning_enabled:
            for name in (
                "rough_score_thresh", "refine_score_thresh",
                "rough_scan_resolution", "refine_scan_resolution",
                "rough_map_resolution", "refine_map_resolution",
                "rough_max_iteration", "refine_max_iteration",
            ):
                arguments.append(f"{name}_override:={getattr(config, name)}")
        if config.odometry_backend == 'fast_livo' and config.map_gravity_aligned:
            arguments.append('sc_use_odom_gravity:=true')
        if config.scan_context_map.strip():
            scan_context = Path(
                config.scan_context_map.strip()).expanduser().resolve()
            arguments.append(
                "scan_context_path:="
                f"{scan_context}")
        return arguments


class ExternalLocalizationBackend(LaunchBackend):
    key = "external"
    label = "使用已有定位 TF"
    description = "接收与当前地图一致的 map→odom，不启动 NAV 定位算法。"

    def launch_arguments(self, config):
        return []


class AmclLocalizationBackend(LaunchBackend):
    key = "amcl"
    label = "AMCL · LaserScan 定位"
    description = "由 Nav2 启动 AMCL，使用所选栅格和 LaserScan 定位。"

    def launch_arguments(self, config):
        return []


class DisabledLocalizationBackend(LaunchBackend):
    key = "disabled"
    label = "不启动定位"
    description = "适用于导航后端直接使用 LIO 里程计的场景。"

    def launch_arguments(self, _config) -> List[str]:
        return []


class CustomLocalizationBackend(LaunchBackend):
    key = "custom"
    label = "自定义定位 Launch"
    description = "启动任意定位包；附加参数支持地图和配置文件占位符。"

    def validate(self, config) -> Tuple[bool, str]:
        if not config.custom_localization_package.strip():
            return False, "请填写自定义定位包名"
        if not config.custom_localization_launch.strip():
            return False, "请填写自定义定位 Launch 文件"
        try:
            self._extra_arguments(config)
        except (KeyError, ValueError) as exc:
            return False, f"自定义定位参数格式错误：{exc}"
        return True, ""

    @staticmethod
    def _extra_arguments(config) -> List[str]:
        values = {
            "map": expanded_or_empty(config.localization_map),
            "config": expanded_or_empty(config.localizer_config),
            "use_sim_time": launch_bool(config.use_sim_time),
        }
        return [token.format(**values) for token in
                shlex.split(config.custom_localization_arguments)]

    def launch_arguments(self, config) -> List[str]:
        return [
            "launch",
            config.custom_localization_package.strip(),
            config.custom_localization_launch.strip(),
            *self._extra_arguments(config),
        ]


class Nav2Backend(LaunchBackend):
    key = "nav2"
    label = "Nav2"
    description = "使用栅格地图和 Nav2 Bringup 进行全局导航。"

    def validate(self, config) -> Tuple[bool, str]:
        map_path = Path(config.navigation_map).expanduser()
        if not map_path.is_file():
            return False, f"Nav2 导航地图不存在：{map_path}"
        if map_path.suffix.lower() not in (".yaml", ".yml"):
            return False, "Nav2 导航地图必须是 YAML 文件"
        return existing_optional_file(config.nav2_params, "Nav2 参数文件")

    def launch_arguments(self, config) -> List[str]:
        arguments = [
            "launch", "nav_app",
            "woosh_nav2.launch.py" if config.robot_profile == "woosh" else "navigation_external.launch.py",
            f"external_localization:={launch_bool(config.localization_backend not in ('disabled', 'amcl'))}",
            f"odom_topic:={config.external_odom_topic if config.odometry_backend == 'external' else (('/nav/woosh/odom' if config.robot_profile == 'woosh' else '/aft_mapped_to_init') if config.odometry_backend == 'fast_livo' else config.localizer_odom_topic)}",
            f"scan_topic:={config.scan_topic.strip()}",
            f"map:={Path(config.navigation_map).expanduser().resolve()}",
            f"use_sim_time:={launch_bool(config.use_sim_time)}",
            f"autostart:={launch_bool(config.autostart)}",
            f"rviz:={launch_bool(config.nav2_rviz)}",
        ]
        if config.nav2_params.strip():
            argument = "nav2_params_file" if config.robot_profile == "woosh" else "params_file"
            arguments.append(
                f"{argument}:={Path(config.nav2_params).expanduser().resolve()}")
        return arguments




class ScanPlannerBackend(LaunchBackend):
    key = "scan_planner"
    label = "SCAN-Planner"
    description = "直接消费里程计和点云进行局部空间规划，不需要栅格地图。"

    def validate(self, config) -> Tuple[bool, str]:
        if config.scan_navi_mode not in (1, 2, 3):
            return False, "SCAN-Planner 导航模式必须是 1、2 或 3"
        if config.scan_sensor_type not in ("lidar", "depth"):
            return False, "SCAN-Planner 传感器类型必须是 lidar 或 depth"
        for path, label in (
            (config.scan_planner_params, "SCAN-Planner 参数文件"),
            (config.scan_controller_params, "SCAN 控制器参数文件"),
            (config.scan_keypoints_file, "SCAN 途径点文件"),
            (config.scan_reference_path_file, "SCAN 参考路径文件"),
        ):
            valid, message = existing_optional_file(path, label)
            if not valid:
                return valid, message
        if config.scan_navi_mode == 2 and not config.scan_keypoints_file.strip():
            return False, "SCAN-Planner 模式 2 必须选择途径点 YAML"
        if config.scan_reference_path_file.strip() and config.scan_navi_mode != 3:
            return False, "SCAN 参考路径文件只能用于导航模式 3"
        return True, ""

    def launch_arguments(self, config) -> List[str]:
        arguments = [
            "launch", "scan_planner", "run.launch.py",
            f"navi_mode:={config.scan_navi_mode}",
            f"sensor_type:={config.scan_sensor_type}",
            f"body_pose_topic:={config.scan_body_pose_topic.strip()}",
            f"sensor_pose_topic:={config.scan_sensor_pose_topic.strip()}",
            f"cloud_topic:={config.scan_cloud_topic.strip()}",
            f"depth_topic:={config.scan_depth_topic.strip()}",
            f"goal_topic:={config.scan_goal_topic.strip()}",
            f"initial_path_topic:={config.scan_initial_path_topic.strip()}",
            f"cmd_vel_topic:={config.scan_cmd_vel_topic.strip()}",
            f"start_controller:={launch_bool(config.scan_start_controller)}",
            f"cloud_is_world:={launch_bool(config.scan_cloud_is_world)}",
            f"need_extrinsic:={launch_bool(config.scan_need_extrinsic)}",
        ]
        for name, path in (
            ("planner_params_file", config.scan_planner_params),
            ("controller_params_file", config.scan_controller_params),
            ("keypoints_file", config.scan_keypoints_file),
            ("reference_path_file", config.scan_reference_path_file),
        ):
            if path.strip():
                arguments.append(
                    f"{name}:={Path(path).expanduser().resolve()}")
        return arguments


class CustomNavigationBackend(LaunchBackend):
    key = "custom"
    label = "自定义导航 Launch"
    description = "启动任意导航包；附加参数支持地图和参数文件占位符。"

    def validate(self, config) -> Tuple[bool, str]:
        if not config.custom_navigation_package.strip():
            return False, "请填写自定义导航包名"
        if not config.custom_navigation_launch.strip():
            return False, "请填写自定义导航 Launch 文件"
        try:
            self._extra_arguments(config)
        except (KeyError, ValueError) as exc:
            return False, f"自定义导航参数格式错误：{exc}"
        return True, ""

    @staticmethod
    def _extra_arguments(config) -> List[str]:
        values = {
            "map": expanded_or_empty(config.navigation_map),
            "params": expanded_or_empty(config.nav2_params),
            "use_sim_time": launch_bool(config.use_sim_time),
        }
        return [token.format(**values) for token in
                shlex.split(config.custom_navigation_arguments)]

    def launch_arguments(self, config) -> List[str]:
        return [
            "launch",
            config.custom_navigation_package.strip(),
            config.custom_navigation_launch.strip(),
            *self._extra_arguments(config),
        ]




LOCALIZATION_BACKENDS: Dict[str, LaunchBackend] = {
    backend.key: backend for backend in (
        PointCloudLocalizerBackend(),
        DisabledLocalizationBackend(),
        CustomLocalizationBackend(),
        ExternalLocalizationBackend(),
        AmclLocalizationBackend(),
    )
}

NAVIGATION_BACKENDS: Dict[str, LaunchBackend] = {
    backend.key: backend for backend in (
        Nav2Backend(),
        ScanPlannerBackend(),
        CustomNavigationBackend(),
    )
}

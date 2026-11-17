"""Estimator only: sensors are owned by the Qt controller or a bag player."""
import os
import math
import yaml
import json
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _estimator_exited(event, context):
    if context.is_shutdown:
        return []
    detail = ("stopped before launch shutdown (exit code 0); inspect [NAV_STOP] for stop source"
              if event.returncode == 0 else f"exited with code {event.returncode}")
    return [
        LogInfo(msg=f"[NAV_FAILURE] FAST-LIVO2 {detail}; automatic restart disabled"),
        EmitEvent(event=Shutdown(reason="FAST-LIVO2 stopped")),
    ]


def camera_parameters(path):
    """Read the same camera calibration locally for both estimator and snapshot node."""
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("Camera calibration must be a ROS parameter mapping")
    values = {}
    for name in ("/**", "parameter_blackboard", "/parameter_blackboard", "laserMapping", "/laserMapping"):
        section = document.get(name, {})
        if not isinstance(section, dict) or not isinstance(section.get("ros__parameters", {}), dict):
            raise ValueError(f"Invalid camera parameter section: {name}")
        values.update(section.get("ros__parameters", {}))
    # This navigation profile uses rectilinear RGB. Reject an incomplete model before
    # starting any process, rather than silently constructing a zero-sized camera.
    if values.get("cam_model") != "Pinhole":
        raise ValueError("Navigation requires a calibrated Pinhole camera model")
    for key in ("cam_width", "cam_height"):
        if type(values.get(key)) is not int or values[key] <= 0:
            raise ValueError(f"Camera calibration requires positive integer {key}")
    for key in ("cam_fx", "cam_fy", "cam_cx", "cam_cy", "scale"):
        value = values.get(key, 1.0 if key == "scale" else None)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"Camera calibration requires finite {key}")
        if key in ("cam_fx", "cam_fy", "scale") and value <= 0:
            raise ValueError(f"Camera calibration requires positive {key}")
        values[key] = float(value)
    for key in ("cam_d0", "cam_d1", "cam_d2", "cam_d3"):
        value = values.get(key, 0.0)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"Camera calibration requires finite {key}")
        values[key] = float(value)
    return {key: value for key, value in values.items() if key.startswith("cam_") or key == "scale"}


def validate_device_profile(params_path, simulated, driver_path=""):
    """A device-specific calibration is valid only for its recorded driver frame.

    Bag parameters are reconstructed from that bag's snapshot by the APP. A
    deliberate offline calibration experiment may use this profile with sim time.
    """
    sidecar = Path(params_path).with_suffix(".calibration.json")
    if simulated or not sidecar.is_file():
        return
    contract = json.loads(sidecar.read_text(encoding="utf-8"))
    driver = json.loads(Path(driver_path or contract["driver_config"]).expanduser().read_text(encoding="utf-8"))
    configurations = driver.get("lidar_configs", [])
    if len(configurations) != 1:
        raise ValueError("设备标定要求单雷达配置，请重新核对坐标系")
    actual = configurations[0].get("extrinsic_parameter", {})
    for key, expected in contract["driver_extrinsic_parameter"].items():
        if key not in actual or not math.isclose(float(actual[key]), expected, abs_tol=1e-6):
            raise ValueError(f"雷达驱动坐标与相机标定不匹配：{key}；请恢复或重新标定")


def camera_info_parameters(info):
    """Use this camera's raw-image model, not another device's YAML intrinsics."""
    if (info.distortion_model not in ('plumb_bob', 'rational_polynomial') or
            info.width <= 0 or info.height <= 0 or len(info.k) != 9 or
            len(info.d) not in (0, 4, 5, 8) or
            not all(math.isfinite(float(v)) for v in [*info.k, *info.d]) or
            info.k[0] <= 0 or info.k[4] <= 0):
        raise ValueError('当前相机未提供有效的内参，请检查相机数据')
    # The estimator's pinhole camera supports k1,k2,p1,p2 only.
    if any(abs(float(v)) > 1e-10 for v in info.d[4:]):
        raise ValueError('当前相机畸变模型需要校正适配，不能截断畸变参数')
    distortion = list(info.d[:4]) if info.d else [0., 0., 0., 0.]
    return dict(cam_model='Pinhole', cam_width=int(info.width), cam_height=int(info.height),
                cam_fx=float(info.k[0]), cam_fy=float(info.k[4]),
                cam_cx=float(info.k[2]), cam_cy=float(info.k[5]), scale=1.,
                **{f'cam_d{i}': float(value) for i, value in enumerate(distortion)})


def live_camera_parameters(topic, timeout=12.):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    ctx = Context()
    rclpy.init(args=[], context=ctx)
    node = rclpy.create_node('nav_camera_parameters_probe', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    result, errors = [], []
    def received(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        if not -.1 <= age <= 2.:
            return
        try:
            result.append(camera_info_parameters(message))
        except ValueError as exc:
            errors.append(str(exc))
    node.create_subscription(CameraInfo, topic, received, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout
        while not result and not errors and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        if not result:
            raise ValueError(errors[-1] if errors else f'未收到当前相机参数：{topic}，请检查相机是否启动')
        return result[-1]
    finally:
        executor.shutdown()
        node.destroy_node()
        ctx.shutdown()


def _launch_nodes(context):
    simulated = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    validate_device_profile(LaunchConfiguration("params_file").perform(context),
                            simulated,
                            LaunchConfiguration("lidar_config_file").perform(context))
    if not simulated and int(LaunchConfiguration("img_en").perform(context)):
        topic = LaunchConfiguration("camera_info_topic").perform(context).strip()
        if not topic:
            topic = LaunchConfiguration("image_topic").perform(context).rsplit('/', 1)[0] + '/camera_info'
        camera = live_camera_parameters(topic)
    else:
        camera = camera_parameters(LaunchConfiguration("camera_params_file").perform(context))
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    realtime = {}
    if not simulated:
        requested = LaunchConfiguration("live_surface_resolution").perform(context).strip()
        # Keep the device's configured estimator density. The former implicit
        # 0.5 m override made live mapping differ from bag replay and could
        # deprive plane voxels of the multiple observations needed at startup.
        realtime = {"publish.dense_map_en": True}
        if requested:
            surface = float(requested)
            if not math.isfinite(surface) or surface <= 0:
                raise ValueError("实时匹配点云分辨率必须大于0")
            realtime["preprocess.filter_size_surf"] = surface
    return [
        Node(package="demo_nodes_cpp", executable="parameter_blackboard",
             name="parameter_blackboard", output="screen",
             parameters=[camera, {"use_sim_time": sim}]),
        Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
             respawn=False, on_exit=_estimator_exited,
             # Qt captures console output into its bounded asynchronous log
             # writer. ROS's duplicate spdlog file sink writes on the estimator
             # callback thread and can stall it behind recording IO.
             ros_arguments=["--disable-external-lib-logs"],
             output="screen", parameters=[LaunchConfiguration("params_file"), camera, realtime, {
                 "use_sim_time": sim,
                 "common.lid_topic": LaunchConfiguration("pointcloud_topic"),
                 "common.imu_topic": LaunchConfiguration("imu_topic"),
                 "common.img_topic": LaunchConfiguration("image_topic"),
                 "common.img_en": ParameterValue(LaunchConfiguration("img_en"), value_type=int),
                 # Real-time handheld work must not run one ICP per camera frame.
                 # Offline preparation preserves the original image cadence.
                 "navigation.lio_min_interval_s": .005 if simulated else .095,
                 "navigation.vio_min_interval_s": 0. if simulated else .2,
                 "navigation.enabled": True, "navigation.world_frame": "odom",
                 "navigation.body_frame": LaunchConfiguration("body_frame"), "uav.gravity_align_en": True,
                 "navigation.propagated_tf": ParameterValue(LaunchConfiguration("imu_rate_odom"), value_type=bool),
                 "uav.imu_rate_odom": ParameterValue(LaunchConfiguration("imu_rate_odom"), value_type=bool),
                 "pcd_save.pcd_save_en": False, "pcd_save.colmap_output_en": False,
                 "image_save.img_save_en": False,
             }]),
        Node(package="rviz2", executable="rviz2", name="fast_livo_rviz",
             condition=IfCondition(LaunchConfiguration("rviz")),
             parameters=[{"use_sim_time": sim}]),
    ]


def generate_launch_description():
    share = get_package_share_directory("fast_livo")
    defaults = {
        "params_file": os.path.join(share, "config", "mid360_live.yaml"),
        "camera_params_file": os.path.join(share, "config", "camera_pinhole_d435.yaml"),
        "pointcloud_topic": "/livox/lidar", "imu_topic": "/livox/imu",
        "image_topic": "/camera/camera/color/image_raw", "img_en": "1",
        "camera_info_topic": "", "body_frame": "base_link",
        "use_sim_time": "false", "rviz": "false",
        "lidar_config_file": "",
        "live_surface_resolution": "", "imu_rate_odom": "false",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        OpaqueFunction(function=_launch_nodes),
    ])

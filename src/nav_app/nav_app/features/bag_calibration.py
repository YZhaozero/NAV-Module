"""Restore recorded sensor calibration without inheriting today's device profile."""
from pathlib import Path
import hashlib
import json
import math
import yaml


def recorded_parameters(snapshot):
    if not isinstance(snapshot, dict):
        raise ValueError("录包参数快照格式无效")
    parameters = snapshot.get("parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("录包参数快照格式无效")
    fields = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
              6: "bool_array_value", 7: "integer_array_value", 8: "double_array_value", 9: "string_array_value"}
    restored = {}
    for key, value in parameters.items():
        if not isinstance(value, dict):
            raise ValueError(f"录包参数快照格式无效：{key}")
        field = fields.get(value.get("type"))
        if field is None:
            continue
        if field not in value:
            raise ValueError(f"录包参数快照缺少参数值：{key}")
        restored[key] = value[field]
    return restored


def _algorithm_parameters(base):
    """Return the FAST-LIVO2 ROS parameter map from supported YAML layouts."""
    if not isinstance(base, dict):
        raise ValueError("FAST-LIVO2 基础配置格式无效")
    # Deployed FAST-LIVO2 profiles use the wildcard selector.  Older project
    # profiles used the node name, with or without the leading slash.
    for selector in ("laserMapping", "/laserMapping", "/**"):
        node = base.get(selector)
        if not isinstance(node, dict):
            continue
        parameters = node.get("ros__parameters")
        if isinstance(parameters, dict):
            return parameters
    raise ValueError(
        "FAST-LIVO2 基础配置格式无效：未找到 "
        "laserMapping、/laserMapping 或 /** 的 ros__parameters")


def restore_bag_calibration(bag, base_params, output_root, *, require_camera=True):
    """Return derived algorithm/camera YAML paths; never change a source bag.

    Restore calibration only. Current runtime limits and fixes remain in the base
    profile. Correcting a known bad historical calibration is a separate reviewed
    offline experiment, not an implicit modification during APP replay.
    """
    manifest = Path(bag).expanduser() / "nav_manifest.json"
    if not manifest.is_file():
        raise ValueError("此录包缺少标定快照，不能自动套用当前设备参数；请先完成离线标定核对")
    raw = manifest.read_bytes()
    document = json.loads(raw)
    snapshots = document.get("parameters_start", {})
    algorithm = recorded_parameters(snapshots.get("/laserMapping", {}))
    camera = recorded_parameters(snapshots.get("/parameter_blackboard", {}))
    camera.update({k: v for k, v in algorithm.items() if k.startswith("cam_") or k == "scale"})
    camera = {k: v for k, v in camera.items() if k.startswith("cam_") or k == "scale"}
    required = {"extrin_calib.Rcl": 9, "extrin_calib.Pcl": 3,
                "extrin_calib.extrinsic_R": 9, "extrin_calib.extrinsic_T": 3}
    for key, size in required.items():
        values = algorithm.get(key)
        if not isinstance(values, list) or len(values) != size or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
            raise ValueError(f"录包标定快照缺失或无效：{key}")
    for key in ("time_offset.img_time_offset", "time_offset.imu_time_offset", "time_offset.exposure_time_init"):
        if key not in algorithm or not isinstance(algorithm[key], (float, int)) or not math.isfinite(algorithm[key]):
            raise ValueError(f"录包缺少有效时间参数：{key}")
    if require_camera and (camera.get("cam_model") != "Pinhole" or any(not camera.get(key, 0) > 0 for key in ("cam_fx", "cam_fy", "cam_width", "cam_height"))):
        raise ValueError("录包缺少有效的相机参数快照")
    base = yaml.safe_load(Path(base_params).read_text(encoding="utf-8"))
    parameters = _algorithm_parameters(base)
    # ROS permits both nested and dotted parameters. Avoid duplicated names when
    # layering a flat ROS service snapshot over a nested configuration file.
    for family in ("extrin_calib", "time_offset"):
        parameters.pop(family, None)
        for key in list(parameters):
            if key.startswith(family + "."):
                parameters.pop(key)
        parameters.update({k: v for k, v in algorithm.items() if k.startswith(family + ".")})
    payload = yaml.safe_dump(base, sort_keys=True)
    camera_payload = yaml.safe_dump({"parameter_blackboard": {"ros__parameters": camera}}, sort_keys=True)
    # Include source identity as well as generated contents.  Otherwise two
    # bags with identical calibration share a cache directory and overwrite
    # source.json, making the provenance record point at whichever replay ran
    # last.
    provenance = {
        "bag": str(Path(bag).resolve()),
        "manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "base_params": str(Path(base_params).resolve()),
        "visual_inputs_required": require_camera,
    }
    identity = hashlib.sha256(
        (payload + camera_payload + json.dumps(
            provenance, sort_keys=True)).encode()).hexdigest()[:20]
    root = Path(output_root) / identity
    root.mkdir(parents=True, exist_ok=True)
    for name, content in (("algorithm.yaml", payload), ("camera.yaml", camera_payload)):
        target = root / name
        if target.exists() and target.read_text(encoding="utf-8") != content:
            raise ValueError("回放配置缓存内容不一致，请重新核对")
        target.write_text(content, encoding="utf-8")
    (root / "source.json").write_text(json.dumps({
        **provenance,
        "policy": "Recorded calibration with current runtime controls",
    }, indent=2), encoding="utf-8")
    return str(root / "algorithm.yaml"), str(root / "camera.yaml") if require_camera else ""

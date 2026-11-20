"""Explicit map bundles. Filenames alone never certify coordinate alignment."""
from dataclasses import dataclass, field
from datetime import datetime
import json
import os
from pathlib import Path
import uuid


@dataclass
class Scene:
    title: str
    manifest: Path | None = None
    grid: str = ""
    cloud: str = ""
    scan_context: str = ""
    frame: str = "map"
    verified: bool = False
    scan_context_verified: bool = False
    destinations: list = field(default_factory=list)
    viewer_url: str = ""
    error: str = ""
    gravity_aligned: bool = False
    meshnav: str = ""
    meshnav_error: str = ""
    previous_meshnav: str = ""

    @property
    def file_time(self):
        try:
            return Path(self.cloud).stat().st_mtime if self.cloud else 0.
        except OSError:
            return 0.

    @property
    def time_label(self):
        value=self.file_time
        return "点云文件时间 · " + datetime.fromtimestamp(value).strftime("%Y-%m-%d %H:%M") if value else "点云文件时间未知"

    @property
    def ready(self):
        return (not self.error and self.verified and bool(self.frame)
                and Path(self.grid).is_file() and Path(self.cloud).is_file())

    @property
    def global_ready(self):
        return (self.ready and self.scan_context_verified
                and bool(self.scan_context)
                and Path(self.scan_context).is_file())

    @property
    def summary(self):
        if self.error:
            return self.error
        if not self.cloud or not Path(self.cloud).is_file():
            return "缺少定位地图"
        if not self.grid or not Path(self.grid).is_file():
            return "待登记 · 通行栅格未绑定"
        if not self.manifest:
            return "待登记 · 请核对地图坐标"
        if not self.verified:
            return "待核对地图坐标"
        return (
            "地图包完整 · 全局重定位就绪"
            if self.global_ready else
            "可导航 · SC 待生成或核对")

    @property
    def resource_summary(self):
        cloud = "PCD ✓" if self.cloud and Path(self.cloud).is_file() else "PCD 缺失"
        grid = (
            "栅格 ✓" if self.manifest and self.grid and Path(self.grid).is_file()
            else "栅格待登记" if self.grid and Path(self.grid).is_file()
            else "栅格未绑定")
        scan_context = (
            "SC ✓" if self.global_ready else
            "SC 待核对" if self.scan_context and Path(self.scan_context).is_file()
            else "SC 未生成")
        mesh = "MeshNav ✓" if self.meshnav and not self.meshnav_error else "MeshNav 待生成"
        return f"{cloud} · {grid} · {scan_context} · {mesh}"


def normalize_scene_title(value):
    """Return a user-facing bundle title or reject ambiguous input."""
    if not isinstance(value, str):
        raise ValueError("地图包名称无效")
    title = value.strip()
    if not title:
        raise ValueError("地图包名称不能为空")
    if len(title) > 64:
        raise ValueError("地图包名称不能超过 64 个字符")
    if any(ord(character) < 32 for character in title):
        raise ValueError("地图包名称不能包含换行或控制字符")
    return title


def rename_scene(path, title):
    """Atomically update only the display title of a registered map bundle."""
    path = Path(path).expanduser().resolve(strict=True)
    title = normalize_scene_title(title)
    current = read_scene(path)
    if current.title == title:
        return current
    data = json.loads(path.read_text(encoding="utf-8"))
    data["title"] = title
    temporary = path.with_name(
        f".{path.name}.{uuid.uuid4().hex}.rename.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_scene(path)


def grid_candidates_for_cloud(cloud_path):
    """Return same-name grid candidates in deterministic preference order."""
    cloud = Path(cloud_path)
    fixed = (
        cloud.with_suffix(".yaml"),
        cloud.with_suffix(".yml"),
        cloud.with_name(cloud.stem + "-grid.yaml"),
        cloud.with_name(cloud.stem + "-grid.yml"),
    )
    numbered = sorted({
        *cloud.parent.glob(cloud.stem + "-grid-*.yaml"),
        *cloud.parent.glob(cloud.stem + "-grid-*.yml"),
    })
    return (*fixed, *numbered)


def read_scene(path):
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    version = data.get("version")
    if version not in (1, 2):
        raise ValueError("不支持的地图包版本")
    def asset(key):
        value = data.get(key, "")
        return str((path.parent / value).resolve()) if value else ""
    points = []
    for point in data.get("destinations", []):
        import math
        xy = [float(v) for v in point["xy"]]
        if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
            raise ValueError("地点坐标无效")
        points.append({"name": str(point["name"]), "xy": xy})
    cloud = asset("cloud")
    scan_context = asset("scan_context")
    if not scan_context and cloud:
        candidate = Path(cloud + ".sc.v2")
        scan_context = str(candidate) if candidate.is_file() else ""
    scene = Scene(
        title=str(data.get("title", path.stem)),
        manifest=path,
        grid=asset("grid"),
        cloud=cloud,
        scan_context=scan_context,
        frame=str(data.get("frame", "map")),
        verified=data.get("coordinates_verified") is True,
        scan_context_verified=(
            version == 2
            and data.get("scan_context_verified") is True),
        destinations=points,
        viewer_url=str(data.get("viewer_url", "")),
        gravity_aligned=data.get('generated', {}).get('gravity_aligned') is True,
    )
    mesh = data.get("meshnav")
    if mesh is not None:
        try:
            scene.meshnav = str((path.parent / (mesh.get("physical_v2") or mesh["file"])).resolve())
            if mesh.get("physical_v2"):
                scene.previous_meshnav = str((path.parent / mesh["file"]).resolve())
            source = Path(cloud).stat()
            if (Path(scene.meshnav).suffix.lower() != ".ply" or not Path(scene.meshnav).is_file()
                    or source.st_size != mesh["pcd_size"] or source.st_mtime_ns != mesh["pcd_mtime_ns"]):
                scene.meshnav_error = "MeshNav 网格已失效，请在地图管理重新生成"
        except (OSError, KeyError, TypeError):
            scene.meshnav_error = "MeshNav 网格关联无效，请在地图管理重新生成"
    if scene.grid and Path(scene.grid).suffix.lower() not in (".yaml", ".yml"):
        scene.error = "通行地图格式不支持"
    if scene.cloud and Path(scene.cloud).suffix.lower() != ".pcd":
        scene.error = "定位地图需要 PCD 格式"
    if scene.scan_context and not scene.scan_context.endswith(".sc.v2"):
        scene.error = "SC 地图格式不支持"
    if scene.frame != "map":
        scene.error = "当前导航配置需要 map 坐标系"
    return scene


def discover_scenes(directory):
    directory = Path(directory)
    scenes, claimed = [], set()
    for path in sorted(directory.glob("*.scene.json")):
        try:
            scene = read_scene(path)
            claimed.update((scene.grid, scene.cloud, scene.scan_context))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            scene = Scene(path.stem, path, error=f"地图包无法读取：{exc}")
        scenes.append(scene)
    # The primary list is PCD-centric.  Loose YAML/PGM assets remain available
    # in the registration page, but no longer appear as misleading map rows.
    for path in sorted(directory.glob("*.pcd")):
        if str(path.resolve()) not in claimed:
            sc_path = Path(str(path.resolve()) + ".sc.v2")
            grid = next(
                (candidate for candidate in grid_candidates_for_cloud(path)
                 if candidate.is_file()),
                None)
            scenes.append(Scene(
                path.stem,
                grid=str(grid.resolve()) if grid is not None else "",
                cloud=str(path.resolve()),
                scan_context=str(sc_path) if sc_path.is_file() else "",
            ))
    # Stable ordering preserves the existing path order when timestamps tie.
    return sorted(scenes, key=lambda scene: -scene.file_time)

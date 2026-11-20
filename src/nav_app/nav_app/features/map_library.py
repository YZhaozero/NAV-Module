"""Safe deletion plans for local map bundles and individual map assets."""

from dataclasses import dataclass
from pathlib import Path
import stat

import yaml


@dataclass(frozen=True)
class MapDeletion:
    root: Path
    kind: str
    targets: tuple[Path, ...]
    fingerprint: tuple[tuple, ...]
    preserved: tuple[Path, ...] = ()


def _inside(root: Path, path: Path) -> bool:
    return path != root and root in path.parents


def _grid_assets(path: str) -> tuple[Path, ...]:
    if not path:
        return ()
    grid = Path(path).expanduser().resolve()
    assets = [grid]
    if grid.suffix.lower() in (".yaml", ".yml") and grid.is_file():
        try:
            document = yaml.safe_load(grid.read_text(encoding="utf-8")) or {}
            image = str(document.get("image", "")).strip()
            if image:
                image_path = Path(image).expanduser()
                if not image_path.is_absolute():
                    image_path = grid.parent / image_path
                assets.append(image_path.resolve())
        except (OSError, ValueError, TypeError, yaml.YAMLError):
            pass
    return tuple(dict.fromkeys(assets))


def scene_assets(scene, kind: str = "package") -> tuple[Path, ...]:
    if kind not in ("package", "grid", "cloud", "scan_context"):
        raise ValueError("未知的地图删除类型")
    paths = []
    if kind == "package" and getattr(scene, "manifest", None):
        paths.append(Path(scene.manifest).expanduser().resolve())
    if kind in ("package", "grid"):
        paths.extend(_grid_assets(str(getattr(scene, "grid", ""))))
    if kind in ("package", "cloud") and getattr(scene, "cloud", ""):
        paths.append(Path(scene.cloud).expanduser().resolve())
    if kind in ("package", "cloud", "scan_context") and getattr(
        scene, "scan_context", ""
    ):
        paths.append(Path(scene.scan_context).expanduser().resolve())
    if kind == "package" and getattr(scene, "meshnav", ""):
        paths.append(Path(scene.meshnav).expanduser().resolve())
        if getattr(scene, "previous_meshnav", ""):
            paths.append(Path(scene.previous_meshnav))
    return tuple(dict.fromkeys(paths))


def _fingerprint(path: Path) -> tuple:
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise ValueError(f"地图资产不是普通文件，不能删除：{path.name}")
    return (
        str(path), info.st_dev, info.st_ino, info.st_mode,
        info.st_size, info.st_mtime_ns,
    )


def prepare_map_deletion(scene, root, scenes=(), kind: str = "package") -> MapDeletion:
    """Snapshot local files while preserving external and shared assets."""
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("地图保存目录不存在")
    requested = scene_assets(scene, kind)
    if not requested:
        raise ValueError("当前条目没有可删除的地图文件")

    other_references = set()
    for candidate in scenes:
        if candidate is scene:
            continue
        other_references.update(scene_assets(candidate, "package"))

    targets, preserved = [], []
    for path in requested:
        if not _inside(root, path) or path in other_references:
            preserved.append(path)
            continue
        if path.exists():
            _fingerprint(path)
            targets.append(path)

    if kind != "package" and preserved:
        reason = "被其他地图包共用" if any(
            path in other_references for path in preserved) else "不在本地地图目录"
        raise ValueError(f"所选地图{reason}，不能在这里删除")
    if not targets and kind != "package":
        raise ValueError("所选地图文件不存在或不可删除")
    fingerprint = tuple(_fingerprint(path) for path in targets)
    return MapDeletion(
        root, kind, tuple(targets), fingerprint, tuple(preserved))


def delete_map(plan: MapDeletion) -> int:
    """Revalidate the confirmed files, then delete only those exact files."""
    current = tuple(_fingerprint(path) for path in plan.targets)
    if current != plan.fingerprint:
        raise ValueError("地图文件已变化，请刷新后重新确认删除")
    deleted = 0
    for path in plan.targets:
        try:
            deleted += path.stat().st_size
            path.unlink()
        except OSError as exc:
            raise OSError(f"删除未完成，请刷新检查 {path.name}：{exc}") from exc
    return deleted

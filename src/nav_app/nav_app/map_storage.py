"""Shared paths and collision-free names for map files."""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Optional


def default_map_directory(configured: str = "") -> Path:
    """Resolve the common map directory used by every map-saving workflow."""
    requested = str(configured).strip() or os.environ.get(
        "NAV_MAP_DIR", "").strip()
    if requested:
        return Path(requested).expanduser().resolve()

    return (Path.home() / "nav_maps").resolve()


def unique_map_file(
    directory: Path,
    map_name: str,
    suffix: str,
    now: Optional[datetime] = None,
) -> Path:
    """Return a timestamped map path without replacing an existing file."""
    extension = suffix if suffix.startswith(".") else f".{suffix}"
    raw_name = Path(str(map_name).strip()).stem
    safe_name = "".join(
        character if character.isalnum() or character in ("-", "_") else "_"
        for character in raw_name
    ).strip("_-") or "nav_map"
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S_%f")[:-3]
    candidate = Path(directory) / f"{safe_name}_{timestamp}{extension}"

    def conflicts(path: Path) -> bool:
        if path.exists():
            return True
        return extension.lower() in (".yaml", ".yml") and (
            path.with_suffix(".pgm").exists())

    suffix_index = 1
    while conflicts(candidate):
        candidate = Path(directory) / (
            f"{safe_name}_{timestamp}_{suffix_index:02d}{extension}")
        suffix_index += 1
    return candidate

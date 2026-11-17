import json
from pathlib import Path

import pytest

from nav_app.features.map_library import (
    delete_map,
    prepare_map_deletion,
)
from nav_app.scene_catalog import read_scene


def make_scene(root, name="demo", cloud=None):
    root.mkdir(parents=True, exist_ok=True)
    (root / f"{name}.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (root / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    cloud_path = cloud or root / f"{name}.pcd"
    if cloud is None:
        cloud_path.write_bytes(b"pcd")
    manifest = root / f"{name}.scene.json"
    manifest.write_text(json.dumps({
        "version": 1,
        "title": name,
        "grid": f"{name}.yaml",
        "cloud": str(cloud_path),
        "frame": "map",
        "coordinates_verified": True,
    }), encoding="utf-8")
    return read_scene(manifest)


def test_deletes_local_package_and_grid_companion(tmp_path):
    scene = make_scene(tmp_path)
    plan = prepare_map_deletion(scene, tmp_path, [scene])

    assert delete_map(plan) > 0
    assert not (tmp_path / "demo.scene.json").exists()
    assert not (tmp_path / "demo.yaml").exists()
    assert not (tmp_path / "demo.pgm").exists()
    assert not (tmp_path / "demo.pcd").exists()


def test_single_map_delete_preserves_other_asset(tmp_path):
    scene = make_scene(tmp_path)
    plan = prepare_map_deletion(scene, tmp_path, [scene], "grid")

    delete_map(plan)
    assert scene.manifest.is_file()
    assert Path(scene.cloud).is_file()
    assert not Path(scene.grid).exists()
    assert not (tmp_path / "demo.pgm").exists()


def test_deleting_localization_map_also_deletes_bound_scan_context(tmp_path):
    scene = make_scene(tmp_path)
    scan_context = tmp_path / "demo.pcd.sc.v2"
    scan_context.write_bytes(b"scan-context")
    scene.manifest.write_text(json.dumps({
        "version": 2,
        "title": "demo",
        "grid": "demo.yaml",
        "cloud": "demo.pcd",
        "scan_context": "demo.pcd.sc.v2",
        "scan_context_verified": True,
        "frame": "map",
        "coordinates_verified": True,
    }), encoding="utf-8")
    scene = read_scene(scene.manifest)

    plan = prepare_map_deletion(scene, tmp_path, [scene], "cloud")
    delete_map(plan)

    assert scene.manifest.is_file()
    assert Path(scene.grid).is_file()
    assert not Path(scene.cloud).exists()
    assert not scan_context.exists()


def test_package_preserves_external_asset_and_single_delete_refuses_it(tmp_path):
    root = tmp_path / "library"
    external = tmp_path / "external-map.pcd"
    external.write_bytes(b"keep")
    scene = make_scene(root, cloud=external)
    plan = prepare_map_deletion(scene, root, [scene])

    assert external in plan.preserved
    delete_map(plan)
    assert external.read_bytes() == b"keep"
    with pytest.raises(ValueError, match="不在本地地图目录"):
        prepare_map_deletion(scene, root, [scene], "cloud")


def test_shared_asset_is_preserved_and_changed_snapshot_is_rejected(tmp_path):
    first = make_scene(tmp_path, "first")
    second_manifest = tmp_path / "second.scene.json"
    second_manifest.write_text(json.dumps({
        "version": 1,
        "title": "second",
        "grid": "first.yaml",
        "cloud": "first.pcd",
        "frame": "map",
        "coordinates_verified": True,
    }), encoding="utf-8")
    second = read_scene(second_manifest)
    plan = prepare_map_deletion(first, tmp_path, [first, second])
    assert Path(first.grid) in plan.preserved
    assert Path(first.cloud) in plan.preserved
    first.manifest.write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="已变化"):
        delete_map(plan)

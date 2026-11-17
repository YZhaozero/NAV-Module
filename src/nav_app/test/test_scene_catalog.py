import json
import os
import pytest
from nav_app.scene_catalog import discover_scenes, read_scene, rename_scene


def test_map_list_sorts_by_cloud_time_and_does_not_invent_creation_time(tmp_path):
    for name,stamp in [('a-new.pcd',2000000),('z-old.pcd',1000000)]:
        path=tmp_path/name
        path.write_text('fixture')
        os.utime(path,(stamp,stamp))
    scenes=discover_scenes(tmp_path)
    assert [s.title for s in scenes]==['a-new','z-old']
    assert all(s.time_label.startswith('点云文件时间 · ') for s in scenes)
    (tmp_path/'a-new.pcd').unlink()
    assert scenes[0].time_label=='点云文件时间未知'


def test_mismatched_legacy_files_are_not_usable_bundles(tmp_path):
    (tmp_path / "A.pcd").write_text("test")
    (tmp_path / "B.yaml").write_text("test")
    scenes = discover_scenes(tmp_path)
    assert len(scenes) == 1 and not scenes[0].ready
    assert scenes[0].cloud.endswith("A.pcd")
    assert not scenes[0].grid
    assert "栅格未绑定" in scenes[0].resource_summary


def test_same_name_does_not_certify_coordinates(tmp_path):
    (tmp_path / "A.pcd").write_text("test")
    (tmp_path / "A.yaml").write_text("test")
    assert not discover_scenes(tmp_path)[0].ready
    manifest = tmp_path / "A.scene.json"
    manifest.write_text(json.dumps(dict(version=1, title="Entrance", grid="A.yaml", cloud="A.pcd",
                                        coordinates_verified=True, destinations=[dict(name="Exit", xy=[1, 2])])) )
    scenes = discover_scenes(tmp_path)
    assert len(scenes) == 1 and scenes[0].ready
    assert scenes[0].grid == str((tmp_path / "A.yaml").resolve())
    assert read_scene(manifest).destinations[0]["xy"] == [1., 2.]


def test_same_name_yml_is_offered_as_registration_candidate(tmp_path):
    (tmp_path / "A.pcd").write_text("test")
    (tmp_path / "A.yml").write_text("test")
    scene = discover_scenes(tmp_path)[0]
    assert scene.grid == str((tmp_path / "A.yml").resolve())
    assert "栅格待登记" in scene.resource_summary


def test_numbered_grid_suffix_is_offered_after_name_collisions(tmp_path):
    (tmp_path / "A.pcd").write_text("test")
    (tmp_path / "A-grid-02.yaml").write_text("test")
    scene = discover_scenes(tmp_path)[0]
    assert scene.grid == str((tmp_path / "A-grid-02.yaml").resolve())


def test_v2_bundle_tracks_optional_verified_scan_context(tmp_path):
    for name in ("A.pcd", "A.yaml", "A.pcd.sc.v2"):
        (tmp_path / name).write_text("test")
    manifest = tmp_path / "A.scene.json"
    manifest.write_text(json.dumps(dict(
        version=2,
        title="Entrance",
        grid="A.yaml",
        cloud="A.pcd",
        scan_context="A.pcd.sc.v2",
        scan_context_verified=True,
        frame="map",
        coordinates_verified=True,
        destinations=[],
    )))
    scene = read_scene(manifest)
    assert scene.ready and scene.global_ready
    assert scene.resource_summary == "PCD ✓ · 栅格 ✓ · SC ✓"


def test_rename_scene_only_changes_display_title(tmp_path):
    for name in ("A.pcd", "A.yaml", "A.pcd.sc.v2"):
        (tmp_path / name).write_text("test")
    manifest = tmp_path / "A.scene.json"
    original = dict(
        version=2,
        title="旧名称",
        grid="A.yaml",
        cloud="A.pcd",
        scan_context="A.pcd.sc.v2",
        scan_context_verified=True,
        frame="map",
        coordinates_verified=True,
        destinations=[{"name": "出口", "xy": [1.0, 2.0]}],
        viewer_url="https://example.invalid/map",
        extension={"keep": True},
    )
    manifest.write_text(
        json.dumps(original, ensure_ascii=False), encoding="utf-8")

    renamed = rename_scene(manifest, "  一层大厅  ")
    document = json.loads(manifest.read_text(encoding="utf-8"))

    assert renamed.title == "一层大厅"
    assert renamed.manifest == manifest.resolve()
    assert document == {**original, "title": "一层大厅"}
    assert not list(tmp_path.glob("*.rename.tmp"))


@pytest.mark.parametrize("title", ["", "   ", "第一层\n大厅", "图" * 65])
def test_rename_scene_rejects_invalid_titles_without_changing_file(
    tmp_path, title,
):
    manifest = tmp_path / "A.scene.json"
    manifest.write_text(json.dumps({
        "version": 1,
        "title": "原名称",
        "grid": "",
        "cloud": "",
        "coordinates_verified": False,
        "destinations": [],
    }), encoding="utf-8")
    before = manifest.read_bytes()

    with pytest.raises(ValueError):
        rename_scene(manifest, title)

    assert manifest.read_bytes() == before

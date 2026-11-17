from pathlib import Path
from types import SimpleNamespace

import numpy as np

from nav_app.map_processing import (
    GridMap,
    load_grid_map,
    load_pcd,
    load_ply,
    load_pointcloud,
    pointcloud_to_grid,
    pointcloud2_to_pointcloud,
    save_grid_map,
    save_pcd,
)


def test_pointcloud2_preserves_packed_rgb_colors():
    dtype = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4"),
    ])
    records = np.zeros(2, dtype=dtype)
    records["x"] = [1.0, -2.0]
    records["y"] = [2.0, 3.0]
    records["z"] = [3.0, 4.0]
    expected = np.array([[255, 10, 20], [30, 240, 50]], dtype=np.uint8)
    packed = (
        expected[:, 0].astype(np.uint32) << 16
        | expected[:, 1].astype(np.uint32) << 8
        | expected[:, 2].astype(np.uint32)
    )
    records["rgb"] = packed.view("<f4")
    fields = [
        SimpleNamespace(name=name, offset=offset, datatype=7)
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))
    ]
    message = SimpleNamespace(
        fields=fields, width=2, height=1, point_step=16,
        is_bigendian=False, data=records.tobytes(),
        header=SimpleNamespace(frame_id="odom"),
    )
    cloud = pointcloud2_to_pointcloud(message)
    assert cloud.frame_id == "odom"
    assert cloud.color_source == "rgb"
    assert np.allclose(cloud.points, [[1, 2, 3], [-2, 3, 4]])
    assert np.array_equal(cloud.colors, expected)


def test_binary_pcd_round_trip(tmp_path: Path):
    points = np.array([
        [0.0, 1.0, 0.2],
        [2.0, -1.0, 0.8],
        [3.5, 4.0, -0.1],
    ], dtype=np.float32)
    path = save_pcd(tmp_path / "map", points)
    loaded = load_pcd(path)
    assert loaded.points.shape == (3, 3)
    assert np.allclose(loaded.points, points)


def test_binary_pcd_rgb_round_trip(tmp_path: Path):
    points = np.array([
        [0.0, 1.0, 0.2],
        [2.0, -1.0, 0.8],
        [3.5, 4.0, -0.1],
    ], dtype=np.float32)
    colors = np.array([
        [255, 10, 20],
        [30, 240, 50],
        [60, 70, 230],
    ], dtype=np.uint8)
    path = save_pcd(tmp_path / "colored_map", points, colors)
    loaded = load_pcd(path)
    assert np.allclose(loaded.points, points)
    assert np.array_equal(loaded.colors, colors)


def test_pcd_intensity_is_preserved_as_source_grayscale(tmp_path: Path):
    path = tmp_path / "intensity_map.pcd"
    path.write_text(
        "# .PCD v0.7\nVERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n"
        "0 0 0 0\n1 0 0 50\n2 0 0 100\n",
        encoding="ascii",
    )
    loaded = load_pcd(path)
    assert loaded.color_source == "intensity"
    assert loaded.colors.shape == (3, 3)
    assert np.all(loaded.colors[:, 0] == loaded.colors[:, 1])
    assert np.all(loaded.colors[:, 1] == loaded.colors[:, 2])
    assert np.all(np.diff(loaded.colors[:, 0].astype(np.int16)) > 0)


def test_ascii_ply_load_rgb_and_convert_to_grid(tmp_path: Path):
    path = tmp_path / "colored_ascii.ply"
    path.write_text(
        "ply\n"
        "format ascii 1.0\n"
        "element vertex 3\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "element face 1\nproperty list uchar int vertex_indices\n"
        "end_header\n"
        "0 0 0.2 255 0 10\n"
        "1 1 0.4 0 240 20\n"
        "2 0 0.6 30 40 230\n"
        "3 0 1 2\n",
        encoding="ascii",
    )
    cloud = load_pointcloud(path)
    assert cloud.points.shape == (3, 3)
    assert np.array_equal(cloud.colors[0], [255, 0, 10])
    grid = pointcloud_to_grid(
        cloud.points, resolution=0.5, z_min=0.0, z_max=1.0,
        padding=0.0, inflation_radius=0.0)
    assert grid.occupancy.shape == (3, 5)
    assert np.count_nonzero(grid.occupancy == 100) == 3


def test_binary_little_and_big_endian_ply(tmp_path: Path):
    expected = np.array([
        [-1.5, 2.0, 0.25],
        [3.0, -4.5, 1.75],
    ], dtype=np.float32)
    expected_colors = np.array([[1, 2, 3], [250, 240, 230]], dtype=np.uint8)
    for format_name, endian in (
        ("binary_little_endian", "<"),
        ("binary_big_endian", ">"),
    ):
        path = tmp_path / f"{format_name}.ply"
        dtype = np.dtype([
            ("x", endian + "f4"), ("y", endian + "f4"),
            ("z", endian + "f4"), ("red", "u1"),
            ("green", "u1"), ("blue", "u1"),
        ])
        records = np.empty(2, dtype=dtype)
        records["x"], records["y"], records["z"] = expected.T
        records["red"], records["green"], records["blue"] = expected_colors.T
        header = (
            "ply\n"
            f"format {format_name} 1.0\n"
            "element vertex 2\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        ).encode("ascii")
        path.write_bytes(header + records.tobytes())
        cloud = load_ply(path)
        assert np.allclose(cloud.points, expected)
        assert np.array_equal(cloud.colors, expected_colors)


def test_gaussian_splat_ply_decodes_color_shape_and_opacity(tmp_path: Path):
    path = tmp_path / "gaussian.ply"
    path.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float f_dc_0\nproperty float f_dc_1\nproperty float f_dc_2\n"
        "property float opacity\n"
        "property float scale_0\nproperty float scale_1\nproperty float scale_2\n"
        "property float rot_0\nproperty float rot_1\n"
        "property float rot_2\nproperty float rot_3\n"
        "end_header\n"
        "0 0 0 1 -1 0 0 0 -0.693147 -1.609438 1 0 0 0\n"
        "1 2 3 -1 1 0 2 -2 -2 -2 1 0 0 0\n",
        encoding="ascii",
    )
    cloud = load_ply(path)
    assert cloud.is_gaussian_splat
    assert cloud.colors[0, 0] > cloud.colors[0, 1]
    assert np.allclose(cloud.splat_scales[0], [1.0, 0.5, 0.2], atol=1e-5)
    assert np.isclose(cloud.splat_opacities[0], 0.5)
    assert cloud.splat_opacities[1] > cloud.splat_opacities[0]
    assert np.allclose(cloud.splat_rotations[:, 0], 1.0)


def test_pointcloud_to_grid_filters_height_and_marks_obstacles():
    points = np.array([
        [0.0, 0.0, 0.2],
        [1.0, 1.0, 0.5],
        [50.0, 50.0, 9.0],
    ], dtype=np.float32)
    grid = pointcloud_to_grid(
        points, resolution=0.1, z_min=0.0, z_max=1.0,
        padding=0.0, inflation_radius=0.0)
    assert grid.occupancy.shape == (11, 11)
    assert grid.occupancy[0, 0] == 100
    assert grid.occupancy[10, 10] == 100
    assert np.count_nonzero(grid.occupancy == 100) == 2
    assert grid.occupancy[5, 5] == -1
    assert np.count_nonzero(grid.occupancy == 0) == 0


def test_free_evidence_cannot_erase_obstacles_or_inflation():
    points = np.array([[0., 0., .5], [1., 1., .5]])
    grid = pointcloud_to_grid(points, resolution=.1, z_min=0, z_max=1,
        padding=0, inflation_radius=.1, free_points=np.array([[0., 0.], [.1, 0.], [.5, .5]]))
    assert grid.occupancy[0, 0] == 100
    assert grid.occupancy[0, 1] == 100
    assert grid.occupancy[5, 5] == 0
    assert grid.occupancy[5, 6] == -1


def test_explicit_unknown_fill_marks_map_bounds_free_but_keeps_obstacles():
    points = np.array([[0., 0., .5], [1., 1., .5]])
    grid = pointcloud_to_grid(
        points, resolution=.1, z_min=0, z_max=1,
        padding=0, inflation_radius=0, unknown_is_free=True)

    assert grid.occupancy[0, 0] == 100
    assert grid.occupancy[10, 10] == 100
    assert grid.occupancy[5, 5] == 0
    assert not np.any(grid.occupancy == -1)


def test_nav2_grid_round_trip(tmp_path: Path):
    occupancy = np.array([
        [0, 100, -1],
        [-1, 0, 100],
    ], dtype=np.int8)
    grid = GridMap(
        occupancy, 0.05, np.array([-1.0, 2.0, 0.25]))
    yaml_path, pgm_path = save_grid_map(tmp_path / "office.yaml", grid)
    assert yaml_path.exists()
    assert pgm_path.exists()
    loaded = load_grid_map(yaml_path)
    assert np.array_equal(loaded.occupancy, occupancy)
    assert np.allclose(loaded.origin, grid.origin)
    assert loaded.resolution == grid.resolution

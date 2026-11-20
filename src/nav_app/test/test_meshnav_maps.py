import json
import struct
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from nav_app.features.meshnav_maps import MeshSurface, generate_scene_mesh, load_mesh_surface, traversable_mesh
from nav_app.map_processing import GridMap, save_grid_map, save_pcd
from nav_app.qt_nav_node import MapPanel, MeshPreviewPanel
from nav_app.scene_catalog import read_scene


def separated_surface():
    vertices = np.array([[1, 1, 0], [3, 1, 0], [3, 3, 0], [1, 3, 0],
                         [5, 1, .3], [7, 1, .3], [7, 3, .3], [5, 3, .3]])
    return MeshSurface(vertices, [[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]])


def test_contains_ground_and_distinguishes_disconnected_regions():
    surface = separated_surface()
    first, second = surface.point([2, 2, .1]), surface.point([6, 2, .1])
    assert first[1] != second[1]
    np.testing.assert_allclose(second[0], [6., 2., .3])
    assert surface.point([4, 2, .1]) is None
    assert surface.point([3.01, 2, .1]) is None


def test_generated_mesh_is_linked_to_exact_pcd(tmp_path, monkeypatch):
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    x, y = np.meshgrid(np.arange(0, 3, .04), np.arange(0, 2, .04))
    flat = np.c_[x.ravel(), y.ravel(), np.full(x.size, -.2)]
    obstacles = np.c_[np.full(10, 1.5), np.linspace(.3, 1.5, 10), np.full(10, .1)]
    pcd = save_pcd(tmp_path / "map.pcd", np.r_[flat, obstacles])
    grid, _ = save_grid_map(tmp_path / "map.yaml", GridMap(np.zeros((31, 41)), .1, np.zeros(3)))
    manifest = tmp_path / "map.scene.json"
    manifest.write_text(json.dumps(dict(version=2, title="office", cloud=pcd.name, grid=grid.name,
                                         coordinates_verified=True, destinations=[{"name":"A","xy":[.5,1.]}])))
    scene = generate_scene_mesh(manifest)
    assert Path(scene.meshnav).is_file() and not scene.meshnav_error
    assert scene.destinations == [{"name":"A","xy":[.5,1.]}]
    assert generate_scene_mesh(manifest).meshnav == scene.meshnav
    old_mesh = Path(scene.meshnav)
    document = json.loads(manifest.read_text())
    document["meshnav"]["generator"] = "terrain-v1"
    manifest.write_text(json.dumps(document))
    scene = generate_scene_mesh(manifest)
    assert scene.meshnav != str(old_mesh) and old_mesh.is_file()
    entry = json.loads(manifest.read_text())["meshnav"]
    assert entry["file"] == old_mesh.name and entry["physical_v2"] == Path(scene.meshnav).name
    assert generate_scene_mesh(manifest).meshnav == scene.meshnav
    from nav_app.features.navigation_backends import MeshNavBackend
    from types import SimpleNamespace
    assert MeshNavBackend.mesh_path(SimpleNamespace(meshnav_map=str(old_mesh))) == Path(scene.meshnav)
    with pcd.open("ab") as stream:
        stream.write(b"\n")
    assert read_scene(manifest).meshnav_error


def test_mesh_overlay_matches_grid_and_keeps_gap_uncolored():
    app = QApplication.instance() or QApplication([])
    surface = separated_surface()
    panel = MapPanel()
    panel.resize(800, 480)
    panel.set_map(np.zeros((50, 90), dtype=np.int8), (0., 0.), .1, 0.)
    panel.set_meshnav_overlay(surface, surface.point([2, 2, 0])[1])
    panel.show()
    app.processEvents()
    rendered = panel.grab().toImage()
    colors = [rendered.pixelColor(panel.world_to_widget(np.asarray(xy)).toPoint())
              for xy in ((2., 2.), (6., 2.), (4., 2.))]
    green, yellow, gap = colors
    assert green.green() > green.red() + 40
    assert yellow.red() > yellow.blue() + 35
    assert abs(gap.green() - gap.red()) < 15
    panel.close()


def test_binary_ply_preserves_faces_and_mesh_view_picks_the_rendered_triangle(tmp_path):
    mesh_path = tmp_path / "floor.ply"
    vertices = [(0., 0., 0.), (4., 0., 0.), (0., 2., .4)]
    header = (b"ply\nformat binary_little_endian 1.0\nelement vertex 3\n"
              b"property float x\nproperty float y\nproperty float z\n"
              b"element face 1\nproperty list uchar int vertex_indices\nend_header\n")
    mesh_path.write_bytes(header + b"".join(struct.pack("<fff", *point) for point in vertices)
                          + struct.pack("<Biii", 3, 0, 1, 2))
    surface = load_mesh_surface(mesh_path)
    np.testing.assert_allclose(surface.vertices, vertices)
    assert surface.triangles.tolist() == [[0, 1, 2]]

    app = QApplication.instance() or QApplication([])
    panel = MeshPreviewPanel()
    panel.resize(640, 360)
    panel.set_mesh(surface)
    panel.show()
    app.processEvents()
    center = np.asarray(vertices).mean(axis=0)
    x, y, _ = panel._project_cloud_points(center[None, :])
    pixel = QPointF(float(x[0]), float(y[0]))
    fill = panel.grab().toImage().pixelColor(pixel.toPoint())
    assert fill.green() > fill.red() + 40
    np.testing.assert_allclose(panel.widget_to_world(pixel), center, atol=1.e-6)
    picks = []
    panel.point_selected_3d.connect(lambda *xyz: picks.append(xyz))
    QTest.mouseClick(panel, Qt.LeftButton, pos=pixel.toPoint())
    assert len(picks) == 1
    np.testing.assert_allclose(picks[0], center, atol=.03)
    assert panel.widget_to_world(QPointF(2, 2)) is None
    panel.close()


def terrain_rectangle(xmin, xmax, ymin, ymax, height):
    # Four measured points per 15 cm cell. The boundaries align with cell edges.
    x, y = np.meshgrid(np.arange(xmin + .03, xmax, .075), np.arange(ymin + .03, ymax, .075))
    return np.column_stack((x.ravel(), y.ravel(), np.broadcast_to(height(x), x.shape).ravel()))


def layered_terrain_points():
    lower = terrain_rectangle(0., 4.2, 0., 3., lambda x: 0.)
    ramp = terrain_rectangle(4.2, 10.2, 0., 3., lambda x: .275 * (x - 4.2))
    landing = terrain_rectangle(10.2, 11.7, 0., 5.4, lambda x: 1.65)
    return_lane = terrain_rectangle(4.2, 10.2, 3., 5.4, lambda x: 1.65)
    upper = terrain_rectangle(0., 4.2, 0., 5.4, lambda x: 1.65)
    return np.vstack((lower, ramp, landing, return_lane, upper))


def test_terrain_keeps_overlapping_floors_and_a_continuous_ramp():
    surface = MeshSurface(*traversable_mesh(layered_terrain_points()))
    low, low_component = surface.point([2., 1.5, 0.], max_height_error=.04)
    high, high_component = surface.point([2., 1.5, 1.65], max_height_error=.04)
    ramp, ramp_component = surface.point([7.2, 1.5, .825], max_height_error=.04)
    np.testing.assert_allclose([low[2], high[2], ramp[2]], [0., 1.65, .825], atol=.01)
    assert low_component == ramp_component == high_component
    assert surface.point([2., 1.5, .825], max_height_error=.04) is None

    faces = surface.vertices[surface.faces]
    slopes = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    assert np.max(np.linalg.norm(slopes[:, :2], axis=1) / slopes[:, 2]) <= .36
    assert np.any(np.ptp(faces[:, :, 2], axis=1) > .02)


def test_terrain_does_not_bridge_floors_or_mesh_under_an_obstacle():
    low = terrain_rectangle(0., 3., 0., 3., lambda x: 0.)
    high = terrain_rectangle(0., 3., 0., 3., lambda x: 1.65)
    low_ceiling = terrain_rectangle(.9, 2.1, .9, 2.1, lambda x: .45)
    surface = MeshSurface(*traversable_mesh(np.vstack((low, high, low_ceiling))))
    assert surface.point([.45, .45, 0.], max_height_error=.04)[1] != surface.point([.45, .45, 1.65])[1]
    assert surface.point([1.5, 1.5, 0.], max_height_error=.04) is None
    assert surface.point([1.5, 1.5, 1.65], max_height_error=.04)[0][2] == 1.65


def test_physical_mesh_keeps_floor_next_to_wall_and_only_reserves_vehicle_once():
    floor = terrain_rectangle(0., 4., 0., 3., lambda x: 0.)
    x = np.arange(.4, 3.6, .03)
    wall = np.c_[x, np.full_like(x, 1.52), np.full_like(x, .3)]
    surface = MeshSurface(*traversable_mesh(np.vstack((floor, wall))))
    assert surface.point([2., 1.48, 0.]) is None
    assert surface.point([2., 1.3, 0.]) is not None
    physical_edge = 1.425
    path = np.array([[1., physical_edge-.45, 0.], [3., physical_edge-.45, 0.]])
    assert surface.body_conflict(path, 0) is None
    shifted = path + [0., .17, 0.]
    outline, contacts = surface.body_conflict(shifted, 0)
    assert outline[:, 1].max() >= contacts[:, 1].min()


def test_swept_body_detects_internal_hole_between_safe_waypoints():
    floor = terrain_rectangle(0., 5., 0., 3., lambda x: 0.)
    surface = MeshSurface(*traversable_mesh(np.vstack((floor, [2.5, 1.4, .6]))))
    start = [1., 1.7, 0.]
    goal = [4., 1.7, 0.]
    assert all(surface.body_conflict([p], 0) is None for p in (start, goal))
    outline, contacts = surface.body_conflict([start, goal], 0)
    assert len(contacts) and np.all((2.2 < contacts[:, 0]) & (contacts[:, 0] < 2.8))
    assert np.all(contacts[:, 0] <= outline[:, 0].max() + 1.e-8)
    assert surface.body_conflict([[1., 2.1, 0.], [4., 2.1, 0.]], 0) is None


def test_padded_woosh_footprint_straight_corridor_and_corner_rotation():
    from nav_app.features.mesh_follow import BODY_POLYGON, PADDED_BODY
    import yaml
    cfg = yaml.safe_load((Path(__file__).parents[1]/"config/woosh_nav2.yaml").read_text())
    for name in ("global_costmap", "local_costmap"):
        prefix = f"{name}.{name}.ros__parameters."
        np.testing.assert_array_equal(BODY_POLYGON, json.loads(cfg[prefix + "footprint"]))
        np.testing.assert_allclose(PADDED_BODY, BODY_POLYGON + np.sign(BODY_POLYGON) * cfg[prefix + "footprint_padding"])
    surface = MeshSurface([[-2,-.34,0], [2,-.34,0], [2,.34,0], [-2,.34,0]], [[0,1,2],[0,2,3]])
    start, goal = [0.,0.,0.], [1.,0.,0.]
    assert surface.body_conflict([start,goal], 0, start_yaw=0.) is None
    assert surface.body_conflict([start], 0, start_yaw=np.pi/2) is None
    # Both endpoint orientations fit, but a corner crosses the border mid-turn.
    outline, contacts = surface.body_conflict([start,goal], 0, start_yaw=np.pi/2)
    assert np.any(np.isclose(np.abs(contacts[:,1]), .34))
    assert len(outline) >= 8
    assert surface.body_conflict([[0.,.045,0.]], 0, start_yaw=0.) is not None

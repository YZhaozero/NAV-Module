"""MeshNav terrain generation and geometric point selection in map coordinates."""

import json
import math
import os
from pathlib import Path
import tempfile
import uuid

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from ..map_processing import load_pcd, load_ply_triangles
from ..scene_catalog import read_scene
from .capture_quality import atomic_json
from .resource_lease import ResourceLease
from .mesh_follow import (CLEARANCE_LOW, CLEARANCE_HIGH,
                          footprint_planes, swept_footprints)


def traversable_mesh(points):
    """Triangulate observed layers in a gravity-aligned PCD without flattening Z."""
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 3:
        raise ValueError("有效点云不足，无法提取可通行面")

    spacing = .15
    max_grade = .36
    cells = np.floor(points[:, :2] / spacing).astype(np.int64)
    order = np.lexsort((points[:, 2], cells[:, 1], cells[:, 0]))
    cells, heights = cells[order], points[order, 2]
    breaks = np.any(np.diff(cells, axis=0) != 0, axis=1) | (np.diff(heights) > .07)
    first = np.r_[0, np.flatnonzero(breaks) + 1]
    ends = np.r_[first[1:], len(heights)]
    counts = ends - first
    # Dense vertical returns occupy a tall band within a cell. A local walking
    # surface gives a narrow band, even on a ramp.
    supported = (counts >= 3) & (heights[ends - 1] - heights[first] <= .11)
    z = (np.add.reduceat(heights, first) / counts)[supported]
    keys = cells[first[supported]]
    vertices = np.column_stack(((keys + .5) * spacing, z))
    columns = {}
    for index, (x, y) in enumerate(keys):
        columns.setdefault((x, y), []).append(index)

    neighbors = np.full((len(vertices), 3), -1, dtype=np.int64)
    for column, (dx, dy) in enumerate(((1, 0), (0, 1), (1, 1))):
        limit = max_grade * spacing * np.hypot(dx, dy)
        for (x, y), indices in columns.items():
            adjacent = columns.get((x + dx, y + dy))
            if adjacent is None:
                continue
            gaps = np.abs(z[indices, None] - z[adjacent])
            nearest = gaps.argmin(axis=1)
            reciprocal = gaps.argmin(axis=0)[nearest] == np.arange(len(indices))
            matched = reciprocal & (gaps[np.arange(len(indices)), nearest] <= limit)
            neighbors[np.asarray(indices)[matched], column] = np.asarray(adjacent)[nearest[matched]]

    faces = []
    for index, (east, north, diagonal) in enumerate(neighbors):
        if diagonal < 0:
            continue
        if east >= 0 and neighbors[east, 1] == diagonal:
            faces.append((index, east, diagonal))
        if north >= 0 and neighbors[north, 0] == diagonal:
            faces.append((index, diagonal, north))
    if not faces:
        raise ValueError("没有足够连续的可通行面，请核对点云的地面覆盖和 Z 轴方向")
    faces = np.asarray(faces)
    used, inverse = np.unique(faces, return_inverse=True)
    vertices, faces = vertices[used], inverse.reshape(-1, 3)

    a, b, c = vertices[faces].transpose(1, 0, 2)
    normals = np.cross(b - a, c - a)
    slope = np.linalg.norm(normals[:, :2], axis=1) / normals[:, 2]
    faces = faces[slope <= max_grade]

    # Remove only triangles with an observed obstruction directly above them.
    # The planner tests the vehicle radius on these physical surface boundaries.
    cloud_index = cKDTree(points[:, :2])
    clear = np.ones(len(faces), dtype=bool)
    for offset in range(0, len(faces), 256):
        tri = vertices[faces[offset:offset+256]]
        centers = tri[:, :, :2].mean(axis=1)
        radius = np.linalg.norm(tri[:, :, :2] - centers[:, None], axis=2).max(axis=1)
        nearby = cloud_index.query_ball_point(centers, radius + 1.e-7)
        owners = np.repeat(np.arange(len(tri)), [len(row) for row in nearby])
        indices = np.concatenate(nearby).astype(int)
        delta = points[indices] - tri[owners, 0]
        a, b = (tri[owners, 1:] - tri[owners, :1]).transpose(1, 0, 2)
        det = a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0]
        u = (delta[:, 0]*b[:, 1] - delta[:, 1]*b[:, 0]) / det
        v = (a[:, 0]*delta[:, 1] - a[:, 1]*delta[:, 0]) / det
        height = delta[:, 2] - u*a[:, 2] - v*b[:, 2]
        blocked = ((u >= -1.e-6) & (v >= -1.e-6) & (u+v <= 1.+1.e-6)
                   & (height >= CLEARANCE_LOW) & (height <= CLEARANCE_HIGH))
        clear[offset + owners[blocked]] = False
    faces = faces[clear]
    if not len(faces):
        raise ValueError("采集范围内未找到满足坡度和净空条件的可通行面")
    used, inverse = np.unique(faces, return_inverse=True)
    return vertices[used], inverse.reshape(-1, 3)


def generate_scene_mesh(manifest):
    """Publish a new mesh only after its source and manifest still agree."""
    manifest = Path(manifest).resolve()
    with ResourceLease("live"):
        scene = read_scene(manifest)
        if not scene.ready:
            raise ValueError("请先保存并核对地图包的点云与栅格")
        cloud = Path(scene.cloud)
        info = cloud.stat()
        identity = (info.st_size, info.st_mtime_ns)
        document = json.loads(manifest.read_text(encoding="utf-8"))
        bound = document.get("meshnav", {})
        if scene.meshnav and not scene.meshnav_error and (
                bound.get("generator") == "physical-v2" or bound.get("physical_v2")):
            return scene
        vertices, faces = traversable_mesh(load_pcd(cloud).points)

        mesh = manifest.with_name(f"{manifest.stem}-{uuid.uuid4().hex[:8]}.mesh.ply")
        with tempfile.NamedTemporaryFile(mode="w", dir=manifest.parent, prefix=".meshnav-",
                                         delete=False, encoding="ascii") as stream:
            temporary = Path(stream.name)
            try:
                stream.write(
                    f"ply\nformat ascii 1.0\nelement vertex {len(vertices)}\n"
                    "property float x\nproperty float y\nproperty float z\n"
                    f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
                np.savetxt(stream, vertices, fmt="%.7f")
                np.savetxt(stream, np.c_[np.full(len(faces), 3), faces], fmt="%d")
                stream.flush()
                os.fsync(stream.fileno())
                current = read_scene(manifest)
                info = cloud.stat()
                if (not current.ready or current.cloud != scene.cloud or current.grid != scene.grid
                        or (info.st_size, info.st_mtime_ns) != identity):
                    raise ValueError("生成期间地图包或 PCD 已变化，请重新生成")
                document = json.loads(manifest.read_text(encoding="utf-8"))
                if document.get("meshnav", {}) != bound:
                    raise ValueError("生成期间网格关联已变化，请刷新地图包")
                if (bound.get("generator") == "terrain-v1" and
                        (bound.get("pcd_size"), bound.get("pcd_mtime_ns")) == identity):
                    # An older application continues to read the original file on rollback.
                    document["meshnav"]["physical_v2"] = mesh.name
                else:
                    document["meshnav"] = dict(file=mesh.name, pcd_size=identity[0],
                                               pcd_mtime_ns=identity[1], generator="physical-v2")
                temporary.replace(mesh)
                atomic_json(manifest, document)
            finally:
                temporary.unlink(missing_ok=True)
        return read_scene(manifest)


def load_mesh_surface(path):
    return MeshSurface(*load_ply_triangles(path))


class MeshSurface:
    """Triangle containment and connected components of the mesh loaded by MeshNav."""

    def __init__(self, vertices, faces):
        self.vertices = np.asarray(vertices, dtype=float)
        faces = np.asarray(faces, dtype=np.int64)
        if not len(faces) or not np.isfinite(self.vertices).all() or faces.min() < 0 or faces.max() >= len(vertices):
            raise ValueError("MeshNav 三角网格数据无效")
        xy = self.vertices[faces, :2]
        edges = xy[:, 1:] - xy[:, :1]
        determinant = edges[:, 0, 0] * edges[:, 1, 1] - edges[:, 1, 0] * edges[:, 0, 1]
        horizontal = np.abs(determinant) > 1.e-9
        if not horizontal.any():
            raise ValueError("MeshNav 网格没有可用于俯视选点的地面")
        self.triangles = faces
        self.faces = faces[horizontal]
        self._xy, self._edges, self._det = xy[horizontal], edges[horizontal], determinant[horizontal]
        centers = self._xy.mean(axis=1)
        self._face_index = cKDTree(centers)
        self._radius = float(np.linalg.norm(self._xy - centers[:, None, :], axis=2).max()) + 1.e-7
        links = np.vstack((self.faces[:, [0, 1]], self.faces[:, [1, 2]], self.faces[:, [2, 0]]))
        graph = coo_matrix((np.ones(len(links)), links.T), shape=(len(vertices), len(vertices))).tocsr()
        _, labels = connected_components(graph, directed=False)
        self.components = labels[self.faces[:, 0]]
        self.triangle_components = labels[faces[:, 0]]
        edges, counts = np.unique(np.sort(links, axis=1), axis=0, return_counts=True)
        boundary = edges[counts == 1]
        self._border = self.vertices[boundary]
        self._border_labels = labels[boundary[:, 0]]
        self._border_index = cKDTree(self._border[:, :, :2].mean(axis=1))
        self._border_half = .5 * np.linalg.norm(self._border[:, 1, :2] - self._border[:, 0, :2],
                                                axis=1).max(initial=0.)

    def _ground_samples(self, xy):
        neighbors = self._face_index.query_ball_point(xy, self._radius)
        owners = np.repeat(np.arange(len(xy)), [len(row) for row in neighbors])
        nearby = np.concatenate(neighbors).astype(int)
        delta = xy[owners] - self._xy[nearby, 0]
        a, b = self._edges[nearby].transpose(1, 0, 2)
        determinant = self._det[nearby]
        u = (delta[:, 0] * b[:, 1] - delta[:, 1] * b[:, 0]) / determinant
        v = (a[:, 0] * delta[:, 1] - a[:, 1] * delta[:, 0]) / determinant
        inside = (u >= -1.e-6) & (v >= -1.e-6) & (u+v <= 1.+1.e-6)
        nearby = nearby[inside]
        triangles = self.vertices[self.faces[nearby]]
        weights = np.column_stack((1.-u[inside]-v[inside], u[inside], v[inside]))
        ground = np.sum(triangles * weights[:, :, None], axis=1)
        return owners[inside], ground, self.components[nearby]

    def body_conflict(self, path, component, *, start_yaw=None, sweeps=None):
        """Return the checked footprint and failing map-frame contacts."""
        path = np.atleast_2d(np.asarray(path, dtype=float))
        # Also check the support height between graph vertices and at both exact clicks.
        # Crossing between floors can be invalid even with no nearby exposed edge.
        lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        intervals = np.ceil(lengths / .075).astype(int)
        probes = [np.linspace(a, b, steps, endpoint=False) for a, b, steps
                  in zip(path[:-1], path[1:], intervals) if steps]
        samples = np.vstack([*probes, path[-1:]])
        owners, ground, labels = self._ground_samples(samples[:, :2])
        supported = (labels == component) & (np.abs(ground[:, 2] - samples[owners, 2]) <= .25)
        missing = np.flatnonzero(~np.isin(np.arange(len(samples)), owners[supported]))
        if len(missing):
            point = samples[missing[0]]
            return np.empty((0, 3)), point[None, :]
        if sweeps is None:
            positions, headings = [path[0]], [start_yaw]
            for a, b in zip(path, path[1:]):
                if np.array_equal(a, b):
                    continue
                direction = math.atan2(b[1] - a[1], b[0] - a[0])
                if headings[0] is None:
                    headings[0] = direction
                positions.extend((a, b))
                headings.extend((direction, direction))
            if headings[0] is None:
                headings[0] = 0.
            sweeps = swept_footprints(positions, headings)
        for outline, dz in sweeps:
            center = outline.mean(axis=0)
            radius = np.max(np.linalg.norm(outline[:, :2] - center[:2], axis=1))
            ids = np.asarray(self._border_index.query_ball_point(
                center[:2], radius + self._border_half + 1.e-8), dtype=int)
            edges = self._border[ids[self._border_labels[ids] == component]]
            if not len(edges):
                continue
            # Clip each border segment against the convex footprint and the
            # support-height band. This catches edge crossings and holes inside the body.
            normals, limits = footprint_planes(outline)
            normals = np.column_stack((normals, np.zeros(len(normals))))
            normals = np.vstack((normals, [0., 0., 1.], [0., 0., -1.]))
            limits = np.r_[limits, center[2] + dz + .25, -center[2] + dz + .25]
            a = edges[:, 0]
            delta = edges[:, 1] - a
            gap = limits + 1.e-9 - a @ normals.T
            rate = delta @ normals.T
            parallel = np.abs(rate) < 1.e-12
            t = np.divide(gap, rate, out=np.zeros_like(gap), where=~parallel)
            lower = np.maximum(0., np.where(rate < -1.e-12, t, -np.inf).max(axis=1))
            upper = np.minimum(1., np.where(rate > 1.e-12, t, np.inf).min(axis=1))
            hits = (lower <= upper) & ~np.any(parallel & (gap < 0.), axis=1)
            if np.any(hits):
                contacts = a[hits] + ((lower[hits] + upper[hits]) / 2)[:, None] * delta[hits]
                return outline, contacts
        return None

    def obstacle_in_clearance(self, points, support_z, component):
        owners, ground, labels = self._ground_samples(points[:, :2])
        local = (labels == component) & (np.abs(ground[:, 2] - support_z) <= .25)
        # Unknown ground at a real nearby return cannot establish clearance.
        if np.unique(owners[local]).size < len(points):
            return True
        above_ground = points[owners[local], 2] - ground[local, 2]
        return bool(np.any((above_ground >= CLEARANCE_LOW) & (above_ground <= CLEARANCE_HIGH)))

    def covers(self, xy, z, component, max_height_error):
        owners, ground, labels = self._ground_samples(xy)
        heights = np.broadcast_to(z, (len(xy),))[owners]
        valid = (labels == component) & (np.abs(ground[:, 2] - heights) <= max_height_error)
        return np.unique(owners[valid]).size == len(xy)

    def point(self, xyz, *, max_height_error=np.inf):
        _, ground, labels = self._ground_samples(np.asarray(xyz)[None, :2])
        if not len(ground):
            return None
        height_error = np.abs(ground[:, 2] - xyz[2])
        face = np.argmin(height_error)
        if height_error[face] > max_height_error:
            return None
        return ground[face], int(labels[face])


"""Track one confirmed map-frame XYZ route on a forward-driving WOOSH base."""
import math

import numpy as np
from scipy.spatial import ConvexHull


# ROS1 move_base footprint in base_footprint, including its axis-wise padding rule.
BODY_POLYGON = np.array([[.29,.23],[.235,.275],[-.235,.275],[-.29,.23],
                         [-.29,-.23],[-.235,-.275],[.235,-.275],[.29,-.23]])
PADDED_BODY = BODY_POLYGON + .025 * np.sign(BODY_POLYGON)
CORNER_RADIUS = np.linalg.norm(PADDED_BODY, axis=1).max()
INSCRIBED_RADIUS = .30
MAX_SPEED = .10
MAX_YAW = .35
CLEARANCE_LOW = .05
CLEARANCE_HIGH = 1.50


def body_outline(center, yaw, polygon=PADDED_BODY):
    c, s = math.cos(yaw), math.sin(yaw)
    xy = polygon @ np.array([[c, s], [-s, c]])
    return center + np.column_stack((xy, np.zeros(len(xy))))


def swept_footprints(path, yaws):
    """Convex enclosures of each short translation and shortest signed turn."""
    path = np.asarray(path, dtype=float)
    first = body_outline(path[0], yaws[0])
    yield first, 0.
    for a, b, yaw, target in zip(path, path[1:], yaws, yaws[1:]):
        delta = b - a
        angle = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        if not angle and not np.any(delta):
            continue
        steps = max(1, math.ceil(np.linalg.norm(delta) / .075), math.ceil(abs(angle) / .175))
        step_angle = abs(angle) / steps
        # Corners sweep arcs between the endpoint polygons. Enclose the sagitta
        # of each corner, and of the center chord during curved travel.
        sagitta = (2 * CORNER_RADIUS * math.sin(step_angle / 4)**2
                   + .5 * np.linalg.norm(delta[:2]) / steps * math.tan(step_angle / 4))
        for i in range(1, steps + 1):
            center = a + delta * ((i - .5) / steps)
            last = body_outline(a + delta * (i / steps), yaw + angle * (i / steps))
            vertices = np.vstack((first[:, :2], last[:, :2])) - center[:2]
            xy = vertices[ConvexHull(vertices).vertices] * (1. + sagitta / INSCRIBED_RADIUS)
            yield center + np.column_stack((xy, np.zeros(len(xy)))), abs(delta[2]) / (2 * steps)
            first = last


def footprint_planes(outline):
    """Outward unit normals for the CCW outline, and their plane offsets."""
    xy = outline[:, :2]
    edges = np.roll(xy, -1, axis=0) - xy
    normals = np.column_stack((edges[:, 1], -edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return normals, np.einsum('ij,ij->i', normals, xy)


class FootprintError(ValueError):
    def __init__(self, conflict, message):
        super().__init__(message)
        self.conflict = conflict


class MeshFollower:
    def __init__(self, path):
        path = np.array(path, dtype=float, copy=True)
        if path.ndim != 2 or path.shape[1] != 3 or not np.isfinite(path).all() or len(path) < 2:
            raise ValueError("MeshNav 路线缺少有效三维坐标")
        self.path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1.e-6]]
        if len(self.path) < 2:
            raise ValueError("MeshNav 路线过短")
        self.segments = np.diff(self.path, axis=0)
        self.length = np.linalg.norm(self.segments, axis=1)
        self.distance = np.r_[0., np.cumsum(self.length)]
        self.progress = 0.

    def command(self, position, yaw, surface, component, obstacles, measured):
        first = max(0, self.distance.searchsorted(self.progress - .12) - 1)
        last = min(len(self.length), self.distance.searchsorted(self.progress + .42))
        segment = self.segments[first:last]
        start = self.path[first:last]
        lengths = self.length[first:last]
        along = self.distance[first:last]
        fraction = np.sum((position - start) * segment, axis=1) / lengths**2
        offset = np.clip(fraction * lengths, np.maximum(0., self.progress - .12 - along),
                         np.minimum(lengths, self.progress + .42 - along))
        error = np.linalg.norm(position - (start + (offset / lengths)[:, None] * segment), axis=1)
        index = np.argmin(error)
        if error[index] > .23:
            raise ValueError("已偏离确认路线，请重新定位或规划")
        self.progress = max(self.progress, along[index] + offset[index])
        remaining = self.distance[-1] - self.progress
        if remaining <= .10 and np.linalg.norm(position - self.path[-1]) <= .10:
            return None

        target = min(self.progress + .35, self.distance[-1])
        dx, dy = [np.interp(target, self.distance, self.path[:, i]) - position[i] for i in (0, 1)]
        angle = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        curvature = 2. * math.sin(angle) / max(.08, math.hypot(dx, dy))
        speed = min(MAX_SPEED, .6 * remaining, MAX_YAW / max(abs(curvature), .01))
        if abs(angle) > .65:
            speed, turn = 0., math.copysign(MAX_YAW, angle)
        else:
            turn = float(np.clip(speed * curvature, -MAX_YAW, MAX_YAW))

        # Reserve reaction and braking distance at the greater of measured/requested speed.
        velocity = max(speed, abs(measured[0]), abs(measured[1]))
        stopping = .12 + velocity * .35 + velocity**2 / (2 * .30)
        travel = np.linspace(0., stopping if velocity > .001 else 0., 4)
        arc = travel * (curvature if speed > 0. else 0.)
        centers = np.column_stack((travel * np.sinc(arc / np.pi),
                                   travel * np.sinc(arc / (2*np.pi))**2 * arc / 2.))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        world_centers = centers @ rotation.T + position[:2]
        sweep = np.column_stack((world_centers, np.full(len(world_centers), position[2])))
        sweeps = list(swept_footprints(sweep, yaw + arc))
        # Include angular reaction and braking even when turning in place.
        for rate in {measured[2], turn}:
            if rate:
                stop_angle = rate * .35 + rate * abs(rate) / (2 * .50)
                sweeps.extend(swept_footprints([position, position], [yaw, yaw + stop_angle]))
        if len(obstacles):
            # Same ground-relative clearance used on the native .5 platform. Keep
            # height until testing actual returns against the swept volume.
            z = obstacles[:, 2] - position[2]
            candidates = obstacles[(z >= CLEARANCE_LOW - .25) & (z <= CLEARANCE_HIGH + .25)]
            inside = np.zeros(len(candidates), dtype=bool)
            for outline, _ in sweeps:
                normals, limits = footprint_planes(outline)
                inside |= np.all(candidates[:, :2] @ normals.T <= limits + .08, axis=1)
            if inside.any() and surface.obstacle_in_clearance(candidates[inside], position[2], component):
                raise ValueError("车体行驶范围内有三维障碍物")
        conflict = surface.body_conflict(sweep, component, sweeps=sweeps)
        if conflict is not None:
            raise FootprintError(conflict, "车体行驶范围超出当前可通行网格，请查看红色标记")
        return speed, turn

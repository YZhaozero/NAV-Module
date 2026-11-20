"""Display-only route cues adapted from the upstream walking geometry."""
import math
import time
import numpy as np


def route_guidance(points, position, yaw, progress=0.):
    """Use local route progress and robot heading; never command robot motion."""
    position = np.asarray(position, float)
    route = np.asarray(points, float)[:, :len(position)]
    delta = np.diff(route, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    distances = np.r_[0., np.cumsum(lengths)]
    fraction = np.clip(np.sum((position-route[:-1])*delta, axis=1) / np.maximum(lengths**2, 1e-8), 0., 1.)
    projected = route[:-1] + fraction[:, None]*delta
    along = distances[:-1] + fraction*lengths
    lateral = np.linalg.norm(projected-position, axis=1)
    eligible = (along >= progress-.75) & (along <= progress+3.)
    lateral[~eligible] = np.inf
    index = int(np.argmin(lateral))
    if lateral[index] > 1.5:
        return dict(live=False, instruction='已偏离路线，请查看地图', progress=progress,
                    remaining_m=None, angle=None)
    progress = max(progress, float(along[index]))
    remaining = max(0., float(distances[-1])-progress)

    def at(distance):
        return np.array([np.interp(distance, distances, route[:, axis]) for axis in (0, 1)])

    target = at(min(float(distances[-1]), progress+1.5)) - position[:2]
    angle = math.atan2(math.sin(math.atan2(target[1], target[0])-yaw),
                       math.cos(math.atan2(target[1], target[0])-yaw))
    direction = ('请转身' if abs(angle)>2.2 else '向左转' if angle>.6 else
                 '向右转' if angle<-.6 else '沿路线前行')
    display_angle = angle
    if remaining < .8 and np.linalg.norm(position-route[-1]) < .8:
        direction = '接近目的地'
        display_angle = 0.
    elif abs(angle) <= .6:
        # Three-metre before/after chords suppress tiny planner-segment bends.
        upcoming = None
        for i in range(1, len(route)-1):
            distance = distances[i]-progress
            if not 1. <= distance <= 20.:
                continue
            before = route[i, :2]-at(max(0., distances[i]-3.))
            after = at(min(float(distances[-1]), distances[i]+3.))-route[i, :2]
            if min(np.linalg.norm(before), np.linalg.norm(after)) < .5:
                continue
            turn = math.atan2(np.cross(before, after), np.dot(before, after))
            # A dense planner path crosses the threshold before the bend itself.
            # Report the first bend's peak, so distance does not depend on sampling.
            if upcoming is not None and abs(turn) < abs(upcoming[1])-1e-6:
                break
            if abs(turn) >= math.radians(35) and (upcoming is None or abs(turn) > abs(upcoming[1])):
                upcoming = distance, turn
        if upcoming is not None:
            distance, turn = upcoming
            maneuver = '掉头' if abs(turn)>math.radians(150) else '左转' if turn>0 else '右转'
            direction = f'前方 {distance:.0f} 米{maneuver}'
    return dict(live=True, instruction=direction, progress=progress,
                remaining_m=remaining, angle=display_angle)


def route_chevrons(center, valid, bounds, spacing=100.):
    """Screen-space arrows in planned route order; never create a location dot."""
    center = np.asarray(center, float)
    valid = np.asarray(valid, bool)
    xmin, ymin, xmax, ymax = bounds
    result = []
    pending = 35.
    for index in range(1, len(center)):
        if not (valid[index-1] and valid[index]):
            pending = 35.
            continue
        a, b = center[index-1], center[index]
        delta = b - a
        length = float(np.linalg.norm(delta))
        if not math.isfinite(length) or length < 1e-6:
            continue
        direction = delta / length
        # Skip off-screen distances arithmetically; no pixel-length loop.
        lo, hi = 0., length
        for axis, lower, upper in ((0, xmin+12, xmax-12), (1, ymin+12, ymax-12)):
            if abs(direction[axis]) < 1e-9:
                if not lower <= a[axis] <= upper:
                    hi = -1.
            else:
                first, last = (lower-a[axis])/direction[axis], (upper-a[axis])/direction[axis]
                lo, hi = max(lo, min(first, last)), min(hi, max(first, last))
        sample = pending + max(0., math.ceil((lo-pending)/spacing)) * spacing
        while sample <= hi and len(result) < 12:
            tip = a + sample * direction
            normal = np.array([-direction[1], direction[0]])
            result.append(np.array([tip-10*direction+6*normal, tip, tip-10*direction-6*normal]))
            sample += spacing
        if len(result) >= 12:
            break
        pending = (pending-length) % spacing
        if pending < 1e-6:
            pending = spacing
    return result


def guidance_state(node):
    def waiting(message):
        return dict(live=False, instruction=message, remaining_m=None, angle=None)
    if not node.navigation_active:
        return waiting('等待导航')
    if node.path_error:
        return waiting(node.path_error)
    if node.require_localizer_quality and not node.localization_tools.health.valid(node.get_clock().now().nanoseconds):
        return waiting('地图定位失效，方向提示暂停')
    if len(node.path) < 2:
        return waiting('等待规划路线')
    stamp = node.image_stamp
    if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
        return waiting('等待有效相机时间')
    age = (node.get_clock().now().nanoseconds-stamp.sec*10**9-stamp.nanosec)/1e9
    if not -.05 <= age <= .5 or time.monotonic()-node._image_received_at > .5 or node.camera_error:
        return waiting('相机画面延迟，方向提示暂停')
    matrix = node.lookup_matrix(node.path_frame, node.base_frame, stamp)
    if matrix is None:
        return waiting('等待最新位置与方向')
    if node.path_frame == node.base_frame and time.monotonic()-node._path_received_at > .5:
        return waiting('等待最新局部路线')
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    position = matrix[:2, 3]
    if getattr(node, "_meshnav_active", False):
        ground = node.lookup_matrix(node.path_frame, node.ground_frame, stamp)
        position = (matrix if ground is None else ground)[:3, 3]
    cue = route_guidance(node.path, position, yaw, node._guidance_progress)
    node._guidance_progress = cue['progress']
    return cue

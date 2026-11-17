"""Bounded, nonblocking selection of motion-consistent GNSS/LIO pairs.

Consistency is an empirical trajectory check, not receiver accuracy. A rigid
transform preserves distances, so this check does not require an initial yaw.
"""
from collections import deque
import math


class StableAlignmentSamples:
    def __init__(self):
        self.recent = deque(maxlen=6)
        self.keyframes = deque(maxlen=100)
        self.rejected = 0
        self.reason = '正在收集连续卫星观测；建图和录包继续'

    @property
    def pairs(self):
        return [(p, q) for _, p, q, _ in self.keyframes]

    @property
    def weights(self):
        return [w for _, _, _, w in self.keyframes]

    def add(self, stamp, local, world, weight=1.):
        if self.recent and stamp <= self.recent[-1][0]:
            return False
        if self.recent and stamp-self.recent[-1][0] > 1.5:
            self.recent.clear()
        while self.keyframes and stamp-self.keyframes[0][0] > 180.:
            self.keyframes.popleft()
        if not math.isfinite(weight) or weight<=0:return False
        self.recent.append((stamp, tuple(local), tuple(world), weight))
        if len(self.recent) < 6:
            self.reason = '正在收集连续卫星观测；建图和录包继续'
            return False
        rows = list(self.recent)
        # Compare displacement lengths across the entire small window, rather
        # than mistaking a frozen GNSS coordinate for a stable moving receiver.
        errors = [abs(math.dist(a[1][:2], b[1][:2])-math.dist(a[2][:2], b[2][:2]))
                  for i, a in enumerate(rows) for b in rows[i+1:]]
        if math.sqrt(sum(e*e for e in errors)/len(errors)) > .4 or max(errors) > .8:
            self.rejected += 1
            self.reason = '卫星与雷达移动不一致，暂不采用这组点；建图和录包继续'
            return False
        mass=sum(r[3] for r in rows)
        p = tuple(sum(r[3]*r[1][i] for r in rows)/mass for i in range(3))
        q = tuple(sum(r[3]*r[2][i] for r in rows)/mass for i in range(3))
        # Apply identical averaging to both sides of the correspondence. Long
        # stops must not fill the history with duplicate positions or evict the
        # earlier travelled baseline needed to observe heading.
        if self.keyframes and math.dist(p[:2], self.keyframes[-1][1][:2]) < .75:
            self.reason = f'已保留{len(self.keyframes)}个稳定轨迹点，请平稳移动；建图和录包继续'
            return False
        self.keyframes.append((stamp, p, q, mass/len(rows)))
        self.reason = f'已保留{len(self.keyframes)}个稳定轨迹点，正在核对方向；建图和录包继续'
        return True

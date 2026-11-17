"""Small display-only buffer for images arriving before their timestamped TF."""
from collections import deque
from dataclasses import dataclass


@dataclass
class DisplayFrame:
    image: object
    stamp: object
    received: float
    frame: str
    raw_frame: str

    @property
    def nanoseconds(self):
        return self.stamp.sec * 10**9 + self.stamp.nanosec


class ImageTimeBuffer:
    def __init__(self, capacity=6, max_bytes=24 * 1024 * 1024):
        self.frames = deque()
        self.capacity, self.max_bytes = capacity, max_bytes
        self.bytes = 0

    def append(self, frame):
        if self.frames and (frame.nanoseconds <= self.frames[-1].nanoseconds or
                            frame.frame != self.frames[-1].frame):
            self.frames.clear()
            self.bytes = 0
        self.frames.append(frame)
        self.bytes += frame.image.nbytes
        # One oversized source frame is already the preview's unavoidable cost;
        # never retain a second one or an unbounded raw ROS message queue.
        while len(self.frames) > 1 and (len(self.frames) > self.capacity or self.bytes > self.max_bytes):
            self.bytes -= self.frames.popleft().image.nbytes

    def select(self, now_ns, now_monotonic, ready=None):
        if not self.frames:
            return None
        if ready is not None:
            for frame in reversed(self.frames):
                if (-.05 <= (now_ns - frame.nanoseconds) / 1e9 <= .5 and
                        now_monotonic - frame.received <= .5 and ready(frame)):
                    return frame
        # Keep the camera visible when localization is absent; projection reports
        # its existing reason instead of freezing or substituting a latest TF.
        return self.frames[-1]

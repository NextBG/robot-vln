"""ROS-free CityWalker client logic: 1 Hz frame/odometry history, request building and
tracking of the returned local plan.

Frames: odometry frame (x, y, yaw) from the S1 wheel odometry; body frame x forward, y left.
The plan (body frame at the request's current frame) is stored in the odometry frame, so it stays
valid while the robot moves during the HTTP round trip and between replies.
"""
import math
from collections import deque

import numpy as np


def to_body(pose, pts):
    """odometry-frame points -> body frame of `pose` (x, y, yaw)"""
    x, y, yaw = pose
    d = np.asarray(pts, float).reshape(-1, 2) - (x, y)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)


def from_body(pose, pts):
    x, y, yaw = pose
    p = np.asarray(pts, float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], 1)


class CityWalkerPlanner:
    context = 5

    def __init__(self, period=1.0, plan_timeout=3.0, lookahead=1.5, step_scale=0.5, hold_dist=0.3):
        self.period = period
        self.plan_timeout = plan_timeout
        self.lookahead = lookahead
        self.step_scale = step_scale
        self.hold_dist = hold_dist
        self.frames = deque(maxlen=self.context)   # (t, jpeg bytes, odom pose)
        self.plan = None                           # (t, odom points (5, 2), arrive_prob)

    def reset(self):
        self.frames.clear()
        self.plan = None

    def want_frame(self, t):
        return not self.frames or t - self.frames[-1][0] >= 0.95 * self.period

    def add_frame(self, t, jpeg, pose):
        if not self.want_frame(t):
            return False
        if self.frames and t - self.frames[-1][0] > 2.5 * self.period:
            self.frames.clear()     # gap (camera dropped out): history no longer 1 Hz
        self.frames.append((t, jpeg, tuple(pose)))
        return True

    def ready(self):
        return len(self.frames) == self.context

    def build_request(self, target_odom):
        """target in the odometry frame -> (meta json, [jpeg x5], odom pose and time of the current frame)"""
        if not self.ready():
            return None
        frames = list(self.frames)
        cur = frames[-1][2]
        hist = to_body(cur, [f[2][:2] for f in frames])
        tgt = to_body(cur, target_odom)[0]
        meta = {'hist': np.round(hist, 3).tolist(),
                'target': [round(float(tgt[0]), 3), round(float(tgt[1]), 3)],
                'step_scale': self.step_scale}
        return meta, [f[1] for f in frames], cur, frames[-1][0]

    def set_plan(self, t, pose, waypoints_body, arrive_prob=0.0):
        self.plan = (t, from_body(pose, waypoints_body), float(arrive_prob))

    def plan_age(self, t):
        return None if self.plan is None else t - self.plan[0]

    def path_body(self, pose):
        return None if self.plan is None else to_body(pose, self.plan[1])

    def target(self, t, pose):
        """Body-frame point to steer at, 'hold' if the fresh plan says stand still,
        None when there is no usable plan (caller falls back to GPS pure pursuit)."""
        if self.plan is None or t - self.plan[0] > self.plan_timeout:
            return None
        pts = to_body(pose, self.plan[1])
        if np.hypot(*pts[-1]) < self.hold_dist:
            return 'hold'
        for p in pts:
            if p[0] > 0 and np.hypot(*p) >= self.lookahead:
                return p
        p = pts[-1]
        return p if p[0] > self.hold_dist else None

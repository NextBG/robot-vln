"""ROS-free GPS route follower: GPS/odometry fusion + pure pursuit on a closed route.

Heading: the S1 odometry yaw (gyro-integrated) is in an arbitrary frame. The offset to
ENU is estimated by comparing GPS and odometry displacement over the same window
(works while turning too) and is tracked continuously to absorb gyro drift.
Before the first estimate the robot drives straight ahead slowly (state ALIGN).

Position: complementary filter, odometry increments rotated into ENU + GPS correction.
"""
import math
from collections import deque

import numpy as np

from .geo import LoopRoute, pure_pursuit, wrap_angle

WAIT_FIX, ALIGN, FOLLOW, HOLD, DONE, FAULT = 'WAIT_FIX', 'ALIGN', 'FOLLOW', 'HOLD', 'DONE', 'FAULT'


class FollowerParams:
    max_v = 0.6            # m/s cruise
    min_v = 0.15
    align_v = 0.3
    max_w = 0.8            # rad/s
    turn_in_place = math.radians(60)
    lookahead = 4.0        # m (GPS noise ~2.5 m -> keep >= 3 m)
    gps_gain = 0.25        # complementary filter weight per fix
    start_radius = 25.0    # must start within this distance of the route
    max_xte = 12.0         # cross-track error that triggers FAULT (stop)
    fix_timeout = 2.5      # s without a usable fix -> HOLD
    max_fix_std = 6.0      # m, reject fixes with larger horizontal std
    align_dist = 10.0      # m driven straight for the first heading estimate
    heading_baseline = 25.0  # m of displacement per later heading-offset update
    heading_gain = 0.1     # low-pass weight of later updates (gyro drift is slow)
    goal_tol = 3.0         # m before the end of the route -> DONE
    wp_radius = 3.0        # m, a waypoint counts as reached inside this radius (or once passed)
    search_back = 15.0
    search_fwd = 40.0


class RouteFollower:
    def __init__(self, route: LoopRoute, params=None, start_progress=None):
        self.route = route
        self.P = params or FollowerParams()
        self.state = WAIT_FIX
        self.reason = ''
        self.pos = None             # fused ENU position
        self.offset = None          # ENU yaw = odom yaw + offset
        self.s = None               # unwrapped arc length along the route
        self.s_start = None
        self.start_progress = start_progress   # resume: progress already done (m)
        self.xte = 0.0
        self.last_fix_t = -1e9
        self.odom = None            # (x, y, yaw) last odometry
        self.hist = deque(maxlen=200)   # (gps_xy, odom_xy) pairs
        self.n_offset_updates = 0

    # ------------------------------------------------------------- inputs
    def on_odom(self, x, y, yaw):
        if self.odom is not None and self.pos is not None and self.offset is not None:
            dx, dy = x - self.odom[0], y - self.odom[1]
            c, s = math.cos(self.offset), math.sin(self.offset)
            self.pos = self.pos + np.array([c * dx - s * dy, s * dx + c * dy])
        self.odom = (x, y, yaw)

    def on_fix(self, t, lat, lon, std=None):
        if std is not None and std > self.P.max_fix_std:
            return
        g = np.array(self.route.enu.to_enu(lat, lon))
        self.last_fix_t = t
        if self.pos is None or self.offset is None:
            self.pos = g
        else:
            self.pos = self.pos + self.P.gps_gain * (g - self.pos)
        if self.odom is not None:
            self._update_heading(g)

    def _update_heading(self, g):
        o = np.array(self.odom[:2])
        self.hist.append((g, o))
        base = self.P.align_dist if self.offset is None else self.P.heading_baseline
        for gi, oi in self.hist:
            dg, do = g - gi, o - oi
            ng, no = np.hypot(*dg), np.hypot(*do)
            if ng < base:
                break
            if no > 0.5 * base and 0.7 < ng / no < 1.4:
                est = math.atan2(dg[1], dg[0]) - math.atan2(do[1], do[0])
                if self.offset is None:
                    self.offset = wrap_angle(est)
                else:
                    self.offset = wrap_angle(self.offset + self.P.heading_gain * wrap_angle(est - self.offset))
                self.n_offset_updates += 1
                while self.hist and np.hypot(*(g - self.hist[0][0])) >= base:
                    self.hist.popleft()
                return

    # ------------------------------------------------------------- control
    @property
    def yaw(self):
        if self.odom is None or self.offset is None:
            return None
        return wrap_angle(self.odom[2] + self.offset)

    @property
    def progress(self):
        return 0.0 if self.s is None else self.s - self.s_start

    @property
    def end(self):
        """Unwrapped arc length where the route ends."""
        return self.s_start + self.route.length if self.route.closed else self.route.length

    def _vertex_after(self, s):
        """(index, unwrapped arc length) of the first route vertex strictly after arc length s."""
        r = self.route
        if not r.closed:
            k = int(np.searchsorted(r.s0, s, side='right'))
            return (k, float(r.s0[k])) if k < len(r.s0) else (len(r.p) - 1, r.length)
        base = math.floor(s / r.length) * r.length
        k = int(np.searchsorted(r.s0, s - base, side='right'))
        if k >= len(r.s0):
            return 0, base + r.length
        return k, base + r.s0[k]

    @property
    def first_waypoint(self):
        return None if self.s_start is None else self._vertex_after(self.s_start)[0]

    def next_waypoint(self):
        """Next waypoint (route vertex) not yet reached: ahead of the current progress and outside
        `wp_radius`. Returns (index, (e, n), distance_m); index -1 = route end (not a vertex)."""
        if self.s is None or self.pos is None:
            return None
        r, end = self.route, self.end
        k, sk = self._vertex_after(self.s)
        while sk < end - 1e-6:
            d = float(np.hypot(*(r.p[k] - self.pos)))
            if d > self.P.wp_radius:
                return k, r.p[k], d
            k, sk = self._vertex_after(sk)
        goal = r.point_at(end)
        k_goal = int(np.argmin(np.hypot(*(r.p - goal).T)))
        if np.hypot(*(r.p[k_goal] - goal)) > 1e-3:
            k_goal = -1
        return k_goal % r.n_wp if k_goal >= 0 else -1, goal, float(np.hypot(*(goal - self.pos)))

    def route_point_ahead(self, ahead):
        """ENU point `ahead` metres further along the route (clamped to the end), or None."""
        if self.s is None:
            return None
        return self.route.point_at(min(self.s + ahead, self.end))

    def step(self, t, local_target=None):
        """Returns (v, w) command. Call at >= 10 Hz.
        local_target: ENU point from a local planner (CityWalker) to steer at instead of the route
        lookahead point, or 'hold' to stand still; route progress / safety checks are unchanged."""
        P = self.P
        if self.state in (DONE, FAULT):
            return 0.0, 0.0
        if self.pos is None:
            self.state = WAIT_FIX
            return 0.0, 0.0
        if t - self.last_fix_t > P.fix_timeout:
            self.state, self.reason = HOLD, 'no GPS fix'
            return 0.0, 0.0

        if self.s is None:
            r = self.route
            if self.start_progress is not None:
                s, xte = r.project(self.pos, self.start_progress,
                                   back=3 * P.search_fwd, fwd=3 * P.search_fwd)
                if s is not None:
                    self.s_start = s - self.start_progress if r.closed else 0.0
            else:
                s, xte = r.project(self.pos)
                if s is not None:
                    if r.closed:
                        # starting at the finish point means "start of route", not "already done"
                        s = s if s < r.length - P.goal_tol else 0.0
                        self.s_start = s
                    else:
                        self.s_start = 0.0      # open route: join at the closest point
            if s is None or abs(xte) > P.start_radius:
                self.state = FAULT
                self.reason = f'not on route (distance {abs(xte or 1e9):.0f} m > {P.start_radius} m)'
                return 0.0, 0.0
            self.s = s

        end = self.end
        s, xte = self.route.project(self.pos, self.s, P.search_back, max(0.0, min(P.search_fwd, end - self.s)))
        if s is None or abs(xte) > P.max_xte:
            self.state = FAULT
            self.reason = f'off route (cross-track {abs(xte or 1e9):.1f} m)'
            return 0.0, 0.0
        self.s = max(self.s, s)   # progress is monotonic
        self.xte = xte

        if self.s >= end - P.goal_tol:
            self.state, self.reason = DONE, 'route completed'
            return 0.0, 0.0

        if self.offset is None:
            self.state = ALIGN
            return P.align_v, 0.0

        self.state = FOLLOW
        if isinstance(local_target, str):
            return 0.0, 0.0
        target = self.route.point_at(min(self.s + P.lookahead, end)) if local_target is None else local_target
        alpha, k = pure_pursuit(self.pos, self.yaw, target)
        if abs(alpha) > P.turn_in_place:
            return 0.0, math.copysign(P.max_w, alpha)
        remaining = end - self.s
        v = P.max_v * max(0.3, math.cos(alpha)) * min(1.0, remaining / 5.0)
        v = max(P.min_v, v)
        w = float(np.clip(v * k, -P.max_w, P.max_w))
        if abs(w) >= P.max_w:
            v = min(v, P.max_w / max(abs(k), 1e-6))
        return v, w

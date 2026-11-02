"""Geodesy + closed-loop route geometry (ROS-free, unit tested)."""
import csv
import math

import numpy as np

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalENU:
    """Local tangent-plane (east, north) metres around an origin; accurate to cm over a few km."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        phi = math.radians(lat0)
        s = 1.0 - WGS84_E2 * math.sin(phi) ** 2
        self.rn = WGS84_A * (1.0 - WGS84_E2) / s ** 1.5  # meridian radius
        self.re = WGS84_A / math.sqrt(s) * math.cos(phi)  # parallel radius

    def to_enu(self, lat, lon):
        return (math.radians(lon - self.lon0) * self.re, math.radians(lat - self.lat0) * self.rn)

    def to_lla(self, e, n):
        return (self.lat0 + math.degrees(n / self.rn), self.lon0 + math.degrees(e / self.re))


def load_route_csv(path):
    pts = []
    with open(path) as f:
        for row in csv.reader(line for line in f if not line.startswith('#')):
            if not row or row[0].strip() == 'lat':
                continue
            pts.append((float(row[0]), float(row[1])))
    if len(pts) < 2:
        raise ValueError(f'route {path} has < 2 points')
    return pts


def simplify_waypoints(latlon, tol=1.0, max_spacing=25.0):
    """Densely sampled route -> sparse waypoints joined by straight lines.

    Douglas-Peucker with `tol` metres (keeps every corner), then long straight legs are split so
    consecutive waypoints are at most `max_spacing` apart. First and last points are kept.
    """
    if len(latlon) < 3:
        return list(latlon)
    enu = LocalENU(*latlon[0])
    p = np.array([enu.to_enu(la, lo) for la, lo in latlon], dtype=float)
    keep = np.zeros(len(p), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(p) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, d = p[i], p[j] - p[i]
        n = np.hypot(*d)
        w = p[i + 1:j] - a
        dist = np.abs(d[0] * w[:, 1] - d[1] * w[:, 0]) / n if n > 1e-9 else np.hypot(w[:, 0], w[:, 1])
        k = int(np.argmax(dist))
        if dist[k] > tol:
            k += i + 1
            keep[k] = True
            stack += [(i, k), (k, j)]
    out = []
    idx = np.flatnonzero(keep)
    for i, j in zip(idx[:-1], idx[1:]):
        m = max(1, int(math.ceil(np.hypot(*(p[j] - p[i])) / max_spacing)))
        out += [p[i] + (p[j] - p[i]) * t / m for t in range(m)]
    out.append(p[idx[-1]])
    return [enu.to_lla(e, n) for e, n in out]


class LoopRoute:
    """Closed polyline in ENU with arc-length parametrisation (s wraps modulo `length`)."""

    def __init__(self, latlon, close_tol=30.0):
        self.enu = LocalENU(*latlon[0])
        p = np.array([self.enu.to_enu(la, lo) for la, lo in latlon], dtype=float)
        if np.hypot(*(p[-1] - p[0])) > close_tol:
            raise ValueError('route is not a closed loop (first/last point %.0f m apart)'
                             % np.hypot(*(p[-1] - p[0])))
        if np.hypot(*(p[-1] - p[0])) > 1e-6:
            p = np.vstack([p, p[:1]])
        keep = np.r_[True, np.hypot(*np.diff(p, axis=0).T) > 1e-3]
        self.p = p[keep]
        self.a = self.p[:-1]
        self.d = np.diff(self.p, axis=0)
        self.seg_len = np.hypot(self.d[:, 0], self.d[:, 1])
        self.s0 = np.r_[0.0, np.cumsum(self.seg_len)[:-1]]
        self.length = float(self.seg_len.sum())

    def point_at(self, s):
        s = s % self.length
        i = int(np.searchsorted(self.s0, s, side='right') - 1)
        t = (s - self.s0[i]) / self.seg_len[i]
        return self.a[i] + t * self.d[i]

    def tangent_at(self, s):
        i = int(np.searchsorted(self.s0, s % self.length, side='right') - 1)
        return math.atan2(self.d[i, 1], self.d[i, 0])

    def project(self, xy, s_hint=None, back=15.0, fwd=40.0):
        """Closest point on the route.

        With `s_hint`, only candidates whose arc length lies in [s_hint-back, s_hint+fwd]
        are considered (prevents jumping across the loop) and the returned s is *unwrapped*
        relative to s_hint. Returns (s, signed_cross_track_m) or (None, None) if no candidate.
        Cross-track is positive when the robot is left of the route direction.
        """
        q = np.asarray(xy, dtype=float)
        w = q - self.a
        t = np.clip((w * self.d).sum(1) / self.seg_len ** 2, 0.0, 1.0)
        foot = self.a + t[:, None] * self.d
        dist = np.hypot(*(q - foot).T)
        s = self.s0 + t * self.seg_len
        if s_hint is not None:
            delta = (s - s_hint + self.length / 2) % self.length - self.length / 2
            mask = (delta >= -back) & (delta <= fwd)
            if not mask.any():
                return None, None
            dist = np.where(mask, dist, np.inf)
        i = int(np.argmin(dist))
        if s_hint is None:
            # overlapping out-and-back legs: prefer the earliest part of the route
            i = int(np.flatnonzero(dist <= dist[i] + 1.0)[0])
        cross = self.d[i, 0] * w[i, 1] - self.d[i, 1] * w[i, 0]
        xte = math.copysign(dist[i], cross)
        if s_hint is None:
            return float(s[i]), xte
        return float(s_hint + delta[i]), xte


def pure_pursuit(pose_xy, yaw, target_xy):
    """Returns (alpha, curvature) for a target point; alpha = bearing in the robot frame."""
    dx, dy = target_xy[0] - pose_xy[0], target_xy[1] - pose_xy[1]
    ld = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    return alpha, (2.0 * math.sin(alpha) / ld if ld > 1e-6 else 0.0)

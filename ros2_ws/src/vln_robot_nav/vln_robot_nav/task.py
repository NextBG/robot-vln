"""Build a navigation task route (WGS-84 waypoints) from a task description.

Task dict (all coordinates WGS-84 [lat, lon]):
  mode     'route'  : part of an existing route file between `start` and `end`
           'manual' : straight lines through `start`, `points`..., `end`
           'amap'   : AMap walking directions from `start` to `end` (needs an AMap web-service key)
  start, end         optional for 'route' (default: start / end of the base route)
  points             'manual' only: intermediate waypoints
  reverse            'route' only: follow the base route backwards
  loop               return to the start (closed loop)
  simplify_tol, max_spacing   waypoint simplification (see geo.simplify_waypoints)

`build_task` returns {'points', 'closed', 'length_m', 'n', 'warnings'}.
"""
import math

import numpy as np

from .geo import LoopRoute, simplify_waypoints

SNAP_WARN_M = 30.0


def _near(route, ll, gap=5.0):
    """Places where the route passes `ll`: one (s, dist) per leg (candidates within `gap` m of arc
    length are neighbouring segments of the same leg), sorted by s."""
    s, dist = route.candidates(route.enu.to_enu(*ll))
    idx = np.flatnonzero(dist <= dist.min() + 1.0)
    idx = idx[np.argsort(s[idx])]
    groups = []
    for i in idx:
        if groups and s[i] - s[groups[-1][-1]] <= gap:
            groups[-1].append(i)
        else:
            groups.append([i])
    if route.closed and len(groups) > 1 and s[groups[0][0]] + route.length - s[groups[-1][-1]] <= gap:
        groups[0] += groups.pop()
    out = [(float(s[g[int(np.argmin(dist[g]))]]), float(dist[g].min())) for g in (np.array(g) for g in groups)]
    return sorted(out)


def _pick(route, ll, after=None):
    """Arc length on `route` closest to `ll`. Several legs pass it (out-and-back): the earliest one,
    or with `after` the first one ahead of that arc length. Returns (s, dist), s None if none ahead."""
    near = _near(route, ll)
    if after is None:
        return near[0]
    best = None
    for s, d in near:
        ahead = s - after
        if route.closed:
            ahead %= route.length
            if ahead < 1.0:
                ahead += route.length
        elif ahead < 1.0:
            continue
        if best is None or ahead < best[0]:
            best = (ahead, d)
    if best is None:
        return None, min(d for _, d in near)
    return after + best[0], best[1]


def _segment(route, sa, sb):
    """Points along `route` from arc length sa to sb (sb may exceed length on a loop)."""
    pts = [route.point_at(sa)]
    laps = [0.0, route.length] if route.closed else [0.0]
    for lap in laps:
        for k, sk in enumerate(route.s0):
            if sa < sk + lap < sb:
                pts.append(route.p[k])
    if not route.closed and sa < route.length < sb + 1e-9:
        pts.append(route.p[-1])
    end = route.point_at(sb)
    if np.hypot(*(np.asarray(pts[-1]) - end)) > 1e-3:
        pts.append(end)
    return [route.enu.to_lla(*q) for q in pts]


def _ll(v):
    if v is None:
        return None
    lat, lon = float(v[0]), float(v[1])
    if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError(f'bad coordinate {v}')
    return lat, lon


def route_part(base_latlon, start=None, end=None, reverse=False, loop=False):
    """Returns (latlon, closed, warnings) for the part of the base route between start and end."""
    pts = list(base_latlon)[::-1] if reverse else list(base_latlon)
    base = LoopRoute(pts, closed=None)
    warnings = []
    sa, d = 0.0, 0.0
    if start is not None:
        sa, d = _pick(base, start)
    if base.closed and (loop or end is None):
        if d > SNAP_WARN_M:
            warnings.append(f'起点离路线 {d:.0f} m')
        return _segment(base, sa, sa + base.length), True, warnings
    if end is None:
        sb = base.length
    else:
        # start on an out-and-back leg has several candidates: take the shortest forward start -> end
        starts = [(sa, d)]
        if start is not None:
            starts = _near(base, start)
        best = None
        for sa_c, d_c in starts:
            sb_c, de = _pick(base, end, after=sa_c)
            if sb_c is not None and (best is None or sb_c - sa_c < best[1] - best[0]):
                best = (sa_c, sb_c, d_c, de)
        if best is None:
            raise ValueError('终点在起点之前（沿路线方向），可勾选“反向”')
        sa, sb, d, de = best
        if de > SNAP_WARN_M:
            warnings.append(f'终点离路线 {de:.0f} m')
    if d > SNAP_WARN_M:
        warnings.append(f'起点离路线 {d:.0f} m')
    if sb - sa < 1.0:
        raise ValueError('起点和终点太近')
    part = _segment(base, sa, sb)
    if loop:
        return part + part[-2::-1], True, warnings
    return part, False, warnings


def build_task(task, base_latlon=None, walker=None):
    """walker(start_ll, end_ll) -> [(lat, lon)] for mode 'amap'."""
    mode = task.get('mode', 'route')
    start, end = _ll(task.get('start')), _ll(task.get('end'))
    loop = bool(task.get('loop'))
    warnings = []
    if mode == 'route':
        if not base_latlon:
            raise ValueError('没有基础路线')
        pts, closed, warnings = route_part(base_latlon, start, end, bool(task.get('reverse')), loop)
    elif mode == 'manual':
        pts = [p for p in [start] + [_ll(q) for q in task.get('points') or []] + [end] if p is not None]
        if len(pts) < 2:
            raise ValueError('至少需要 2 个路径点')
        closed = loop
        if loop:
            pts.append(pts[0])
    elif mode == 'amap':
        if start is None or end is None:
            raise ValueError('高德规划需要起点和终点')
        if walker is None:
            raise ValueError('未配置高德 Key')
        pts = list(walker(start, end))
        closed = loop
        if loop:
            pts += list(walker(end, start))[1:]
            pts.append(pts[0])
    else:
        raise ValueError(f'unknown mode {mode}')
    wps = simplify_waypoints(pts, float(task.get('simplify_tol', 1.0)), float(task.get('max_spacing', 25.0)))
    route = LoopRoute(wps, closed=closed)
    return {'points': [[round(la, 7), round(lo, 7)] for la, lo in wps], 'closed': closed,
            'length_m': round(route.length, 1), 'n': route.n_wp, 'warnings': warnings}

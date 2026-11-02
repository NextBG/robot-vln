import math
import os
import random

import numpy as np
import pytest

from vln_robot_nav.follower import DONE, FAULT, FollowerParams, RouteFollower
from vln_robot_nav.geo import LocalENU, LoopRoute, load_route_csv, pure_pursuit, simplify_waypoints, wrap_angle

ROUTE = os.path.join(os.path.dirname(__file__), '..', 'routes', 'dishui_lake.csv')


def square(side=50.0):
    enu = LocalENU(30.9, 121.9)
    pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    return [enu.to_lla(e, n) for e, n in pts]


def test_enu_roundtrip():
    enu = LocalENU(30.9, 121.94)
    lat, lon = enu.to_lla(1234.5, -678.9)
    e, n = enu.to_enu(lat, lon)
    assert abs(e - 1234.5) < 1e-6 and abs(n + 678.9) < 1e-6


def test_route_projection_and_wrap():
    r = LoopRoute(square())
    assert abs(r.length - 200.0) < 0.01
    s, xte = r.project((25.0, -2.0))
    assert abs(s - 25.0) < 1e-6 and abs(xte + 2.0) < 1e-6      # right of route -> negative
    s, _ = r.project((10.0, -0.5), s_hint=195.0)                  # wraps past the start
    assert abs(s - 210.0) < 1e-6
    assert np.allclose(r.point_at(210.0), (10.0, 0.0))


def test_pure_pursuit_sign():
    alpha, k = pure_pursuit((0, 0), 0.0, (1.0, 1.0))
    assert alpha > 0 and k > 0
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9 or abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9


def simulate(route, gps_std=2.0, max_t=None, seed=0, start_yaw_err=0.3, start_xy=None):
    rng = random.Random(seed)
    P = FollowerParams()
    f = RouteFollower(route, P)
    x, y = start_xy if start_xy is not None else route.point_at(0.0)
    yaw = route.tangent_at(0.0) + start_yaw_err
    odom_off, drift = rng.uniform(-math.pi, math.pi), math.radians(1.0) / 60
    ox = oy = 0.0
    dt, t, worst = 0.1, 0.0, 0.0
    max_t = max_t or 2.0 * route.length / P.min_v
    while t < max_t:
        oyaw = yaw - odom_off + drift * t
        f.on_odom(ox, oy, oyaw)
        if abs(t - round(t)) < 1e-6:
            lat, lon = route.enu.to_lla(x + rng.gauss(0, gps_std), y + rng.gauss(0, gps_std))
            f.on_fix(t, lat, lon, gps_std)
        v, w = f.step(t)
        if f.state in (DONE, FAULT):
            break
        yaw += w * dt
        x, y = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        # S1 odometry integrates wheel speed with its own (drifting) gyro yaw
        ox, oy = ox + v * math.cos(oyaw) * dt, oy + v * math.sin(oyaw) * dt
        if f.s is not None and f.progress > 10:
            _, true_xte = route.project((x, y), f.s, 30, 30)
            if true_xte is not None:
                worst = max(worst, abs(true_xte))
        t = round(t + dt, 6)
    return f, worst, (x, y)


def test_square_loop_completes():
    r = LoopRoute(square(60.0))
    f, worst, end = simulate(r, gps_std=1.0)
    assert f.state == DONE, f.reason
    assert worst < 4.0
    assert np.hypot(*(np.array(end) - r.point_at(0.0))) < 6.0


def test_refuses_far_start():
    r = LoopRoute(square(60.0))
    f, _, _ = simulate(r, start_xy=(30.0, 100.0), max_t=5)
    assert f.state == FAULT and 'not on route' in f.reason


@pytest.mark.skipif(not os.path.exists(ROUTE), reason='route file missing')
@pytest.mark.parametrize('seed', [1, 2])
def test_dishui_lake_full_loop(seed):
    dense = LoopRoute(load_route_csv(ROUTE))
    r = LoopRoute(simplify_waypoints(load_route_csv(ROUTE)))   # what gps_waypoint_follower uses
    assert 8000 < r.length < 11000 and abs(r.length - dense.length) < 0.01 * dense.length
    assert len(r.p) < len(dense.p) / 3
    f, worst, end = simulate(r, gps_std=2.5, seed=seed)
    assert f.state == DONE, f'{f.state} {f.reason} at {f.progress:.0f}/{r.length:.0f} m'
    assert worst < 6.0, worst
    assert np.hypot(*(np.array(end) - r.point_at(0.0))) < 8.0


def dense_square(side=60.0, step=1.0):
    enu = LocalENU(30.9, 121.9)
    corners = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    pts = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        n = int(round(math.hypot(bx - ax, by - ay) / step))
        pts += [(ax + (bx - ax) * t / n, ay + (by - ay) * t / n) for t in range(n)]
    pts.append(corners[-1])
    return [enu.to_lla(e, n) for e, n in pts]


def test_simplify_waypoints():
    wps = simplify_waypoints(dense_square(60.0), tol=0.5, max_spacing=25.0)
    r = LoopRoute(wps)
    assert abs(r.length - 240.0) < 0.05
    assert len(wps) == 4 * 3 + 1                    # 60 m sides split into 3 legs of 20 m
    assert r.seg_len.max() <= 25.0 + 1e-6
    for c in [(0, 0), (60, 0), (60, 60), (0, 60)]:   # every corner is kept
        assert np.min(np.hypot(*(r.p - np.array(c)).T)) < 0.05


def test_next_waypoint():
    r = LoopRoute(simplify_waypoints(dense_square(60.0), tol=0.5, max_spacing=25.0))
    f = RouteFollower(r)
    assert f.next_waypoint() is None
    f.pos, f.s, f.s_start = np.array([5.0, 0.5]), 5.0, 0.0
    k, xy, d = f.next_waypoint()
    assert k == 1 and np.allclose(xy, (20.0, 0.0)) and abs(d - math.hypot(15, 0.5)) < 1e-6
    assert f.first_waypoint == 1
    f.pos, f.s = np.array([18.0, 0.0]), 18.0       # inside wp_radius of #1 -> already the next one
    assert f.next_waypoint()[0] == 2
    f.pos, f.s = np.array([0.0, 2.0]), 238.0       # almost done -> route end (= vertex 0)
    k, xy, _ = f.next_waypoint()
    assert k == 0 and np.allclose(xy, (0.0, 0.0))

import math
import os
import sys

import numpy as np

from vln_robot_nav.citywalker import CityWalkerPlanner, from_body, to_body
from vln_robot_nav.follower import FOLLOW, RouteFollower
from vln_robot_nav.geo import LocalENU, LoopRoute

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'server'))


def test_body_roundtrip():
    pose = (3.0, -2.0, math.radians(70))
    pts = np.array([[1.0, 0.0], [2.0, -1.5]])
    assert np.allclose(to_body(pose, from_body(pose, pts)), pts)
    # 1 m ahead of a robot facing +y (odom) is (x, y+1)
    assert np.allclose(from_body((0, 0, math.pi / 2), [[1.0, 0.0]]), [[0.0, 1.0]])


def test_history_and_request():
    p = CityWalkerPlanner(period=1.0, step_scale=0.5)
    for i in range(6):           # robot drives along odom +x at 0.5 m/s, heading +x
        assert p.add_frame(float(i), b'jpg%d' % i, (0.5 * i, 0.0, 0.0))
        assert not p.add_frame(i + 0.3, b'x', (0.0, 0.0, 0.0))     # < 1 s apart: dropped
    assert p.ready()
    meta, jpegs, pose, t = p.build_request((12.5, 3.0))
    assert jpegs == [b'jpg1', b'jpg2', b'jpg3', b'jpg4', b'jpg5'] and t == 5.0
    assert np.allclose(meta['hist'], [[-2, 0], [-1.5, 0], [-1, 0], [-0.5, 0], [0, 0]])
    assert np.allclose(meta['target'], [10.0, 3.0]) and meta['step_scale'] == 0.5
    # a gap resets the history (no longer 1 Hz)
    p.add_frame(10.0, b'late', (0, 0, 0))
    assert not p.ready() and len(p.frames) == 1


def test_plan_tracking():
    p = CityWalkerPlanner(plan_timeout=3.0, lookahead=1.5)
    assert p.target(0.0, (0, 0, 0)) is None
    p.set_plan(10.0, (1.0, 1.0, math.pi / 2), [[0.5, 0], [1.0, -0.2], [1.6, -0.5], [2.2, -0.8], [2.8, -1.0]])
    # robot moved 0.5 m along its heading (odom +y) since the frame
    tgt = p.target(10.5, (1.0, 1.5, math.pi / 2))
    assert np.allclose(tgt, [1.7, -0.8])
    assert p.target(13.5, (1.0, 1.5, math.pi / 2)) is None          # stale -> GPS fallback
    p.set_plan(20.0, (0, 0, 0), [[0.05, 0], [0.1, 0], [0.1, 0], [0.12, 0], [0.1, 0.05]])
    assert p.target(20.2, (0, 0, 0)) == 'hold'


def test_follower_local_target():
    enu = LocalENU(30.9, 121.9)
    route = LoopRoute([enu.to_lla(e, 0.0) for e in (0.0, 100.0)], closed=False)
    f = RouteFollower(route)
    f.on_odom(0.0, 0.0, 0.0)
    f.on_fix(0.0, *enu.to_lla(10.0, 0.0))
    f.offset = 0.0                                   # heading known: odom +x = east
    v, w = f.step(0.1)
    assert f.state == FOLLOW and abs(w) < 1e-6 and v > 0
    v, w = f.step(0.2, local_target=np.array([12.0, 2.0]))   # planner steers left of the route
    assert w > 0
    assert f.step(0.3, local_target='hold') == (0.0, 0.0) and f.state == FOLLOW
    assert np.allclose(f.route_point_ahead(12.0), (22.0, 0.0), atol=1e-6)


def test_server_frame_conversion():
    try:
        import citywalker_server as cs      # needs torch/flask/cv2: only where the server runs
    except ImportError:
        return
    body = np.array([[2.0, 1.0]])         # 2 m ahead, 1 m left
    assert np.allclose(cs.body_to_model(body), [[-1.0, 2.0]])   # model: x right, z forward
    assert np.allclose(cs.model_to_body(cs.body_to_model(body)), body)

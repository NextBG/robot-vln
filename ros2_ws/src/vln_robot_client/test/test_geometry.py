import math

import numpy as np

from vln_robot_client.geometry import (body_traj_to_world, discrete_actions_to_goal,
                                       goal_controller, project_points_to_depth, pure_pursuit,
                                       quaternion_to_matrix, wrap_angle, yaw_from_quaternion)


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap_angle(0.5) - 0.5) < 1e-9


def test_yaw_and_matrix_consistent():
    yaw = 0.7
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert abs(yaw_from_quaternion(*q) - yaw) < 1e-9
    R = quaternion_to_matrix(*q)
    assert np.allclose(R @ [1, 0, 0], [math.cos(yaw), math.sin(yaw), 0])


def test_discrete_actions():
    goal, stop = discrete_actions_to_goal((0, 0, 0), [1, 1, 2, 2, 2, 2, 2, 2, 1])
    assert not stop
    assert np.allclose(goal[:2], (0.5, 0.25), atol=1e-9)
    assert abs(goal[2] - math.pi / 2) < 1e-9
    _, stop = discrete_actions_to_goal((0, 0, 0), [1, 0, 1])
    assert stop


def test_body_traj_to_world():
    w = body_traj_to_world((1.0, 2.0, math.pi / 2), [[0, 0], [1, 0], [1, 1]], skip=1)
    assert np.allclose(w, [[1.0, 3.0], [0.0, 3.0]])


def test_pure_pursuit_and_goal_controller():
    path = np.array([[0.5, 0.0], [1.0, 0.0]])
    v, w, done = pure_pursuit((0, 0, 0), path)
    assert v > 0 and abs(w) < 1e-6 and not done
    _, _, done = pure_pursuit((1.0, 0.0, 0), path)
    assert done
    v, w, done = goal_controller((0, 0, 0), (0, 0, math.pi / 2))
    assert v == 0 and w > 0 and not done


def test_projection_zbuffer():
    pts = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 1.0], [10.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
    d = project_points_to_depth(pts, 100, 100, 50, 40, 100, 80)
    assert d[40, 50] == 1.0
    assert np.count_nonzero(d) == 1

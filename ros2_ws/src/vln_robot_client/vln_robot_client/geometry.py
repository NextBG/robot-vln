"""Pure (ROS-free) helpers so they can be unit-tested anywhere."""
import math

import numpy as np

# VLN-CE discrete action space (R2R-CE / RxR-CE)
ACTION_STOP = 0
ACTION_FORWARD = 1
ACTION_LEFT = 2
ACTION_RIGHT = 3
ACTION_LOOK_DOWN = 5


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def discrete_actions_to_goal(pose, actions, step_m=0.25, turn_rad=math.radians(15.0)):
    """Integrate VLN-CE discrete actions from pose=(x, y, yaw) into a goal pose.

    Returns (goal_pose, stop) where stop is True if a STOP action was found.
    """
    x, y, yaw = pose
    for a in actions:
        if a == ACTION_STOP:
            return (x, y, yaw), True
        if a == ACTION_FORWARD:
            x += step_m * math.cos(yaw)
            y += step_m * math.sin(yaw)
        elif a == ACTION_LEFT:
            yaw = wrap_angle(yaw + turn_rad)
        elif a == ACTION_RIGHT:
            yaw = wrap_angle(yaw - turn_rad)
    return (x, y, yaw), False


def body_traj_to_world(pose, traj, skip=0):
    """Transform trajectory points [[x_fwd, y_left, ...], ...] in the body frame at `pose` to world."""
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for p in traj[skip:]:
        out.append((x0 + c * p[0] - s * p[1], y0 + s * p[0] + c * p[1]))
    return np.asarray(out, dtype=float).reshape(-1, 2)


def pure_pursuit(pose, path, lookahead=0.4, max_v=0.4, max_w=1.0, goal_tol=0.1):
    """Return (v, w, done) to follow a world-frame polyline with a differential-drive style command."""
    x, y, yaw = pose
    if len(path) == 0:
        return 0.0, 0.0, True
    d = np.hypot(path[:, 0] - x, path[:, 1] - y)
    if d[-1] < goal_tol:
        return 0.0, 0.0, True
    nearest = int(np.argmin(d))
    target = path[-1]
    for i in range(nearest, len(path)):
        if d[i] >= lookahead:
            target = path[i]
            break
    dx, dy = target[0] - x, target[1] - y
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    dist = math.hypot(dx, dy)
    if abs(alpha) > math.radians(60):
        return 0.0, float(np.clip(2.0 * alpha, -max_w, max_w)), False
    curvature = 2.0 * math.sin(alpha) / max(dist, 1e-3)
    v = min(max_v, dist)
    w = float(np.clip(v * curvature, -max_w, max_w))
    return float(v), w, False


def goal_controller(pose, goal, max_v=0.3, max_w=0.8, pos_tol=0.05, yaw_tol=math.radians(4)):
    """Rotate-then-drive controller toward goal=(x, y, yaw). Returns (v, w, done)."""
    x, y, yaw = pose
    gx, gy, gyaw = goal
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist > pos_tol:
        heading = wrap_angle(math.atan2(dy, dx) - yaw)
        if abs(heading) > math.radians(20):
            return 0.0, float(np.clip(1.5 * heading, -max_w, max_w)), False
        return float(min(max_v, 1.2 * dist)), float(np.clip(1.5 * heading, -max_w, max_w)), False
    err = wrap_angle(gyaw - yaw)
    if abs(err) > yaw_tol:
        return 0.0, float(np.clip(1.5 * err, -max_w, max_w)), False
    return 0.0, 0.0, True


def project_points_to_depth(points_cam, fx, fy, cx, cy, width, height, min_z=0.1, max_z=40.0):
    """Z-buffer project Nx3 points (camera optical frame: z forward) into a float32 depth image (0 = no data)."""
    depth = np.zeros((height, width), dtype=np.float32)
    if points_cam.size == 0:
        return depth
    z = points_cam[:, 2]
    m = (z > min_z) & (z < max_z)
    p = points_cam[m]
    z = z[m]
    u = np.round(fx * p[:, 0] / z + cx).astype(np.int64)
    v = np.round(fy * p[:, 1] / z + cy).astype(np.int64)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, z = u[inside], v[inside], z[inside]
    # Write far points first so that nearer points overwrite them (z-buffer).
    order = np.argsort(-z)
    depth[v[order], u[order]] = z[order]
    return depth

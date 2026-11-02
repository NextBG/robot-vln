"""GPS route follower node (RoboMaster S1 + WHEELTEC G60).

  /fix   sensor_msgs/NavSatFix   (nmea_navsat_driver)
  /odom  nav_msgs/Odometry       (robomaster_ros, gyro yaw)
  -> /nav/cmd_vel geometry_msgs/Twist  (goes through safety_stop -> /cmd_vel)
  -> /nav/status  std_msgs/String (JSON, 2 Hz)

Never moves on its own: call  ros2 service call /nav/start std_srvs/srv/Trigger
Pause:                         ros2 service call /nav/stop  std_srvs/srv/Trigger
Progress is saved to `progress_file`; restart with resume:=true after a battery swap
(only applied when the route file content is unchanged).
Velocity commands are published only while navigating (plus one zero on pause/stop), so other
command sources (web teleop, VLN) can share the same topic.
`closed`: auto (loop when first/last point are < 30 m apart) | true | false (open route start -> end).

`planner`: gps (pure pursuit on the route polyline) | citywalker (learned local planner on the GPU
server: camera frames at 1 Hz + a route point `cw_target_ahead` m ahead -> 5 local waypoints that
are tracked instead of the route; falls back to the route when the server does not answer).
"""
import hashlib
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .citywalker import CityWalkerPlanner
from .follower import DONE, FAULT, FOLLOW, FollowerParams, RouteFollower
from .geo import LoopRoute, load_route_csv, simplify_waypoints


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        p = self.declare_parameter
        self.route_file = p('route_file', '').value
        self.progress_file = os.path.expanduser(p('progress_file', '~/.ros/gps_route_progress.json').value)
        resume = p('resume', False).value
        self.enabled = p('autostart', False).value
        params = FollowerParams()
        for k in [k for k in dir(FollowerParams) if not k.startswith('_')]:
            setattr(params, k, float(p(k, float(getattr(FollowerParams, k))).value))

        # dynamic: `-p closed:=true` arrives as a bool, `auto` as a string
        closed = p('closed', 'auto', ParameterDescriptor(dynamic_typing=True)).value
        closed = {'auto': None, 'true': True, 'false': False}[str(closed).lower()]
        self.task_name = str(p('task_name', '', ParameterDescriptor(dynamic_typing=True)).value)
        with open(self.route_file, 'rb') as fh:
            self.route_id = hashlib.sha1(fh.read()).hexdigest()[:12]
        route = LoopRoute(simplify_waypoints(load_route_csv(self.route_file),
                                             p('wp_simplify_tol', 1.0).value,
                                             p('wp_max_spacing', 25.0).value), closed=closed)
        start_progress = None
        if resume and os.path.exists(self.progress_file):
            with open(self.progress_file) as f:
                saved = json.load(f)
            if saved.get('route_id') == self.route_id and saved.get('state') != DONE:
                start_progress = float(saved['progress'])
        self.f = RouteFollower(route, params, start_progress)
        self.get_logger().info(
            f'route {self.route_file}: {route.length:.0f} m, {route.n_wp} waypoints, '
            f'{"closed" if route.closed else "open"}'
            + (f', resuming at {start_progress:.0f} m' if start_progress is not None else ''))

        self.create_subscription(NavSatFix, p('fix_topic', '/fix').value, self.on_fix, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value, self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic', '/nav/cmd_vel').value, 1)
        self.status_pub = self.create_publisher(String, '/nav/status', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.wp_pub = self.create_publisher(String, '/nav/waypoints', latched)
        self.wp_pub.publish(String(data=json.dumps({
            'route_file': self.route_file, 'radius': params.wp_radius, 'closed': route.closed,
            'points': [[round(la, 7), round(lo, 7)] for la, lo in (route.enu.to_lla(*q) for q in route.p)]})))
        self.create_service(Trigger, '/nav/start', self.srv_start)
        self.create_service(Trigger, '/nav/stop', self.srv_stop)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.last_save = 0.0
        self.last_state = None
        self.zero_pending = False

        self.planner = str(p('planner', 'gps').value).lower()
        self.cw = None
        if self.planner == 'citywalker':
            self.cw = CityWalkerPlanner(
                period=p('cw_period', 1.0).value, plan_timeout=p('cw_plan_timeout', 3.0).value,
                lookahead=p('cw_lookahead', 1.5).value,
                step_scale=float(p('cw_step_scale', 0.0).value) or 0.85 * params.max_v)
            self.cw_url = p('cw_server_url', 'http://127.0.0.1:5802/citywalker').value
            self.cw_ahead = p('cw_target_ahead', 12.0).value
            self.cw_timeout = p('cw_http_timeout', 2.5).value
            self.cw_max_hold = p('cw_max_hold', 10.0).value
            self.cw_lock = threading.Lock()
            self.cw_stat = {'ok': None, 'err': '', 'n': 0, 'n_err': 0, 'rtt_ms': None, 'infer_ms': None,
                            'arrive': None, 'active': False, 'frames': 0}
            self.cw_hold_since = None
            self.create_subscription(Image, p('image_topic', '/camera/image_raw').value, self.on_image,
                                     qos_profile_sensor_data)
            threading.Thread(target=self.cw_loop, daemon=True).start()
            self.get_logger().info(f'planner citywalker: {self.cw_url}, step_scale {self.cw.step_scale:.2f} m')

    def now(self):
        return time.monotonic()

    def on_fix(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX or math.isnan(msg.latitude):
            return
        std = None
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            std = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4]))
        self.f.on_fix(self.now(), msg.latitude, msg.longitude, std)

    def on_odom(self, msg):
        pp = msg.pose.pose
        self.f.on_odom(pp.position.x, pp.position.y, yaw_from_quat(pp.orientation))

    # ------------------------------------------------------------- CityWalker
    def on_image(self, msg):
        t = self.now()
        if self.f.odom is None or not self.cw.want_frame(t):
            return
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.cw_stat['err'] = f'unsupported image encoding {msg.encoding}'
            return
        import cv2
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            img = img[..., ::-1]
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            with self.cw_lock:
                self.cw.add_frame(t, jpg.tobytes(), self.f.odom)
                self.cw_stat['frames'] = len(self.cw.frames)

    def cw_loop(self):
        import requests
        sess = requests.Session()
        sess.trust_env = False      # never route the localhost tunnel through an http proxy
        while rclpy.ok():
            t0 = self.now()
            try:
                self.cw_request(sess)
            except Exception as e:  # noqa: BLE001
                self.cw_stat.update(ok=False, err=str(e)[:160], n_err=self.cw_stat['n_err'] + 1)
            time.sleep(max(0.05, self.cw.period - (self.now() - t0)))

    def cw_request(self, sess):
        f = self.f
        if not (self.enabled and f.state == FOLLOW and f.yaw is not None and f.odom is not None):
            return
        tgt = f.route_point_ahead(self.cw_ahead)
        if tgt is None:
            return
        # route target (ENU) -> odometry frame through the current fused pose
        ox, oy, oyaw = f.odom
        d = np.asarray(tgt) - f.pos
        a = oyaw - f.yaw
        tgt_odom = (ox + math.cos(a) * d[0] - math.sin(a) * d[1], oy + math.sin(a) * d[0] + math.cos(a) * d[1])
        with self.cw_lock:
            req = self.cw.build_request(tgt_odom)
        if req is None:
            return
        meta, jpegs, pose, t_frame = req
        files = {f'image{i}': (f'{i}.jpg', j, 'image/jpeg') for i, j in enumerate(jpegs)}
        t0 = self.now()
        r = sess.post(self.cw_url, files=files, data={'json': json.dumps(meta)}, timeout=self.cw_timeout)
        r.raise_for_status()
        reply = r.json()
        with self.cw_lock:
            self.cw.set_plan(t_frame, pose, reply['waypoints'], reply.get('arrive_prob', 0.0))
        self.cw_stat.update(ok=True, err='', n=self.cw_stat['n'] + 1, rtt_ms=round((self.now() - t0) * 1000),
                            infer_ms=reply.get('infer_ms'), arrive=reply.get('arrive_prob'),
                            target=meta['target'])

    def cw_local_target(self):
        """ENU steering point / 'hold' / None (GPS pure pursuit) from the latest CityWalker plan."""
        f = self.f
        if f.yaw is None or f.pos is None or f.odom is None:
            return None
        t = self.now()
        with self.cw_lock:
            b = self.cw.target(t, f.odom)
        if isinstance(b, str):
            self.cw_hold_since = self.cw_hold_since or t
            if t - self.cw_hold_since > self.cw_max_hold:
                b = None            # the model keeps standing still: let the route take over
        else:
            self.cw_hold_since = None
        self.cw_stat['active'] = b is not None
        if b is None or isinstance(b, str):
            return b
        c, s = math.cos(f.yaw), math.sin(f.yaw)
        return f.pos + np.array([c * b[0] - s * b[1], s * b[0] + c * b[1]])

    def srv_start(self, req, res):
        if self.f.state in (DONE, FAULT):
            res.success, res.message = False, f'{self.f.state}: {self.f.reason} (restart the node)'
        else:
            self.enabled = True
            res.success, res.message = True, 'started'
            self.get_logger().info('navigation started')
        return res

    def srv_stop(self, req, res):
        self.enabled = False
        self.zero_pending = True
        res.success, res.message = True, 'paused'
        self.get_logger().info('navigation paused')
        return res

    def tick(self):
        local = self.cw_local_target() if self.cw is not None and self.enabled else None
        v, w = self.f.step(self.now(), local)
        if self.f.state != self.last_state:
            self.get_logger().info(f'state {self.last_state} -> {self.f.state} {self.f.reason}')
            self.last_state = self.f.state
        if self.enabled and self.f.state in (DONE, FAULT):
            self.enabled, self.zero_pending = False, True
        cmd = Twist()
        if self.enabled:
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.cmd_pub.publish(cmd)
        elif self.zero_pending:
            self.cmd_pub.publish(cmd)
            self.zero_pending = False
        if self.f.s is not None and self.now() - self.last_save > 5.0:
            self.save_progress()

    def save_progress(self):
        self.last_save = self.now()
        os.makedirs(os.path.dirname(self.progress_file), exist_ok=True)
        tmp = self.progress_file + '.tmp'
        with open(tmp, 'w') as fh:
            json.dump({'route_file': self.route_file, 'route_id': self.route_id, 'progress': self.f.progress,
                       'state': self.f.state, 'time': time.time()}, fh)
        os.replace(tmp, self.progress_file)

    def publish_status(self):
        f = self.f
        lla = f.route.enu.to_lla(*f.pos) if f.pos is not None else None
        st = {'state': f.state, 'enabled': self.enabled, 'reason': f.reason, 'task': self.task_name,
              'closed': f.route.closed, 'route_id': self.route_id,
              'progress_m': round(f.progress, 1), 'route_m': round(f.route.length, 1),
              'xte_m': round(f.xte, 2),
              'yaw_deg': None if f.yaw is None else round(math.degrees(f.yaw), 1),
              'heading_updates': f.n_offset_updates,
              'lat': None if lla is None else round(lla[0], 7),
              'lon': None if lla is None else round(lla[1], 7)}
        wp = f.next_waypoint()
        if wp is not None:
            wlat, wlon = f.route.enu.to_lla(*wp[1])
            st.update({'wp_idx': wp[0], 'wp_first': f.first_waypoint, 'wp_n': f.route.n_wp,
                       'wp_lat': round(wlat, 7), 'wp_lon': round(wlon, 7),
                       'wp_dist_m': round(wp[2], 1), 'wp_radius_m': f.P.wp_radius})
        st['planner'] = self.planner
        if self.cw is not None:
            cw = dict(self.cw_stat)
            if f.odom is not None:
                with self.cw_lock:
                    age = self.cw.plan_age(self.now())
                    path = self.cw.path_body(f.odom)
                cw['age_s'] = None if age is None else round(age, 1)
                cw['path'] = None if path is None or age > self.cw.plan_timeout else np.round(path, 2).tolist()
            st['cw'] = cw
        self.status_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = GpsWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        if node.f.s is not None:
            node.save_progress()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Robot monitor / remote-control web dashboard (stdlib HTTP server, no extra deps).

    ros2 run vln_robot_web web_node --ros-args -p port:=8080

Open http://<robot-ip>:8080/  (no auth by default; -p require_token:=true enables a token,
printed at start-up and stored in ~/.config/vln_robot_web/token, for exposed/tunnelled use).

Data paths (browser <- robot):
    /api/events      Server-Sent Events: state JSON every 0.5 s (+ LiDAR top view if lidar=1)
    /stream.mjpg     MJPEG camera stream (?fps=5&q=60&w=640)
    /api/track       GPS track so far
    /api/route       route waypoints {"radius": m, "points": [[lat, lon], ...], "source": nav|file} (WGS-84)
Control (browser -> robot):
    POST /api/cmd    {"vx","vy","wz"}  -> cmd_topic (default /vln/cmd_vel -> safety_stop -> /cmd_vel)
                     dead-man: no command for `deadman` s -> zero
    POST /api/estop  {"on": true|false} -> /safety/estop (latched, honoured by safety_stop)
    POST /api/module {"name": chassis|camera|lidar|gnss|tf|vln|tunnel|all, "action": restart|start|stop}
                     (only with drivers:=true: every driver runs as its own restartable process)
    POST /api/vln    {"instruction": "..."} -> /vln/instruction (starts the VLN client if needed)
    POST /api/vln/stop                      -> /vln/stop
"""
import json
import math
import os
import secrets
import subprocess
import threading
import time
import urllib.request
from collections import deque
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .supervisor import Supervisor

MONITORED = {
    '/camera/image_raw': 'camera', '/livox/lidar': 'lidar', '/livox/imu': 'lidar_imu',
    '/odom': 'odom', '/imu': 'chassis_imu', '/battery': 'battery', '/fix': 'gps',
    '/safety/status': 'safety', '/vln/cmd_vel': 'vln_cmd', '/cmd_vel': 'wheel_cmd', '/vln/state': 'vln',
    '/nav/status': 'nav',
}
DRIVERS = [('chassis', 'S1 底盘'), ('camera', '相机'), ('lidar', '激光雷达'), ('gnss', 'GPS'), ('tf', '静态 TF')]


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class RateMeter:
    def __init__(self, window=3.0):
        self.window = window
        self.stamps = deque(maxlen=2000)

    def tick(self):
        self.stamps.append(time.time())

    def stats(self):
        now = time.time()
        while self.stamps and now - self.stamps[0] > self.window:
            self.stamps.popleft()
        age = now - self.stamps[-1] if self.stamps else None
        n = len(self.stamps)
        span = self.stamps[-1] - self.stamps[0] if n > 1 else 0.0
        rate = (n - 1) / span if span > 0 else 0.0
        return round(rate, 1), (round(age, 2) if age is not None else None)


class WebNode(Node):
    def __init__(self):
        super().__init__('vln_robot_web')
        p = self.declare_parameter
        self.port = p('port', 8080).value
        self.host = p('host', '0.0.0.0').value
        self.max_v = p('max_v', 0.3).value
        self.max_w = p('max_w', 0.8).value
        self.deadman = p('deadman', 0.4).value
        self.base_frame = p('base_frame', 'base_link').value
        self.lidar_range = p('lidar_range', 8.0).value
        self.lidar_grid = p('lidar_grid', 0.08).value
        self.track_min_step = p('track_min_step', 1.0).value
        token = p('token', '').value
        route_file = p('route_file', '').value
        cmd_topic = p('cmd_topic', '/vln/cmd_vel').value
        drivers = p('drivers', False).value
        driver_list = [d.strip() for d in p('driver_modules', 'chassis,camera,lidar,gnss,tf').value.split(',')]
        self.vln_server_url = p('vln_server_url', 'http://127.0.0.1:5801/eval_dual').value
        vln_tunnel = p('vln_tunnel', '').value
        vln_use_depth = p('vln_use_depth', True).value

        self.require_token = p('require_token', False).value
        self.token = (token or self._load_token()) if self.require_token else ''
        self.route_file = route_file or self._default_route()
        try:
            self.static_dir = os.path.join(get_package_share_directory('vln_robot_web'), 'static')
        except PackageNotFoundError:
            self.static_dir = os.path.join(os.path.dirname(__file__), '..', 'static')

        self.rates = {name: RateMeter() for name in MONITORED.values()}
        self.img_msg = None
        self.jpeg_cache = {}
        self.jpeg_lock = threading.Lock()
        self.cloud_msg = None
        self.lidar_xy = []
        self.lidar_seq = 0
        self.fix = None
        self.track = deque(maxlen=20000)
        self.odom = None
        self.imu = None
        self.battery = None
        self.safety = None
        self.wheel_cmd = None
        self.sys = {}
        self.web_cmd = (0.0, 0.0, 0.0)
        self.web_cmd_time = 0.0
        self.web_cmd_active = False
        self.estop = False
        self.n_clients = 0
        self.s1_connected = None
        self.vln = None
        self.vln_pending = None
        self.server = {'ok': None}
        self.nav = None
        self.route = None       # {'radius', 'points': [[lat, lon], ...], 'source'}
        self.route_ver = 0
        self.wp_radius = p('wp_radius', 3.0).value
        self.load_route_file()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_tf = None

        sd = qos_profile_sensor_data
        self._sub(Image, '/camera/image_raw', self.on_image, sd)
        self._sub(PointCloud2, '/livox/lidar', self.on_cloud, sd)
        self._sub(Imu, '/livox/imu', None, sd)
        self._sub(Odometry, '/odom', self.on_odom, 10)
        self._sub(Imu, '/imu', self.on_imu, sd)
        self._sub(BatteryState, '/battery', self.on_battery, 10)
        self._sub(NavSatFix, '/fix', self.on_fix, 10)
        self._sub(String, '/safety/status', self.on_safety, 10)
        self._sub(Twist, '/vln/cmd_vel', None, 10)
        self._sub(Twist, '/cmd_vel', self.on_wheel_cmd, 10)
        self._sub(String, '/vln/state', self.on_vln, 10)
        self._sub(String, '/nav/status', self.on_nav, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/connected', self.on_connected, latched)
        self.create_subscription(String, '/nav/waypoints', self.on_waypoints, latched)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 1)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', latched)
        self.estop_pub.publish(Bool(data=False))
        self.instr_pub = self.create_publisher(String, '/vln/instruction', 1)
        self.vln_stop_pub = self.create_publisher(Empty, '/vln/stop', 1)

        self.sup = Supervisor(self.get_logger(), os.path.expanduser('~/.ros/vln_robot_web'))
        if drivers:
            bringup = ['ros2', 'launch', 'vln_robot_bringup', 'robot.launch.py']
            for name, label in DRIVERS:
                if name in driver_list:
                    self.sup.add(name, label, bringup + [f'use_{m}:={"true" if m == name else "false"}'
                                                         for m, _ in DRIVERS])
        if vln_tunnel:
            self.sup.add('tunnel', 'GPU 隧道', vln_tunnel, autostart=True)
        self.sup.add('vln', 'VLN 客户端',
                     ['ros2', 'launch', 'vln_robot_client', 'vln.launch.py', 'with_safety:=false',
                      f'server_url:={self.vln_server_url}', f'use_depth:={str(vln_use_depth).lower()}'],
                     autostart=False)

        # GPS navigation task: gps_waypoint_follower runs as a supervised module, its command line is
        # rebuilt for every task (route file + parameters); it always starts paused and resumes progress
        self.nav_cmd_topic = cmd_topic
        self.nav_dir = os.path.expanduser('~/.ros/vln_robot_web')
        self.nav_task_file = os.path.join(self.nav_dir, 'nav_task.json')
        self.nav_csv = os.path.join(self.nav_dir, 'nav_task.csv')
        self.nav_progress = os.path.join(self.nav_dir, 'nav_progress.json')
        self.nav_task = None        # {'req', 'result', 'route_id', 't'}
        self.nav_pending = None     # start request waiting for the follower: {'t', 'route_id', 'since', 'future'}
        self.nav_msg = ''
        self.nav_rx = 0.0
        self.nav_start_cli = self.create_client(Trigger, '/nav/start')
        self.nav_stop_cli = self.create_client(Trigger, '/nav/stop')
        self.sup.add('nav', 'GPS 导航', ['true'], autostart=False)
        self.load_nav_task()

        self.create_timer(0.05, self.cmd_tick)
        self.create_timer(0.5, self.lidar_tick)
        self.create_timer(0.5, self.vln_tick)
        self.create_timer(0.5, self.nav_tick)
        threading.Thread(target=self.sys_loop, daemon=True).start()
        threading.Thread(target=self.health_loop, daemon=True).start()

        self.httpd = ThreadingHTTPServer((self.host, self.port), make_handler(self))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'dashboard: http://<robot-ip>:{self.port}/'
                               + (f'?token={self.token}' if self.require_token else ''))

    # ------------------------------------------------------------------ setup helpers
    def _sub(self, typ, topic, cb, qos):
        meter = self.rates[MONITORED[topic]]

        def wrapped(msg):
            meter.tick()
            if cb:
                cb(msg)
        self.create_subscription(typ, topic, wrapped, qos)

    @staticmethod
    def _load_token():
        path = os.path.expanduser('~/.config/vln_robot_web/token')
        if os.path.exists(path):
            with open(path) as f:
                tok = f.read().strip()
            if tok:
                return tok
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tok = secrets.token_urlsafe(12)
        with open(path, 'w') as f:
            f.write(tok + '\n')
        os.chmod(path, 0o600)
        return tok

    @staticmethod
    def _default_route():
        try:
            path = os.path.join(get_package_share_directory('vln_robot_nav'), 'routes', 'dishui_lake.csv')
            return path if os.path.exists(path) else ''
        except PackageNotFoundError:
            return ''

    def load_route_file(self):
        """Waypoints from the route file, simplified exactly like gps_waypoint_follower does.
        Replaced by /nav/waypoints as soon as the follower runs."""
        path = self.route_file
        if not path or not os.path.exists(path):
            return
        try:
            if path.endswith('.csv'):
                from vln_robot_nav.geo import load_route_csv, simplify_waypoints
                pts = simplify_waypoints(load_route_csv(path))
            else:
                with open(path) as f:
                    g = json.load(f)
                geom = g.get('geometry') or g['features'][0]['geometry']
                pts = [(lat, lon) for lon, lat in geom['coordinates']]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'cannot load route {path}: {e}')
            return
        self.route = {'radius': self.wp_radius, 'source': 'file',
                      'points': [[round(la, 7), round(lo, 7)] for la, lo in pts]}
        self.route_ver += 1

    # ------------------------------------------------------------------ ROS callbacks
    def on_image(self, msg):
        self.img_msg = msg

    def on_cloud(self, msg):
        self.cloud_msg = msg

    def on_odom(self, msg):
        p, t = msg.pose.pose, msg.twist.twist
        self.odom = {'x': round(p.position.x, 3), 'y': round(p.position.y, 3),
                     'yaw': round(math.degrees(_yaw(p.orientation)), 1),
                     'vx': round(t.linear.x, 3), 'vy': round(t.linear.y, 3), 'wz': round(t.angular.z, 3)}

    def on_imu(self, msg):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.imu = {'roll': round(math.degrees(roll), 1), 'pitch': round(math.degrees(pitch), 1),
                    'yaw': round(math.degrees(_yaw(q)), 1)}

    def on_battery(self, msg):
        self.battery = {'pct': round(msg.percentage * 100) if msg.percentage >= 0 else None,
                        'v': round(msg.voltage, 2), 'a': round(msg.current, 2),
                        'temp': round(msg.temperature, 1)}

    def on_fix(self, msg):
        ok = msg.status.status >= 0 and not math.isnan(msg.latitude)
        acc = math.sqrt(msg.position_covariance[0]) if msg.position_covariance[0] > 0 else None
        self.fix = {'ok': ok, 'status': msg.status.status, 'lat': msg.latitude if ok else None,
                    'lon': msg.longitude if ok else None, 'alt': round(msg.altitude, 1) if ok else None,
                    'acc': round(acc, 1) if acc else None}
        if ok:
            cur = (round(msg.latitude, 7), round(msg.longitude, 7))
            if not self.track or self._dist_m(self.track[-1], cur) >= self.track_min_step:
                self.track.append(cur)

    @staticmethod
    def _dist_m(a, b):
        dy = (a[0] - b[0]) * 111320.0
        dx = (a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0]))
        return math.hypot(dx, dy)

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except ValueError:
            pass

    def on_wheel_cmd(self, msg):
        self.wheel_cmd = [round(msg.linear.x, 3), round(msg.linear.y, 3), round(msg.angular.z, 3)]

    def on_connected(self, msg):
        self.s1_connected = msg.data

    def on_vln(self, msg):
        try:
            self.vln = json.loads(msg.data)
        except ValueError:
            pass

    def on_nav(self, msg):
        try:
            self.nav = json.loads(msg.data)
            self.nav_rx = time.time()
        except ValueError:
            pass

    def on_waypoints(self, msg):
        try:
            d = json.loads(msg.data)
            self.route = {'radius': float(d.get('radius', self.wp_radius)), 'source': 'nav',
                          'points': d['points']}
            self.route_ver += 1
        except (ValueError, KeyError, TypeError):
            pass

    def vln_alive(self):
        _, age = self.rates['vln'].stats()
        return age is not None and age < 3.0

    def send_instruction(self, text):
        self.nav_pause('VLN 接管：GPS 导航已暂停')
        if self.vln_alive():
            self.instr_pub.publish(String(data=text))
            self.vln_pending = None
        else:
            # client is (re)starting: deliver once its /vln/state shows up
            self.vln_pending = text
            if 'tunnel' in self.sup.modules:
                self.sup.start('tunnel')
            self.sup.start('vln')

    def vln_stop(self):
        self.vln_pending = None
        self.vln_stop_pub.publish(Empty())

    def vln_tick(self):
        if self.vln_pending and self.vln_alive():
            self.instr_pub.publish(String(data=self.vln_pending))
            self.get_logger().info(f'VLN instruction delivered: {self.vln_pending}')
            self.vln_pending = None

    # ------------------------------------------------------------------ GPS navigation task
    def nav_alive(self):
        _, age = self.rates['nav'].stats()
        return self.nav if age is not None and age < 3.0 else None

    def nav_route_files(self):
        dirs = [os.path.join(self.nav_dir, 'routes')]
        try:
            dirs.insert(0, os.path.join(get_package_share_directory('vln_robot_nav'), 'routes'))
        except PackageNotFoundError:
            pass
        out = {}
        for d in dirs:
            if os.path.isdir(d):
                for f in sorted(os.listdir(d)):
                    if f.endswith('.csv'):
                        out[f[:-4]] = os.path.join(d, f)
        return out

    def nav_base(self, name, simplified=False):
        from vln_robot_nav.geo import load_route_csv, simplify_waypoints
        path = self.nav_route_files().get(str(name))
        if path is None:
            raise ValueError(f'没有路线 {name}')
        pts = load_route_csv(path)
        return simplify_waypoints(pts) if simplified else pts

    def nav_build(self, req):
        """Validate a task request from the browser and build its waypoints."""
        from vln_robot_nav.task import build_task
        mode = req.get('mode', 'route')
        task = {'mode': mode, 'start': req.get('start'), 'end': req.get('end'),
                'points': (req.get('points') or [])[:200], 'reverse': bool(req.get('reverse')),
                'loop': bool(req.get('loop')),
                'max_spacing': max(3.0, min(100.0, float(req.get('spacing', 25.0))))}
        base = self.nav_base(req.get('base', 'dishui_lake')) if mode == 'route' else None
        walker = None
        if mode == 'amap':
            from vln_robot_nav import amap
            key = amap.load_key()
            if not key:
                raise ValueError('机器人上未配置高德 Web 服务 Key（~/.config/vln_robot_web/amap_key）')
            if task['start'] is None and self.fix and self.fix.get('ok'):
                task['start'] = [self.fix['lat'], self.fix['lon']]
            walker = lambda a, b: amap.walking(a, b, key)  # noqa: E731
        res = build_task(task, base, walker)
        radius = max(1.0, min(30.0, float(req.get('radius', self.wp_radius))))
        max_v = max(0.1, min(self.max_v, float(req.get('max_v', self.max_v))))
        clean = dict(task, base=req.get('base') if mode == 'route' else None, radius=radius, max_v=max_v,
                     spacing=task['max_spacing'])
        res.update(radius=radius, max_v=max_v, eta_s=round(res['length_m'] / max(0.05, 0.85 * max_v)))
        return clean, res

    def _nav_summary(self):
        t = self.nav_task
        if not t:
            return None
        r, q = t['result'], t['req']
        return {'mode': q.get('mode'), 'base': q.get('base'), 'n': r['n'], 'length_m': r['length_m'],
                'closed': r['closed'], 'radius': q['radius'], 'max_v': q['max_v'], 'route_id': t['route_id'],
                'reverse': q.get('reverse'), 'loop': q.get('loop'), 'warnings': r.get('warnings', []),
                'eta_s': r.get('eta_s'), 't': t.get('t')}

    def _nav_set_cmd(self):
        q, r = self.nav_task['req'], self.nav_task['result']
        cmd = ['ros2', 'run', 'vln_robot_nav', 'gps_waypoint_follower', '--ros-args']
        try:
            yaml = os.path.join(get_package_share_directory('vln_robot_nav'), 'config', 'follower.yaml')
            if os.path.exists(yaml):
                cmd += ['--params-file', yaml]
        except PackageNotFoundError:
            pass
        v = q['max_v']
        # floats need a decimal point, otherwise ROS parses them as integers and the declare fails
        for k, val in (('route_file', self.nav_csv), ('closed', 'true' if r['closed'] else 'false'),
                       ('cmd_topic', self.nav_cmd_topic), ('wp_radius', f'{q["radius"]:.2f}'),
                       ('goal_tol', f'{min(3.0, q["radius"]):.2f}'),
                       ('max_v', f'{v:.2f}'), ('min_v', f'{min(0.15, v):.2f}'), ('align_v', f'{min(0.3, v):.2f}'),
                       ('wp_max_spacing', f'{q["spacing"]:.2f}'), ('autostart', 'false'), ('resume', 'true'),
                       ('progress_file', self.nav_progress)):
            cmd += ['-p', f'{k}:={val}']
        self.sup.modules['nav'].cmd = cmd

    def _nav_set_route(self):
        r = self.nav_task['result']
        self.route = {'radius': self.nav_task['req']['radius'], 'source': 'task', 'closed': r['closed'],
                      'points': r['points']}
        self.route_ver += 1

    def load_nav_task(self):
        try:
            with open(self.nav_task_file) as f:
                t = json.load(f)
            if not os.path.exists(self.nav_csv):
                return
            self.nav_task = t
            self._nav_set_cmd()
            self._nav_set_route()
        except FileNotFoundError:
            pass
        except Exception as e:  # noqa: BLE001
            self.nav_task = None
            self.get_logger().warning(f'cannot load nav task: {e}')

    def nav_save(self, req, res):
        os.makedirs(self.nav_dir, exist_ok=True)
        csv = 'lat,lon\n' + ''.join(f'{la:.7f},{lo:.7f}\n' for la, lo in res['points'])
        with open(self.nav_csv + '.tmp', 'w') as f:
            f.write(csv)
        os.replace(self.nav_csv + '.tmp', self.nav_csv)
        import hashlib
        self.nav_task = {'req': req, 'result': res, 't': time.time(),
                         'route_id': hashlib.sha1(csv.encode()).hexdigest()[:12]}
        with open(self.nav_task_file + '.tmp', 'w') as f:
            json.dump(self.nav_task, f)
        os.replace(self.nav_task_file + '.tmp', self.nav_task_file)
        self._nav_set_cmd()
        self._nav_set_route()

    def _nav_launch(self, fresh):
        """(Re)start the follower process; it comes up paused, nav_tick then calls /nav/start."""
        if fresh and os.path.exists(self.nav_progress):
            os.remove(self.nav_progress)
        m = self.sup.modules['nav']
        if m.proc is not None:
            self.sup.restart('nav')
        else:
            self.sup.start('nav')
        m.backoff = 2.0
        return time.time()

    def nav_start(self, req=None):
        """New task (req) or resume the saved one. Returns a user message."""
        if self.estop:
            raise ValueError('急停中')
        if req is not None:
            clean, res = self.nav_build(req)
            self.nav_save(clean, res)
        if not self.nav_task:
            raise ValueError('没有导航任务，请先设置并预览')
        self.vln_stop()
        self.stop()
        st, m = self.nav_alive(), self.sup.modules['nav']
        running = m.proc is not None and m.proc.poll() is None and not m.busy
        same = st is not None and st.get('route_id') == self.nav_task['route_id']
        since = None
        if req is not None:
            since, msg = self._nav_launch(fresh=True), '新任务启动中'
        elif not running or not same:
            since, msg = self._nav_launch(fresh=False), '导航节点启动中（从上次进度继续）'
        elif st.get('state') == 'DONE':
            since, msg = self._nav_launch(fresh=True), '任务已完成，重新开始'
        elif st.get('state') == 'FAULT':
            since, msg = self._nav_launch(fresh=False), '从故障中恢复（从上次进度继续）'
        else:
            msg = '继续'
        self.nav_pending = {'t': time.time(), 'route_id': self.nav_task['route_id'], 'since': since,
                            'future': None}
        self.nav_msg = msg
        return msg

    def nav_pause(self, why='已暂停'):
        had = self.nav_pending is not None
        self.nav_pending = None
        st = self.nav_alive()
        if st is not None and (st.get('enabled') or had):
            if self.nav_stop_cli.service_is_ready():
                self.nav_stop_cli.call_async(Trigger.Request())
            self.cmd_pub.publish(Twist())
            self.nav_msg = why
        elif had:
            self.nav_msg = why

    def nav_cancel(self):
        self.nav_pending = None
        self.sup.stop('nav')
        self.cmd_pub.publish(Twist())
        self.nav_msg = '任务已结束'

    def nav_tick(self):
        p = self.nav_pending
        if p is None:
            return
        if self.estop:
            self.nav_pending, self.nav_msg = None, '急停：取消启动'
            return
        if time.time() - p['t'] > 45.0:
            self.nav_pending, self.nav_msg = None, '启动超时：导航节点没有响应（查看 nav 模块日志）'
            return
        if p['future'] is not None:
            if p['future'].done():
                r = p['future'].result()
                self.nav_pending = None
                self.nav_msg = '导航中' if r is not None and r.success else f'启动失败：{getattr(r, "message", "")}'
            return
        st = self.nav_alive()
        if st is None or st.get('route_id') != p['route_id']:
            return
        if p['since'] is not None:
            m = self.sup.modules['nav']
            if m.busy or m.proc is None or m.started < p['since'] or self.nav_rx < m.started + 0.5:
                return
        if not self.nav_start_cli.service_is_ready():
            return
        p['future'] = self.nav_start_cli.call_async(Trigger.Request())

    # ------------------------------------------------------------------ periodic work
    def cmd_tick(self):
        fresh = time.time() - self.web_cmd_time < self.deadman and not self.estop
        if fresh:
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = self.web_cmd
            self.cmd_pub.publish(m)
            self.web_cmd_active = True
        elif self.web_cmd_active:
            # one explicit zero, then go quiet so other command sources (vln_client) are not overridden
            self.cmd_pub.publish(Twist())
            self.web_cmd_active = False

    def lidar_tick(self):
        msg = self.cloud_msg
        if msg is None or self.n_clients == 0:
            return
        if self.lidar_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                t = tf.transform.translation
                self.lidar_tf = (_quat_matrix(tf.transform.rotation).astype(np.float32),
                                 np.array([t.x, t.y, t.z], np.float32))
            except TransformException:
                pass
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float32).reshape(-1, 3)
        if self.lidar_tf is not None:
            R, t = self.lidar_tf
            xyz = xyz @ R.T + t
        m = (xyz[:, 2] > 0.05) & (xyz[:, 2] < 1.5) & (np.abs(xyz[:, 0]) < self.lidar_range) \
            & (np.abs(xyz[:, 1]) < self.lidar_range)
        cells = np.unique(np.round(xyz[m, :2] / self.lidar_grid).astype(np.int32), axis=0)
        self.lidar_xy = np.round(cells * self.lidar_grid * 100).astype(np.int32).flatten().tolist()
        self.lidar_seq += 1

    def sys_loop(self):
        last_wifi = 0.0
        prev = None
        while rclpy.ok():
            s = dict(self.sys)
            try:
                with open('/proc/stat') as f:
                    vals = [int(v) for v in f.readline().split()[1:]]
                idle, total = vals[3] + vals[4], sum(vals)
                if prev:
                    s['cpu'] = round(100 * (1 - (idle - prev[0]) / max(1, total - prev[1])))
                prev = (idle, total)
                with open('/proc/meminfo') as f:
                    mem = {ln.split(':')[0]: int(ln.split()[1]) for ln in f}
                s['mem'] = round(100 * (1 - mem['MemAvailable'] / mem['MemTotal']))
                s['load'] = round(os.getloadavg()[0], 2)
                for z in os.listdir('/sys/class/thermal'):
                    if z.startswith('thermal_zone'):
                        with open(f'/sys/class/thermal/{z}/type') as f:
                            if f.read().strip() == 'x86_pkg_temp':
                                with open(f'/sys/class/thermal/{z}/temp') as g:
                                    s['temp'] = round(int(g.read()) / 1000)
                if time.time() - last_wifi > 10:
                    last_wifi = time.time()
                    out = subprocess.run(['nmcli', '-t', '-f', 'ACTIVE,SSID,SIGNAL', 'dev', 'wifi'],
                                         capture_output=True, text=True, timeout=5).stdout
                    act = [ln for ln in out.splitlines() if ln.startswith('yes:')]
                    if act:
                        ssid, sig = act[0][4:].rsplit(':', 1)
                        s['wifi'] = ssid.replace('\\:', ':')
                        s['wifi_signal'] = int(sig) if sig.isdigit() else None
                    else:
                        s['wifi'], s['wifi_signal'] = None, None
            except Exception as e:  # noqa: BLE001 - stats are best effort
                s['err'] = str(e)
            self.sys = s
            time.sleep(2.0)

    def health_loop(self):
        url = self.vln_server_url.rsplit('/', 1)[0] + '/health'
        while rclpy.ok():
            t0 = time.time()
            try:
                with urllib.request.urlopen(url, timeout=3) as r:
                    body = r.read(2000)
                info = None
                try:
                    info = json.loads(body)
                except ValueError:
                    pass
                self.server = {'ok': True, 'rtt': round(time.time() - t0, 3), 'info': info, 'url': url,
                               't': time.time()}
            except Exception as e:  # noqa: BLE001 - report any failure as "server down"
                self.server = {'ok': False, 'error': f'{type(e).__name__}: {e}'[:200], 'url': url,
                               't': time.time()}
            time.sleep(5.0)

    # ------------------------------------------------------------------ API helpers
    def state(self):
        topics = {}
        for name, meter in self.rates.items():
            rate, age = meter.stats()
            topics[name] = {'hz': rate, 'age': age}
        return {'t': time.time(), 'topics': topics, 'odom': self.odom, 'imu': self.imu,
                'battery': self.battery, 'fix': self.fix, 'safety': self.safety,
                'wheel_cmd': self.wheel_cmd, 'sys': self.sys, 'estop': self.estop,
                'web_cmd': list(self.web_cmd) if self.web_cmd_active else None,
                'track_len': len(self.track), 's1_connected': self.s1_connected,
                'modules': self.sup.status(), 'managed': any(n in self.sup.modules for n, _ in DRIVERS),
                'vln': self.vln if self.vln_alive() else None, 'vln_pending': self.vln_pending,
                'server': self.server,
                'nav': self.nav_alive(),
                'nav_task': self._nav_summary(), 'nav_pending': self.nav_pending is not None,
                'nav_msg': self.nav_msg,
                'route_ver': self.route_ver,
                'limits': {'max_v': self.max_v, 'max_w': self.max_w, 'deadman': self.deadman},
                'clients': self.n_clients}

    def set_cmd(self, vx, vy, wz):
        def clip(v, lim):
            try:
                v = float(v)
            except (TypeError, ValueError):
                return 0.0
            return max(-lim, min(lim, v)) if math.isfinite(v) else 0.0
        self.web_cmd = (clip(vx, self.max_v), clip(vy, self.max_v), clip(wz, self.max_w))
        self.web_cmd_time = time.time()
        if self.vln_alive() and (self.vln or {}).get('active'):
            # manual driving takes over: cancel the VLN task so both don't fight over /vln/cmd_vel
            self.vln_stop()
        if self.nav_pending is not None or (self.nav_alive() or {}).get('enabled'):
            self.nav_pause('手动遥控接管：GPS 导航已暂停')

    def stop(self):
        self.web_cmd = (0.0, 0.0, 0.0)
        self.web_cmd_time = 0.0
        self.cmd_pub.publish(Twist())

    def set_estop(self, on):
        self.estop = bool(on)
        if self.estop:
            self.stop()
            self.vln_stop()
            self.nav_pause('急停：GPS 导航已暂停')
        self.estop_pub.publish(Bool(data=self.estop))
        self.get_logger().warn(f'web E-STOP {"ON" if self.estop else "OFF"}')

    def jpeg(self, width, quality):
        msg = self.img_msg
        if msg is None:
            return None, None
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec, width, quality)
        with self.jpeg_lock:
            if key in self.jpeg_cache:
                return key, self.jpeg_cache[key]
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif img.shape[2] == 1:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if width < msg.width:
                img = cv2.resize(img, (width, int(msg.height * width / msg.width)), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            data = buf.tobytes() if ok else None
            if len(self.jpeg_cache) > 8:
                self.jpeg_cache.clear()
            self.jpeg_cache[key] = data
            return key, data


def _num(qs, key, default, lo, hi, cast=float):
    try:
        return max(lo, min(hi, cast(qs.get(key, [default])[0])))
    except (TypeError, ValueError):
        return cast(default)


def make_handler(node: WebNode):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'vln_robot_web'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass

        def _authed(self, qs):
            if not node.require_token:
                return True
            if qs.get('token', [''])[0] == node.token or self.headers.get('X-Token') == node.token:
                return True
            c = SimpleCookie(self.headers.get('Cookie', ''))
            return 'tok' in c and c['tok'].value == node.token

        def _send(self, code, body, ctype='application/json', headers=None):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _static(self, rel):
            # normpath (not realpath): colcon --symlink-install makes these files symlinks into src/
            base = os.path.normpath(os.path.abspath(node.static_dir))
            path = os.path.normpath(os.path.join(base, rel))
            if not path.startswith(base + os.sep) or not os.path.isfile(path):
                return self._send(404, {'error': 'not found'})
            ctype = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
                     '.css': 'text/css; charset=utf-8', '.png': 'image/png', '.svg': 'image/svg+xml'}.get(
                         os.path.splitext(path)[1], 'application/octet-stream')
            with open(path, 'rb') as f:
                self._send(200, f.read(), ctype)

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            if u.path.startswith('/static/'):
                return self._static(u.path[len('/static/'):])
            if not self._authed(qs):
                if u.path in ('/', '/index.html'):
                    return self._static('login.html')
                return self._send(401, {'error': 'unauthorized'})
            if u.path in ('/', '/index.html'):
                if 'token' in qs:
                    return self._send(302, '', 'text/plain', {
                        'Location': '/',
                        'Set-Cookie': f'tok={node.token}; Path=/; Max-Age=2592000; SameSite=Strict; HttpOnly'})
                return self._static('index.html')
            if u.path == '/api/state':
                return self._send(200, node.state())
            if u.path == '/api/track':
                return self._send(200, list(node.track))
            if u.path == '/api/route':
                if node.route:
                    return self._send(200, node.route)
                return self._send(404, {'error': 'no route'})
            if u.path == '/api/events':
                return self._events(qs)
            if u.path == '/api/nav/routes':
                from vln_robot_nav import amap
                return self._send(200, {'routes': list(node.nav_route_files()), 'amap': bool(amap.load_key()),
                                        'task': node.nav_task})
            if u.path == '/api/nav/base':
                try:
                    pts = node.nav_base(qs.get('name', [''])[0], simplified=True)
                except ValueError as e:
                    return self._send(404, {'error': str(e)})
                return self._send(200, {'points': [[round(a, 7), round(b, 7)] for a, b in pts]})
            if u.path == '/stream.mjpg':
                return self._mjpeg(qs)
            if u.path == '/snapshot.jpg':
                _, data = node.jpeg(_num(qs, 'w', 640, 160, 1280, int), _num(qs, 'q', 85, 20, 95, int))
                return self._send(200, data, 'image/jpeg') if data else self._send(503, {'error': 'no image'})
            return self._send(404, {'error': 'not found'})

        def _events(self, qs):
            period = _num(qs, 'period', 0.5, 0.2, 5.0)
            want_lidar = qs.get('lidar', ['1'])[0] == '1'
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            node.n_clients += 1
            last_lidar = -1
            try:
                while rclpy.ok():
                    st = node.state()
                    if want_lidar and node.lidar_seq != last_lidar:
                        last_lidar = node.lidar_seq
                        st['lidar'] = node.lidar_xy
                    self.wfile.write(f'data: {json.dumps(st, separators=(",", ":"))}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(period)
            except OSError:
                pass
            finally:
                node.n_clients -= 1
            self.close_connection = True

        def _mjpeg(self, qs):
            fps = _num(qs, 'fps', 5, 0.5, 15.0)
            quality = _num(qs, 'q', 60, 20, 95, int)
            width = _num(qs, 'w', 640, 160, 1280, int)
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = None
            try:
                while rclpy.ok():
                    t0 = time.time()
                    key, data = node.jpeg(width, quality)
                    if data and key != last:
                        last = key
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
                        self.wfile.flush()
                    time.sleep(max(0.01, 1.0 / fps - (time.time() - t0)))
            except OSError:
                pass
            self.close_connection = True

        def do_POST(self):
            u = urlparse(self.path)
            if not self._authed(parse_qs(u.query)):
                return self._send(401, {'error': 'unauthorized'})
            n = int(self.headers.get('Content-Length', '0') or 0)
            try:
                body = json.loads(self.rfile.read(n)) if n else {}
            except ValueError:
                return self._send(400, {'error': 'bad json'})
            if u.path == '/api/cmd':
                if node.estop:
                    return self._send(409, {'error': 'estop'})
                node.set_cmd(body.get('vx', 0), body.get('vy', 0), body.get('wz', 0))
                return self._send(200, {'ok': True, 'cmd': node.web_cmd,
                                        'blocked': (node.safety or {}).get('blocked')})
            if u.path == '/api/stop':
                node.stop()
                return self._send(200, {'ok': True})
            if u.path == '/api/estop':
                node.set_estop(body.get('on', True))
                return self._send(200, {'ok': True, 'estop': node.estop})
            if u.path == '/api/module':
                name, action = str(body.get('name', '')), body.get('action', 'restart')
                if name != 'all' and name not in node.sup.modules:
                    return self._send(404, {'error': f'unknown module {name}'})
                if name == 'nav' and not node.nav_task:
                    return self._send(400, {'error': '没有导航任务'})
                if action == 'restart':
                    if name in ('all', 'chassis'):
                        node.stop()
                    return self._send(200, {'ok': True, 'restarting': node.sup.restart(name)})
                if action in ('start', 'stop') and name != 'all':
                    return self._send(200, {'ok': getattr(node.sup, action)(name)})
                return self._send(400, {'error': 'bad action'})
            if u.path == '/api/vln':
                text = str(body.get('instruction', '')).strip()[:500]
                if not text:
                    return self._send(400, {'error': 'empty instruction'})
                if node.estop:
                    return self._send(409, {'error': 'estop'})
                node.send_instruction(text)
                return self._send(200, {'ok': True, 'pending': node.vln_pending is not None})
            if u.path == '/api/vln/stop':
                node.vln_stop()
                return self._send(200, {'ok': True})
            if u.path.startswith('/api/nav/'):
                try:
                    if u.path == '/api/nav/preview':
                        _, res = node.nav_build(body.get('task') or {})
                        return self._send(200, dict(res, ok=True))
                    if u.path == '/api/nav/start':
                        msg = node.nav_start(body.get('task'))
                        return self._send(200, {'ok': True, 'msg': msg, 'task': node._nav_summary()})
                    if u.path == '/api/nav/pause':
                        node.nav_pause()
                        return self._send(200, {'ok': True})
                    if u.path == '/api/nav/cancel':
                        node.nav_cancel()
                        return self._send(200, {'ok': True})
                except (ValueError, TypeError, KeyError) as e:
                    return self._send(400, {'error': str(e)})
                except Exception as e:  # noqa: BLE001 - e.g. AMap network errors
                    return self._send(502, {'error': f'{type(e).__name__}: {e}'[:300]})
            return self._send(404, {'error': 'not found'})

    return Handler


def main():
    rclpy.init()
    node = WebNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.httpd.shutdown()
        node.sup.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

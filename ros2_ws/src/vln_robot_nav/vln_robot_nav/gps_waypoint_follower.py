"""GPS route follower node (RoboMaster S1 + WHEELTEC G60).

  /fix   sensor_msgs/NavSatFix   (nmea_navsat_driver)
  /odom  nav_msgs/Odometry       (robomaster_ros, gyro yaw)
  -> /nav/cmd_vel geometry_msgs/Twist  (goes through safety_stop -> /cmd_vel)
  -> /nav/status  std_msgs/String (JSON, 2 Hz)

Never moves on its own: call  ros2 service call /nav/start std_srvs/srv/Trigger
Pause:                         ros2 service call /nav/stop  std_srvs/srv/Trigger
Progress is saved to `progress_file`; restart with resume:=true after a battery swap.
"""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .follower import DONE, FAULT, FollowerParams, RouteFollower
from .geo import LoopRoute, load_route_csv


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

        route = LoopRoute(load_route_csv(self.route_file))
        start_progress = None
        if resume and os.path.exists(self.progress_file):
            with open(self.progress_file) as f:
                saved = json.load(f)
            if saved.get('route_file') == self.route_file:
                start_progress = float(saved['progress'])
        self.f = RouteFollower(route, params, start_progress)
        self.get_logger().info(
            f'route {self.route_file}: {route.length:.0f} m, {len(route.p)} pts'
            + (f', resuming at {start_progress:.0f} m' if start_progress is not None else ''))

        self.create_subscription(NavSatFix, p('fix_topic', '/fix').value, self.on_fix, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value, self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic', '/nav/cmd_vel').value, 1)
        self.status_pub = self.create_publisher(String, '/nav/status', 1)
        self.create_service(Trigger, '/nav/start', self.srv_start)
        self.create_service(Trigger, '/nav/stop', self.srv_stop)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.last_save = 0.0
        self.last_state = None

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
        self.cmd_pub.publish(Twist())
        res.success, res.message = True, 'paused'
        self.get_logger().info('navigation paused')
        return res

    def tick(self):
        v, w = self.f.step(self.now())
        if self.f.state != self.last_state:
            self.get_logger().info(f'state {self.last_state} -> {self.f.state} {self.f.reason}')
            self.last_state = self.f.state
        cmd = Twist()
        if self.enabled:
            cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)
        if self.f.s is not None and self.now() - self.last_save > 5.0:
            self.save_progress()

    def save_progress(self):
        self.last_save = self.now()
        os.makedirs(os.path.dirname(self.progress_file), exist_ok=True)
        tmp = self.progress_file + '.tmp'
        with open(tmp, 'w') as fh:
            json.dump({'route_file': self.route_file, 'progress': self.f.progress,
                       'state': self.f.state, 'time': time.time()}, fh)
        os.replace(tmp, self.progress_file)

    def publish_status(self):
        f = self.f
        lla = f.route.enu.to_lla(*f.pos) if f.pos is not None else None
        st = {'state': f.state, 'enabled': self.enabled, 'reason': f.reason,
              'progress_m': round(f.progress, 1), 'route_m': round(f.route.length, 1),
              'xte_m': round(f.xte, 2),
              'yaw_deg': None if f.yaw is None else round(math.degrees(f.yaw), 1),
              'heading_updates': f.n_offset_updates,
              'lat': None if lla is None else round(lla[0], 7),
              'lon': None if lla is None else round(lla[1], 7)}
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

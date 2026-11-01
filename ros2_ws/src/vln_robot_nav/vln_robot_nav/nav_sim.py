"""Kinematic simulator replacing the S1 chassis + GPS for dry runs.

/cmd_vel -> unicycle model -> /fix (1 Hz, Gaussian noise) + /odom (20 Hz, odom frame rotated
by an unknown yaw offset, gyro drift). Starts at the first route point facing along the route.
`time_scale` > 1 runs faster than real time.
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geo import LoopRoute, load_route_csv


class NavSim(Node):
    def __init__(self):
        super().__init__('nav_sim')
        p = self.declare_parameter
        self.route = LoopRoute(load_route_csv(p('route_file', '').value))
        self.gps_std = p('gps_std', 1.5).value
        self.k = p('time_scale', 1.0).value
        self.drift = math.radians(p('gyro_drift_deg_per_min', 0.5).value) / 60.0
        start_s = p('start_s', 0.0).value
        self.x, self.y = self.route.point_at(start_s)
        self.yaw = self.route.tangent_at(start_s) + math.radians(p('start_yaw_error_deg', 10.0).value)
        self.odom_off = random.uniform(-math.pi, math.pi)
        self.ox = self.oy = 0.0
        self.cmd = Twist()
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 1)
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.05, self.step)
        self.create_timer(1.0 / self.k, self.pub_fix)
        self.t = 0.0

    def step(self):
        dt = 0.05 * self.k
        self.t += dt
        v, w = self.cmd.linear.x, self.cmd.angular.z
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        oyaw = self.yaw - self.odom_off + self.drift * self.t
        self.ox += v * math.cos(oyaw) * dt
        self.oy += v * math.sin(oyaw) * dt
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.ox, self.oy
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
        self.odom_pub.publish(m)

    def pub_fix(self):
        lat, lon = self.route.enu.to_lla(self.x + random.gauss(0, self.gps_std),
                                         self.y + random.gauss(0, self.gps_std))
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gps_link'
        m.status.status = NavSatStatus.STATUS_FIX
        m.latitude, m.longitude = lat, lon
        m.position_covariance = [self.gps_std ** 2, 0.0, 0.0, 0.0, self.gps_std ** 2, 0.0, 0.0, 0.0, 9.0]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(m)


def main():
    rclpy.init()
    node = NavSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Safety gate between the VLN client and the chassis.

/vln/cmd_vel (Twist) --> [obstacle check on /livox/lidar + watchdog] --> /cmd_vel (Twist)

* Forward motion is blocked if any LiDAR point lies inside a box in front of the robot
  (base_link frame: 0 < x < stop_distance, |y| < half_width, z_min < z < z_max).
* Commands older than `cmd_timeout` -> zero (protects against a dead client / Wi-Fi drop).
* Speeds are clamped to max_v / max_w.
* /safety/estop (std_msgs/Bool, latched): true -> zero output regardless of the command source.
* /safety/status (std_msgs/String, JSON, 5 Hz): blocked / lidar_ok / estop / output command.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import quaternion_to_matrix


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.stop_distance = p('stop_distance', 0.45).value
        self.half_width = p('half_width', 0.20).value
        self.z_min = p('z_min', 0.03).value
        self.z_max = p('z_max', 0.60).value
        self.max_v = p('max_v', 0.5).value
        self.max_w = p('max_w', 1.2).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.cloud_timeout = p('cloud_timeout', 0.5).value
        self.require_lidar = p('require_lidar', True).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.blocked = False
        self.last_cloud = 0.0
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0

        self.create_subscription(PointCloud2, p('cloud_topic', '/livox/lidar').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, p('cmd_in', '/vln/cmd_vel').value, self.on_cmd, 1)
        self.pub = self.create_publisher(Twist, p('cmd_out', '/cmd_vel').value, 1)
        self.estop = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, latched)
        self.status_pub = self.create_publisher(String, '/safety/status', 1)
        self.last_out = Twist()
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.publish_status)

    def on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn('E-STOP engaged' if msg.data else 'E-STOP released')
        self.estop = msg.data

    def publish_status(self):
        now = time.time()
        o = self.last_out
        self.status_pub.publish(String(data=json.dumps({
            'blocked': self.blocked, 'estop': self.estop,
            'lidar_ok': (now - self.last_cloud < self.cloud_timeout) or not self.require_lidar,
            'cmd_age': round(min(now - self.last_cmd_time, 999.0), 2),
            'out': [round(o.linear.x, 3), round(o.linear.y, 3), round(o.angular.z, 3)],
            'max_v': self.max_v, 'max_w': self.max_w})))

    def on_cloud(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time(),
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            return
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64).reshape(-1, 3)
        q, t = tf.transform.rotation, tf.transform.translation
        xyz = xyz @ quaternion_to_matrix(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
        m = ((xyz[:, 0] > 0.0) & (xyz[:, 0] < self.stop_distance)
             & (np.abs(xyz[:, 1]) < self.half_width)
             & (xyz[:, 2] > self.z_min) & (xyz[:, 2] < self.z_max))
        blocked = int(m.sum()) >= 5
        if blocked != self.blocked:
            self.get_logger().warn('obstacle ahead -> forward motion blocked' if blocked else 'path clear')
        self.blocked = blocked
        self.last_cloud = time.time()

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def tick(self):
        now = time.time()
        out = Twist()
        if now - self.last_cmd_time < self.cmd_timeout and not self.estop:
            lidar_ok = (now - self.last_cloud < self.cloud_timeout) or not self.require_lidar
            v = float(np.clip(self.last_cmd.linear.x, -self.max_v, self.max_v))
            if v > 0.0 and (self.blocked or not lidar_ok):
                v = 0.0
            out.linear.x = v
            out.linear.y = float(np.clip(self.last_cmd.linear.y, -self.max_v, self.max_v))
            out.angular.z = float(np.clip(self.last_cmd.angular.z, -self.max_w, self.max_w))
        self.last_out = out
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

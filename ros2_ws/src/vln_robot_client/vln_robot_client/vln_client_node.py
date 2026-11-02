"""Robot-side VLN client.

Streams RGB (+ optional depth) to a remote VLN server over HTTP and executes the reply:
  * {"discrete_action": [..]}   -> VLN-CE actions (0 stop, 1 fwd 0.25 m, 2 left 15 deg, 3 right 15 deg)
  * {"trajectory": [[x, y, ...], ..]} -> body-frame waypoints (x forward, y left) followed with pure pursuit

The HTTP contract matches InternNav `scripts/realworld/http_internvla_server.py`
(multipart fields: image=jpeg, depth=uint16 png in 0.1 mm, json={"reset", "idx", "instruction"}),
see server/internvla_n1_server.py in this repo.

Usage:
  ros2 run vln_robot_client vln_client --ros-args -p server_url:=http://<gpu-host>:5801/eval_dual
  ros2 topic pub --once /vln/instruction std_msgs/String "{data: 'Go out of the room and stop at the red chair'}"
  ros2 topic pub --once /vln/stop std_msgs/Empty {}
"""
import json
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from .geometry import (body_traj_to_world, discrete_actions_to_goal, goal_controller,
                       pure_pursuit, yaw_from_quaternion)


class VlnClient(Node):
    def __init__(self):
        super().__init__('vln_client')
        p = self.declare_parameter
        self.server_url = p('server_url', 'http://192.168.31.100:5801/eval_dual').value
        self.use_depth = p('use_depth', True).value
        self.request_period = p('request_period', 0.5).value
        self.http_timeout = p('http_timeout', 15.0).value
        self.jpeg_quality = int(p('jpeg_quality', 85).value)
        self.max_v = p('max_v', 0.3).value
        self.max_w = p('max_w', 0.8).value
        self.traj_skip = int(p('traj_skip', 3).value)
        self.step_m = p('forward_step_m', 0.25).value
        self.turn_deg = p('turn_step_deg', 15.0).value
        self.instruction = p('instruction', '').value

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.rgb = None
        self.rgb_stamp = 0.0
        self.depth = None
        self.odom_queue = deque(maxlen=200)
        self.pose = None
        self.active = False
        self.reset = True
        self.idx = 0
        self.mode = None          # 'goal' | 'path'
        self.goal = None
        self.path = None
        self.new_image = False

        self.create_subscription(Image, p('image_topic', '/camera/image_raw').value,
                                 self.on_image, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, p('depth_topic', '/vln/depth').value,
                                     self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value, self.on_odom, 20)
        self.create_subscription(String, '/vln/instruction', self.on_instruction, 1)
        self.create_subscription(Empty, '/vln/stop', self.on_stop, 1)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic', '/vln/cmd_vel').value, 1)
        self.status_pub = self.create_publisher(String, '/vln/status', 10)
        # machine-readable task state for the web dashboard (JSON, 1 Hz + on change)
        self.state_pub = self.create_publisher(String, '/vln/state', 10)
        self.state = 'idle'       # idle | waiting_image | waiting_depth | requesting | executing | done | stopped | error
        self.state_msg = ''
        self.last_error = None
        self.last_reply = None
        self.last_rtt = None
        self.task_t0 = None
        self.server_ok = None
        self.server_info = None
        self.create_timer(0.1, self.control_step)
        self.create_timer(1.0, self.publish_state)
        threading.Thread(target=self.health_loop, daemon=True).start()

        self._thread = threading.Thread(target=self.planning_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'VLN client ready, server={self.server_url}, use_depth={self.use_depth}')
        if self.instruction:
            self.start(self.instruction)

    # ---------------- callbacks ----------------
    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.rgb = img
            self.rgb_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.new_image = True

    def on_depth(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg)
        if d.dtype == np.uint16:  # assume mm
            d = d.astype(np.float32) / 1000.0
        with self.lock:
            self.depth = d.astype(np.float32)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.pose = pose
            self.odom_queue.append((t, pose))

    def on_instruction(self, msg):
        self.start(msg.data)

    def on_stop(self, _msg):
        self.stop('stopped by user')

    # ---------------- task control ----------------
    def start(self, instruction):
        with self.lock:
            self.instruction = instruction
            self.active = True
            self.reset = True
            self.idx = 0
            self.mode = self.goal = self.path = None
            self.last_reply = None
            self.last_error = None
            self.task_t0 = time.time()
        self.set_state('waiting_image', f'start: {instruction}')

    def stop(self, reason, state='stopped'):
        with self.lock:
            self.active = False
            self.mode = self.goal = self.path = None
        self.cmd_pub.publish(Twist())
        self.set_state(state, f'stop: {reason}')

    def publish_status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def set_state(self, state, text=None):
        changed = state != self.state or (text and text != self.state_msg)
        self.state = state
        if text:
            self.state_msg = text
            self.publish_status(text)
        if changed:
            self.publish_state()

    def publish_state(self):
        with self.lock:
            d = {'state': self.state, 'msg': self.state_msg, 'active': self.active,
                 'instruction': self.instruction, 'idx': self.idx, 'mode': self.mode,
                 'elapsed': round(time.time() - self.task_t0, 1) if self.task_t0 and self.active else None,
                 'reply': self.last_reply, 'rtt': self.last_rtt, 'error': self.last_error,
                 'server_url': self.server_url, 'server_ok': self.server_ok, 'server': self.server_info,
                 'use_depth': self.use_depth, 't': time.time()}
        self.state_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

    def health_loop(self):
        url = self.server_url.rsplit('/', 1)[0] + '/health'
        while rclpy.ok():
            try:
                r = requests.get(url, timeout=3)
                self.server_ok = r.ok
                try:
                    self.server_info = r.json()
                except ValueError:
                    self.server_info = None
            except requests.RequestException as e:
                self.server_ok = False
                self.server_info = {'error': type(e).__name__}
            time.sleep(5.0)

    # ---------------- planning (HTTP) ----------------
    def planning_loop(self):
        while rclpy.ok():
            t0 = time.time()
            with self.lock:
                ready = self.active and self.new_image and self.rgb is not None and self.odom_queue
                if ready:
                    self.new_image = False
                    rgb = self.rgb.copy()
                    depth = None if self.depth is None else self.depth.copy()
                    stamp = self.rgb_stamp
                    pose = min(self.odom_queue, key=lambda o: abs(o[0] - stamp))[1]
                    payload = {'reset': self.reset, 'idx': self.idx, 'instruction': self.instruction}
            if not ready:
                with self.lock:
                    waiting = self.active and self.mode is None
                if waiting and self.state not in ('waiting_image', 'error'):
                    self.set_state('waiting_image')
                time.sleep(0.05)
                continue
            if self.use_depth and depth is None:
                self.get_logger().warn('waiting for depth image', throttle_duration_sec=2.0)
                self.set_state('waiting_depth')
                time.sleep(0.1)
                continue
            if self.mode is None:
                self.set_state('requesting')
            t_req = time.time()
            try:
                reply = self.query_server(rgb, depth, payload)
            except Exception as e:  # noqa: BLE001 - keep the loop alive on network errors
                self.get_logger().error(f'server request failed: {e}', throttle_duration_sec=2.0)
                self.last_error = f'{type(e).__name__}: {e}'[:300]
                with self.lock:
                    self.mode = self.goal = self.path = None
                self.cmd_pub.publish(Twist())
                self.set_state('error')
                time.sleep(0.5)
                continue
            self.last_rtt = round(time.time() - t_req, 3)
            self.last_error = None
            self.last_reply = self.summarize(reply)
            with self.lock:
                self.reset = False
                self.idx += 1
            self.handle_reply(reply, pose)
            if self.active:
                self.set_state('executing')
            time.sleep(max(0.0, self.request_period - (time.time() - t0)))

    @staticmethod
    def summarize(reply):
        s = {k: reply[k] for k in ('discrete_action', 'pixel_goal', 'latency_s') if k in reply}
        if 'trajectory' in reply:
            traj = reply['trajectory']
            s['traj_n'] = len(traj)
            if traj:
                s['traj_end'] = [round(float(v), 2) for v in traj[-1][:2]]
        if isinstance(reply.get('depth'), dict):
            s['depth'] = reply['depth']
        return s

    def query_server(self, rgb, depth, payload):
        ok, jpg = cv2.imencode('.jpg', rgb, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        assert ok
        if depth is None:
            depth = np.zeros(rgb.shape[:2], dtype=np.float32)
        # Server convention (InternNav realworld): uint16 PNG, value / 10000 = meters
        d16 = np.clip(np.nan_to_num(depth) * 10000.0, 0, 65535).astype(np.uint16)
        ok, png = cv2.imencode('.png', d16)
        assert ok
        files = {'image': ('rgb.jpg', jpg.tobytes(), 'image/jpeg'),
                 'depth': ('depth.png', png.tobytes(), 'image/png')}
        r = requests.post(self.server_url, files=files, data={'json': json.dumps(payload)},
                          timeout=self.http_timeout)
        r.raise_for_status()
        return r.json()

    def handle_reply(self, reply, pose_at_capture):
        if 'discrete_action' in reply:
            actions = [int(a) for a in reply['discrete_action']]
            with self.lock:
                start = self.pose or pose_at_capture
            goal, stop = discrete_actions_to_goal(
                start, actions, self.step_m, np.radians(self.turn_deg))
            if stop:
                self.stop('model issued STOP (task finished)', state='done')
                return
            with self.lock:
                self.mode, self.goal, self.path = 'goal', goal, None
        elif 'trajectory' in reply:
            path = body_traj_to_world(pose_at_capture, reply['trajectory'], skip=self.traj_skip)
            with self.lock:
                self.mode, self.path, self.goal = 'path', path, None
        else:
            self.get_logger().warn(f'unknown reply: {reply}')

    # ---------------- low-level control ----------------
    def control_step(self):
        with self.lock:
            active, mode, pose = self.active, self.mode, self.pose
            goal, path = self.goal, self.path
        if not active or pose is None or mode is None:
            return
        if mode == 'goal':
            v, w, done = goal_controller(pose, goal, self.max_v, self.max_w)
        else:
            v, w, done = pure_pursuit(pose, path, max_v=self.max_v, max_w=self.max_w)
        cmd = Twist()
        if not done:
            cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = VlnClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

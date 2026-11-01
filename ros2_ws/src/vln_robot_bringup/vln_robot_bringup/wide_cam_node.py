"""Widest-FOV publisher for the 'USB 2.0 Camera 4K' (eba4:1303).

Only the 4000x3000 MJPG mode uses the full sensor (~66 deg HFOV); every smaller mode is a centre crop
(~50 deg). usb_cam would fully decode 12 MP frames, which is too slow on the N305, so this node grabs the
raw MJPG, decodes at 1/4 scale (libjpeg DCT scaling, ~25 ms) and resizes to the calibrated output size.

Publishes /camera/image_raw (bgr8) and /camera/camera_info (from camera_info.yaml) - same topics as usb_cam.
"""
import os
import threading

import cv2
import numpy as np
import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    m = CameraInfo()
    m.width, m.height = int(c['image_width']), int(c['image_height'])
    m.distortion_model = c.get('distortion_model', 'plumb_bob')
    m.d = [float(v) for v in c['distortion_coefficients']['data']]
    m.k = [float(v) for v in c['camera_matrix']['data']]
    m.r = [float(v) for v in c['rectification_matrix']['data']]
    m.p = [float(v) for v in c['projection_matrix']['data']]
    return m


class WideCam(Node):
    def __init__(self):
        super().__init__('vln_cam')
        p = self.declare_parameter
        dev = p('video_device', '/dev/vln_cam').value
        cw, ch = p('capture_width', 4000).value, p('capture_height', 3000).value
        self.size = (p('image_width', 640).value, p('image_height', 480).value)
        self.frame_id = p('frame_id', 'camera_optical_frame').value
        info = p('camera_info_path', os.path.join(
            get_package_share_directory('vln_robot_bringup'), 'config', 'camera_info.yaml')).value

        self.info = load_camera_info(info)
        if (self.info.width, self.info.height) != self.size:
            self.get_logger().warn(f'camera_info is {self.info.width}x{self.info.height}, output is {self.size}')
        self.info.header.frame_id = self.frame_id

        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cw)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ch)
        self.cap.set(cv2.CAP_PROP_FPS, p('framerate', 15.0).value)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        got = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if not self.cap.isOpened() or got != (cw, ch):
            raise RuntimeError(f'cannot open {dev} at {cw}x{ch} MJPG (got {got}); is usb_cam running?')
        r = cw // self.size[0]
        self.flag = (cv2.IMREAD_COLOR if r < 2 else cv2.IMREAD_REDUCED_COLOR_2 if r < 4
                     else cv2.IMREAD_REDUCED_COLOR_4 if r < 8 else cv2.IMREAD_REDUCED_COLOR_8)

        self.pub = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
        self.get_logger().info(f'{dev}: {cw}x{ch} MJPG -> {self.size[0]}x{self.size[1]} bgr8')
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.running and rclpy.ok():
            ok, buf = self.cap.read()
            if not ok or buf is None:
                self.get_logger().warn('frame grab failed', throttle_duration_sec=2.0)
                continue
            stamp = self.get_clock().now().to_msg()
            im = cv2.imdecode(np.asarray(buf).reshape(-1), self.flag)
            if im is None:
                continue
            if (im.shape[1], im.shape[0]) != self.size:
                im = cv2.resize(im, self.size, interpolation=cv2.INTER_AREA)
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
            msg.height, msg.width = im.shape[:2]
            msg.encoding, msg.step = 'bgr8', im.shape[1] * 3
            msg.data = im.tobytes()
            self.info.header.stamp = stamp
            try:
                self.pub.publish(msg)
                self.pub_info.publish(self.info)
            except Exception:  # context shut down while grabbing
                if not rclpy.ok():
                    break
                raise

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    n = WideCam()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

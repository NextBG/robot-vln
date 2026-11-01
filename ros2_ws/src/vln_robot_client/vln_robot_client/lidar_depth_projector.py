"""Project Livox Mid-360 points into the webcam image to produce a sparse metric depth image.

Subscribes : /livox/lidar (PointCloud2), /camera/camera_info (CameraInfo)
Publishes  : /vln/depth (sensor_msgs/Image, 32FC1, meters, 0 = no return), stamped like the cloud,
             frame = camera optical frame.

Mid-360 is non-repetitive and sparse per 100 ms frame; `accumulate` merges the last N clouds
(already in camera frame at their own time; fine at low speed). `dilate_px` fills small holes.
If the policy needs dense depth, prefer running a metric monocular depth model on the GPU server.
"""
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import project_points_to_depth, quaternion_to_matrix


class LidarDepthProjector(Node):
    def __init__(self):
        super().__init__('lidar_depth_projector')
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        self.accumulate = int(self.declare_parameter('accumulate', 3).value)
        self.dilate_px = int(self.declare_parameter('dilate_px', 2).value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.info = None
        self.clouds = deque(maxlen=max(1, self.accumulate))
        self.create_subscription(CameraInfo, self.declare_parameter(
            'camera_info_topic', '/camera/camera_info').value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.declare_parameter(
            'cloud_topic', '/livox/lidar').value, self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, '/vln/depth', qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_cloud(self, msg):
        if self.info is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=2.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.camera_frame, msg.header.frame_id,
                                                 Time(), timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f'TF {msg.header.frame_id}->{self.camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64).reshape(-1, 3)
        q, t = tf.transform.rotation, tf.transform.translation
        R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        xyz_cam = xyz @ R.T + np.array([t.x, t.y, t.z])
        self.clouds.append(xyz_cam)

        k = self.info.k
        depth = project_points_to_depth(np.concatenate(self.clouds), k[0], k[4], k[2], k[5],
                                        self.info.width, self.info.height)
        if self.dilate_px > 0:
            # grey dilation on inverse depth keeps the nearest return when spreading points
            inv = np.where(depth > 0, 1.0 / np.maximum(depth, 1e-6), 0).astype(np.float32)
            kernel = np.ones((2 * self.dilate_px + 1, 2 * self.dilate_px + 1), np.uint8)
            inv = cv2.dilate(inv, kernel)
            depth = np.where(inv > 0, 1.0 / np.maximum(inv, 1e-6), 0).astype(np.float32)
        out = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.camera_frame
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarDepthProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

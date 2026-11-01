"""Full robot bringup on the onboard PC (MOREFINE M11, Ubuntu 22.04, ROS 2 Humble).

    ros2 launch vln_robot_bringup robot.launch.py
    ros2 launch vln_robot_bringup robot.launch.py use_lidar:=false use_gnss:=false

Starts: RoboMaster S1 chassis driver (robomaster_ros), USB webcam (wide_cam_node, or usb_cam with
camera_driver:=usb_cam),
Livox Mid-360 (livox_ros_driver2), WHEELTEC G60 (nmea_navsat_driver) and the
static TFs between base_link and the sensors.
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = 'vln_robot_bringup'


def _static_tf(parent, child, p):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'tf_{parent}_to_{child}',
        arguments=[
            '--x', str(p['x']), '--y', str(p['y']), '--z', str(p['z']),
            '--roll', str(p['roll']), '--pitch', str(p['pitch']), '--yaw', str(p['yaw']),
            '--frame-id', parent, '--child-frame-id', child,
        ])


def generate_launch_description():
    share = get_package_share_directory(PKG)
    cfg = os.path.join(share, 'config')
    with open(os.path.join(cfg, 'extrinsics.yaml')) as f:
        extr = yaml.safe_load(f)

    use_chassis = LaunchConfiguration('use_chassis')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    use_gnss = LaunchConfiguration('use_gnss')

    chassis = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robomaster_ros'), 'launch', 's1.launch')),
        launch_arguments={
            'conn_type': LaunchConfiguration('s1_conn_type'),
            'serial_number': LaunchConfiguration('s1_serial_number'),
            'chassis_rate': '20',
            # The USB webcam is used for VLN; disable the S1 gimbal camera/audio streams to save CPU.
            # Modes: OFF=0, ON=1, ON_DEMAND=2, DISABLED=-1
            'video': '-1',
            'audio': '-1',
        }.items(),
        condition=IfCondition(use_chassis))

    # Default: full-sensor 4000x3000 grab downscaled to 640x480 (~66 deg HFOV, camera_info.yaml).
    # camera_driver:=usb_cam uses the cropped 640x480 mode (~50 deg) with camera_info_640crop.yaml.
    camera_driver = LaunchConfiguration('camera_driver')
    camera_wide = Node(
        package='vln_robot_bringup', executable='wide_cam_node', name='vln_cam', namespace='camera',
        parameters=[os.path.join(cfg, 'wide_cam.yaml')],
        condition=IfCondition(PythonExpression(["'", use_camera, "' == 'true' and '", camera_driver, "' == 'wide'"])))
    camera_usb = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='vln_cam', namespace='camera',
        parameters=[os.path.join(cfg, 'usb_cam.yaml')],
        condition=IfCondition(PythonExpression(["'", use_camera, "' == 'true' and '", camera_driver, "' == 'usb_cam'"])))

    lidar = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{
            'xfer_format': 0,          # 0 = PointCloud2 (PointXYZRTLT); use 1 (CustomMsg) for FAST-LIO
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': os.path.join(cfg, 'MID360_config.json'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        condition=IfCondition(use_lidar))

    gnss = Node(
        package='nmea_navsat_driver', executable='nmea_serial_driver',
        name='nmea_navsat_driver', output='screen',
        parameters=[os.path.join(cfg, 'gnss.yaml')],
        condition=IfCondition(use_gnss))

    tfs = [_static_tf('base_link', name, pose) for name, pose in extr.items()]
    tfs.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_camera_optical',
        arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                   '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']))

    return LaunchDescription([
        DeclareLaunchArgument('use_chassis', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('camera_driver', default_value='wide',
                              description='wide (full sensor, ~66 deg HFOV) or usb_cam (640x480 crop, ~50 deg)'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_gnss', default_value='true'),
        DeclareLaunchArgument('s1_conn_type', default_value='rndis',
                              description='rndis (USB cable to intelligent controller), ap or sta'),
        DeclareLaunchArgument('s1_serial_number', default_value=''),
        chassis, camera_wide, camera_usb, lidar, gnss, *tfs,
    ])

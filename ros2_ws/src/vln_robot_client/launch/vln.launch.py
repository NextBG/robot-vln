"""VLN stack on the robot: safety gate + (optional) LiDAR->depth + HTTP client.

    ros2 launch vln_robot_client vln.launch.py server_url:=http://<gpu-host>:5801/eval_dual
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_depth = LaunchConfiguration('use_depth')
    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://192.168.31.100:5801/eval_dual'),
        DeclareLaunchArgument('use_depth', default_value='true',
                              description='Send LiDAR-projected depth (needed by RGB-D policies, e.g. NavDP S1)'),
        DeclareLaunchArgument('instruction', default_value=''),
        Node(package='vln_robot_client', executable='safety_stop', output='screen'),
        Node(package='vln_robot_client', executable='lidar_depth_projector', output='screen',
             condition=IfCondition(use_depth)),
        Node(package='vln_robot_client', executable='vln_client', output='screen', parameters=[{
            'server_url': LaunchConfiguration('server_url'),
            'use_depth': ParameterValue(use_depth, value_type=bool),
            'instruction': LaunchConfiguration('instruction'),
        }]),
    ])

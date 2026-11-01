"""GPS loop around a route (default: 凯悦嘉轩酒店 -> 滴水湖一圈 -> 凯悦嘉轩酒店).

Real robot (sensors + chassis + LiDAR safety stop + follower):
    ros2 launch vln_robot_nav gps_loop.launch.py
    ros2 service call /nav/start std_srvs/srv/Trigger
Dry run without hardware (simulated GPS/odometry, no wheels move):
    ros2 launch vln_robot_nav gps_loop.launch.py sim:=true autostart:=true
Resume after a battery swap:
    ros2 launch vln_robot_nav gps_loop.launch.py resume:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('vln_robot_nav')
    sim = LaunchConfiguration('sim')
    route = LaunchConfiguration('route_file')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vln_robot_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'use_camera': LaunchConfiguration('use_camera'),
                          'use_lidar': LaunchConfiguration('use_lidar')}.items(),
        condition=UnlessCondition(sim))

    sim_node = Node(package='vln_robot_nav', executable='nav_sim', output='screen',
                    parameters=[{'route_file': route,
                                 'time_scale': ParameterValue(LaunchConfiguration('time_scale'), value_type=float)}],
                    condition=IfCondition(sim))

    safety = Node(
        package='vln_robot_client', executable='safety_stop', output='screen',
        parameters=[{'cmd_in': '/nav/cmd_vel', 'cmd_out': '/cmd_vel',
                     'max_v': ParameterValue(LaunchConfiguration('max_v'), value_type=float),
                     'max_w': 0.8,
                     'require_lidar': ParameterValue(PythonExpression([
                         "'", sim, "' != 'true' and '", LaunchConfiguration('use_lidar'), "' == 'true'"]),
                         value_type=bool)}])

    follower = Node(
        package='vln_robot_nav', executable='gps_waypoint_follower', output='screen',
        parameters=[os.path.join(share, 'config', 'follower.yaml'), {
            'route_file': route,
            'max_v': ParameterValue(LaunchConfiguration('max_v'), value_type=float),
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'resume': ParameterValue(LaunchConfiguration('resume'), value_type=bool),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('time_scale', default_value='1.0', description='sim only'),
        DeclareLaunchArgument('route_file', default_value=os.path.join(share, 'routes', 'dishui_lake.csv')),
        DeclareLaunchArgument('max_v', default_value='0.6'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('resume', default_value='false'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        robot, sim_node, safety, follower,
    ])

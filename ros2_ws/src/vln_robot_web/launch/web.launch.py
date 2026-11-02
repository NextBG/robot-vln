"""Robot drivers + safety_stop + web dashboard (+ on-demand VLN client).

    ros2 launch vln_robot_web web.launch.py                    # everything
    ros2 launch vln_robot_web web.launch.py with_robot:=false  # drivers already running elsewhere
    ros2 launch vln_robot_web web.launch.py \\
        vln_tunnel:="ssh -N -o ServerAliveInterval=15 -o ExitOnForwardFailure=yes -L 5801:127.0.0.1:5801 -p <port> root@<gpu-host>"

With with_robot:=true the web node starts every driver (chassis, camera, lidar, gnss, tf) as its own
child process, so a single driver can be restarted from the page (e.g. after a battery swap).
The VLN client is started on demand when an instruction is sent from the page.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    modules = PythonExpression(["'chassis,camera,lidar,tf' + (',gnss' if '", lc('use_gnss'), "' == 'true' else '')"])
    return LaunchDescription([
        DeclareLaunchArgument('with_robot', default_value='true'),
        DeclareLaunchArgument('use_gnss', default_value='true'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('max_v', default_value='0.3', description='m/s, enforced by safety_stop and web'),
        DeclareLaunchArgument('max_w', default_value='0.8', description='rad/s'),
        DeclareLaunchArgument('vln_server_url', default_value='http://127.0.0.1:5801/eval_dual'),
        DeclareLaunchArgument('vln_tunnel', default_value='',
                              description='optional command that forwards the GPU server to localhost'),
        DeclareLaunchArgument('vln_use_depth', default_value='true'),
        Node(package='vln_robot_client', executable='safety_stop', name='safety_stop', output='screen',
             parameters=[{'max_v': lc('max_v'), 'max_w': lc('max_w')}]),
        Node(package='vln_robot_web', executable='web_node', name='vln_robot_web', output='screen',
             parameters=[{'port': lc('port'), 'max_v': lc('max_v'), 'max_w': lc('max_w'),
                          'drivers': ParameterValue(lc('with_robot'), value_type=bool),
                          'driver_modules': modules,
                          'vln_server_url': lc('vln_server_url'),
                          'vln_tunnel': ParameterValue(lc('vln_tunnel'), value_type=str),
                          'vln_use_depth': ParameterValue(lc('vln_use_depth'), value_type=bool)}]),
    ])

"""
Launch all multiagent_driving nodes for one car.

Usage:
  # Multi-car (default):
  ros2 launch multiagent_driving multiagent.launch.py server_ip:=192.168.1.100 namespace:=/car1

  # Single-car (no websocket/filter/obstacle nodes):
  ros2 launch multiagent_driving multiagent.launch.py mode:=single

  # Server only (just the websocket relay):
  ros2 launch multiagent_driving multiagent.launch.py mode:=server

  # With metrics logging (works in any mode):
  ros2 launch multiagent_driving multiagent.launch.py log_metrics:=true
  ros2 launch multiagent_driving multiagent.launch.py log_metrics:=true \
    metrics_output_file:=/tmp/baseline_run1.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # ---------- arguments ----------
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='multi',
        description='Launch mode: "multi" (all per-car nodes), '
                    '"single" (track_navigator only), '
                    '"server" (websocket_server only)')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip', default_value='localhost',
        description='IP address of the WebSocket server')

    server_port_arg = DeclareLaunchArgument(
        'server_port', default_value='8765',
        description='Port of the WebSocket server')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS namespace for this car (e.g. /car1)')

    car_radius_arg = DeclareLaunchArgument(
        'car_radius', default_value='0.25',
        description='Radius around known cars to filter from lidar')

    log_metrics_arg = DeclareLaunchArgument(
        'log_metrics', default_value='false',
        description='Set to "true" to launch the metrics_logger node')

    metrics_output_file_arg = DeclareLaunchArgument(
        'metrics_output_file', default_value='',
        description='CSV output path for metrics_logger. '
                    'Empty string = auto-generate in /tmp')

    near_miss_threshold_arg = DeclareLaunchArgument(
        'near_miss_threshold', default_value='0.35',
        description='Scan range (m) that triggers a near-miss event in the metrics logger')

    # ---------- substitutions ----------
    mode = LaunchConfiguration('mode')
    is_multi = PythonExpression(["'", mode, "' == 'multi'"])
    is_server = PythonExpression(["'", mode, "' == 'server'"])
    is_not_server = PythonExpression(["'", mode, "' != 'server'"])
    is_logging = LaunchConfiguration('log_metrics')

    # ---------- nodes ----------
    websocket_server_node = Node(
        package='multiagent_driving',
        executable='websocket_server',
        name='websocket_server',
        output='screen',
        condition=IfCondition(is_server),
    )

    car_nodes = GroupAction(
        condition=IfCondition(is_not_server),
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),

            # Always launched (single + multi)
            Node(
                package='multiagent_driving',
                executable='track_navigator',
                name='track_navigator',
                output='screen',
            ),

            # Multi-car only
            Node(
                package='multiagent_driving',
                executable='websocket_client',
                name='websocket_client',
                output='screen',
                parameters=[{
                    'server_ip': LaunchConfiguration('server_ip'),
                    'server_port': LaunchConfiguration('server_port'),
                }],
                condition=IfCondition(is_multi),
            ),
            Node(
                package='multiagent_driving',
                executable='car_filter',
                name='car_filter',
                output='screen',
                parameters=[{
                    'car_radius': LaunchConfiguration('car_radius'),
                }],
                condition=IfCondition(is_multi),
            ),
            Node(
                package='multiagent_driving',
                executable='obstacle_detector',
                name='obstacle_detector',
                output='screen',
                condition=IfCondition(is_multi),
            ),
        ],
    )

    metrics_logger_node = Node(
        package='multiagent_driving',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[{
            'near_miss_threshold': LaunchConfiguration('near_miss_threshold'),
            'output_file': LaunchConfiguration('metrics_output_file'),
        }],
        condition=IfCondition(is_logging),
    )

    return LaunchDescription([
        mode_arg,
        server_ip_arg,
        server_port_arg,
        namespace_arg,
        car_radius_arg,
        log_metrics_arg,
        metrics_output_file_arg,
        near_miss_threshold_arg,
        websocket_server_node,
        car_nodes,
        metrics_logger_node,
    ])

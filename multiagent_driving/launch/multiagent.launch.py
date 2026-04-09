"""
Launch all multiagent_driving nodes for one car.

MODES
-----
  multi            Per-car stack only (navigator + websocket_client + car_filter
                   + obstacle_detector). Use on every driving car when the server
                   runs elsewhere (laptop or another car).

  single           track_navigator only. No websocket stack. Use for single-car
                   baseline experiments.

  server           WebSocket relay server only. No car nodes. Use on a dedicated
                   laptop or coordinator machine.

  server_and_drive WebSocket server + full per-car stack. Use when one car must
                   act as both server and driver (no separate laptop available).

USAGE — 2-car experiment (laptop as server)
-------------------------------------------
  # Laptop:
  ros2 launch multiagent_driving multiagent.launch.py mode:=server

  # Car 1:
  ros2 launch multiagent_driving multiagent.launch.py \
    mode:=multi car_id:=car1 server_ip:=<LAPTOP_IP>

  # Car 2:
  ros2 launch multiagent_driving multiagent.launch.py \
    mode:=multi car_id:=car2 server_ip:=<LAPTOP_IP>

USAGE — no laptop (Car 1 hosts server)
---------------------------------------
  # Car 1 (server + driver):
  ros2 launch multiagent_driving multiagent.launch.py \
    mode:=server_and_drive car_id:=car1

  # Car 2:
  ros2 launch multiagent_driving multiagent.launch.py \
    mode:=multi car_id:=car2 server_ip:=<CAR1_IP>

USAGE — single-car baseline
-----------------------------
  ros2 launch multiagent_driving multiagent.launch.py \
    mode:=single log_metrics:=true metrics_output_file:=/tmp/baseline_run1.csv

USAGE — metrics logging (append to any multi-car command)
----------------------------------------------------------
  log_metrics:=true metrics_output_file:=/tmp/swarm_car1.csv

NOTE: Do NOT use ROS namespaces to distinguish cars. Namespacing causes
relative topic subscriptions (amcl_pose) to resolve incorrectly against
AMCL's absolute /amcl_pose topic. Use car_id instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- arguments ----------
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='multi',
        description='Launch mode: '
                    '"multi" (per-car nodes, server runs elsewhere), '
                    '"single" (track_navigator only, baseline), '
                    '"server" (websocket relay only, no car nodes), '
                    '"server_and_drive" (server + all per-car nodes)')

    car_id_arg = DeclareLaunchArgument(
        'car_id', default_value='car_default',
        description='Unique identifier for this car (e.g. car1, car2). '
                    'Must differ between cars — used by the websocket to filter '
                    'self-messages. Do not use ROS namespaces for this.')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip', default_value='localhost',
        description='IP address of the WebSocket server')

    server_port_arg = DeclareLaunchArgument(
        'server_port', default_value='8765',
        description='Port of the WebSocket server')

    car_margin_arg = DeclareLaunchArgument(
        'car_margin', default_value='0.05',
        description='Margin added to each side of the car OBB for lidar filtering (m)')

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

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='NaN',
        description='Initial pose X for AMCL (NaN = use RViz2 instead)')
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='NaN',
        description='Initial pose Y for AMCL (NaN = use RViz2 instead)')
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='NaN',
        description='Initial pose yaw (rad) for AMCL (NaN = use RViz2 instead)')

    # ---------- substitutions ----------
    mode = LaunchConfiguration('mode')
    # websocket_server runs for "server" and "server_and_drive"
    is_server = PythonExpression(["'", mode, "' in ('server', 'server_and_drive')"])
    # car nodes run for everything except the bare "server" mode
    is_car = PythonExpression(["'", mode, "' != 'server'"])
    # websocket_client/car_filter/obstacle_detector run for "multi" and "server_and_drive"
    is_multi = PythonExpression(["'", mode, "' in ('multi', 'server_and_drive')"])
    is_logging = LaunchConfiguration('log_metrics')

    # ---------- nodes ----------
    websocket_server_node = ExecuteProcess(
        cmd=['ros2', 'run', 'multiagent_driving', 'websocket_server'],
        output='screen',
        condition=IfCondition(is_server),
    )

    car_nodes = GroupAction(
        condition=IfCondition(is_car),
        actions=[
            # Always launched (single + multi).
            # No PushRosNamespace here — namespacing breaks relative topic
            # subscriptions (amcl_pose, follow_waypoints action server).
            Node(
                package='multiagent_driving',
                executable='track_navigator',
                name='track_navigator',
                output='screen',
                parameters=[{
                    'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                    'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                    'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                }],
            ),

            # Multi-car only
            Node(
                package='multiagent_driving',
                executable='websocket_client',
                name='websocket_client',
                output='screen',
                parameters=[{
                    'car_id': LaunchConfiguration('car_id'),
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
                    'car_margin': LaunchConfiguration('car_margin'),
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
        car_id_arg,
        server_ip_arg,
        server_port_arg,
        car_margin_arg,
        log_metrics_arg,
        metrics_output_file_arg,
        near_miss_threshold_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        websocket_server_node,
        car_nodes,
        metrics_logger_node,
    ])

"""
One-command multi-car simulation with costmap visualization.

Usage:
  ros2 launch multiagent_driving sim.launch.py
  ros2 launch multiagent_driving sim.launch.py map:=gdc_3n num_ghost_cars:=3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    maps_dir = os.path.join(
        home_dir, 'roboracer_ws', 'src', 'av_navigation', 'av_navigation', 'maps')
    rviz_dir = os.path.join(home_dir, 'roboracer_ws', 'src', 'multiagent_driving', 'rviz')

    pkg_av_description = get_package_share_directory('av_description')
    urdf_path = os.path.join(pkg_av_description, 'urdf', 'roboracer.urdf.xml')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ---------- arguments ----------
    map_arg = DeclareLaunchArgument(
        'map', default_value='home',
        description='Map name (without .yaml extension)')
    num_ghosts_arg = DeclareLaunchArgument(
        'num_ghost_cars', default_value='2',
        description='Number of simulated ghost cars')
    ghost_speed_arg = DeclareLaunchArgument(
        'ghost_speed', default_value='0.3',
        description='Ghost car speed (m/s)')
    ego_x_arg = DeclareLaunchArgument(
        'ego_start_x', default_value='-0.3',
        description='Ego car start X position')
    ego_y_arg = DeclareLaunchArgument(
        'ego_start_y', default_value='-0.25',
        description='Ego car start Y position')
    ego_yaw_arg = DeclareLaunchArgument(
        'ego_start_yaw', default_value='0.0',
        description='Ego car start yaw (radians)')

    map_name = LaunchConfiguration('map')
    map_yaml = PathJoinSubstitution([maps_dir, [map_name, '.yaml']])

    # ---------- sim_world node ----------
    sim_world_node = Node(
        package='multiagent_driving',
        executable='sim_world',
        name='sim_world',
        output='screen',
        parameters=[{
            'map_yaml': map_yaml,
            'num_ghost_cars': LaunchConfiguration('num_ghost_cars'),
            'ghost_speed': LaunchConfiguration('ghost_speed'),
            'ego_start_x': LaunchConfiguration('ego_start_x'),
            'ego_start_y': LaunchConfiguration('ego_start_y'),
            'ego_start_yaw': LaunchConfiguration('ego_start_yaw'),
        }],
    )

    # ---------- Nav2 stack (map_server, costmap, planner, controller) ----------
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(home_dir, 'roboracer_ws', 'src',
                         'av_navigation', 'av_navigation', 'launch',
                         'navigation.launch.py')
        ),
        launch_arguments={'map': map_name}.items(),
    )

    # ---------- robot_state_publisher (URDF -> TF) ----------
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # ---------- car_filter (removes ghost car points from lidar) ----------
    car_filter_node = Node(
        package='multiagent_driving',
        executable='car_filter',
        name='car_filter',
        output='screen',
    )

    # ---------- track_navigator (drives ego car through waypoints) ----------
    track_nav_node = Node(
        package='multiagent_driving',
        executable='track_navigator',
        name='track_navigator',
        output='screen',
    )

    # ---------- RViz2 ----------
    rviz_config = os.path.join(rviz_dir, 'sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        map_arg,
        num_ghosts_arg,
        ghost_speed_arg,
        ego_x_arg,
        ego_y_arg,
        ego_yaw_arg,
        sim_world_node,
        nav_launch,
        robot_state_pub,
        car_filter_node,
        track_nav_node,
        rviz_node,
    ])

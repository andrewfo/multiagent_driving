import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'multiagent_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Multi-agent autonomous driving for Roboracer',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'track_navigator = multiagent_driving.track_navigator:main',
            'websocket_client = multiagent_driving.websocket_client_node:main',
            'websocket_server = multiagent_driving.websocket:main',
            'set_waypoints = multiagent_driving.set_waypoints:main',
            'car_filter = multiagent_driving.car_filter_node:main',
            'obstacle_detector = multiagent_driving.obstacle_detector_node:main',
            'sim_world = multiagent_driving.sim_world:main',
        ],
    },
)

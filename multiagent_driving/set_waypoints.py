#!/usr/bin/env python3
"""
Set Waypoints - Record waypoints from the car's live pose and write them
into track_navigator.py.

Subscribes to /amcl_pose, samples the position at a fixed interval (default
3 seconds), and when you press Ctrl-C it overwrites TRACK_WAYPOINTS_XY in
track_navigator.py with the recorded coordinates.

Usage:
  ros2 run multiagent_driving set_waypoints              # every 3 seconds
  ros2 run multiagent_driving set_waypoints --interval 2  # every 2 seconds

Prerequisites:
  - Navigation / localization stack running so /amcl_pose is being published
  - Drive the car around the track while this script records
"""

import argparse
import os
import re
import textwrap

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


TRACK_NAV_PATH = os.path.join(
    os.path.dirname(__file__), 'track_navigator.py'
)


class WaypointRecorder(Node):
    def __init__(self, interval: float):
        super().__init__('waypoint_recorder')
        self._interval = interval
        self._waypoints: list[tuple[float, float]] = []
        self._latest_pose = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_cb,
            10,
        )
        self.create_timer(self._interval, self._sample)
        self.get_logger().info(
            f'Recording a waypoint every {self._interval}s. '
            'Drive the car around the track, press Ctrl-C to save.'
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_pose = msg.pose.pose

    def _sample(self):
        if self._latest_pose is None:
            self.get_logger().warn(
                'No pose received yet on /amcl_pose'
            )
            return
        x = self._latest_pose.position.x
        y = self._latest_pose.position.y
        self._waypoints.append((round(x, 2), round(y, 2)))
        self.get_logger().info(
            f'[{len(self._waypoints)}] Recorded waypoint ({x:.2f}, {y:.2f})'
        )

    def save_waypoints(self):
        if not self._waypoints:
            self.get_logger().warn('No waypoints recorded')
            return

        # Build the new Python list literal
        lines = ['TRACK_WAYPOINTS_XY = [']
        for x, y in self._waypoints:
            lines.append(f'    ({x},  {y}),')
        lines.append(']')
        new_block = '\n'.join(lines)

        with open(TRACK_NAV_PATH, 'r') as f:
            source = f.read()

        # Replace the existing TRACK_WAYPOINTS_XY = [...] block
        pattern = r'TRACK_WAYPOINTS_XY\s*=\s*\[.*?\]'
        if not re.search(pattern, source, flags=re.DOTALL):
            self.get_logger().error(
                'Could not find TRACK_WAYPOINTS_XY in track_navigator.py'
            )
            return

        new_source = re.sub(pattern, new_block, source, count=1, flags=re.DOTALL)

        with open(TRACK_NAV_PATH, 'w') as f:
            f.write(new_source)

        self.get_logger().info(
            f'Saved {len(self._waypoints)} waypoints to {TRACK_NAV_PATH}'
        )
        # Also print them so the user can copy/paste if needed
        self.get_logger().info(f'\n{new_block}')


def main(args=None):
    parser = argparse.ArgumentParser(description='Record track waypoints')
    parser.add_argument(
        '--interval', type=float, default=3.0,
        help='Seconds between waypoint samples (default: 3)',
    )
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = WaypointRecorder(parsed.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping recording...')
    finally:
        node.save_waypoints()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

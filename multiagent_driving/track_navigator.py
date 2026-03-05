#!/usr/bin/env python3
"""
Track Navigator - Autonomous lap driving with Nav2 FollowWaypoints.

Sends a loop of PoseStamped waypoints to Nav2's waypoint follower, making the
car drive laps around a track continuously.

Prerequisites:
  1. Navigation stack running:
       ros2 launch av_navigation navigation.launch.py map:=<your_map>
  2. Initial pose set in RViz2 (click "2D Pose Estimate" and drag on map)
  3. R1 held on joystick to enable motion (or joystick in autonomous mode)

Run:
  ros2 run team_tutorial track_navigator
  OR
  python3 track_navigator.py

How to get your waypoint coordinates:
  - Drive the car manually around the track
  - Record poses: ros2 topic echo /amcl_pose
  - Copy x, y from pose.pose.position; compute yaw from pose.pose.orientation
  - OR use RViz2 "Publish Point" tool and click on the map to read coordinates
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


# ---------------------------------------------------------------------------
# Waypoints: (x, y) in the map frame.
# Yaw is auto-computed as the direction toward the next waypoint.

# ---------------------------------------------------------------------------
# Tarun test
TRACK_WAYPOINTS_XY = [
    (-0.3,  -.25),
    (1.56,  .09),
    (3.67,  -.39),
    (5.24,  -1.7),
    (5.24,  -2.97),
    (4.84,  -3.53),
    (4.27,  -3.66),
    (2.60,  -3.07),
    (1.68,  -2.60),
    (.69,  -2.04),
    (-.79,  .5),
    (-.29,  -1.63),
]

# Number of laps. Set to None to loop forever.
NUM_LAPS = None

# Speed is controlled by nav2.yaml (vx_max: 0.5 m/s by default).
# To increase speed, edit: src/av_navigation/av_navigation/config/nav2.yaml -> vx_max


def yaw_to_quaternion(yaw: float):
    """Convert a yaw angle (radians) to a geometry_msgs quaternion (x,y,z,w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def compute_yaw(x0, y0, x1, y1) -> float:
    """Compute the yaw angle pointing from (x0,y0) toward (x1,y1)."""
    return math.atan2(y1 - y0, x1 - x0)


def build_poses(waypoints_xy, node: Node) -> list:
    """
    Build a list of PoseStamped messages from (x, y) pairs.
    Yaw at each waypoint points toward the next waypoint (wraps around).
    """
    n = len(waypoints_xy)
    poses = []
    for i, (x, y) in enumerate(waypoints_xy):
        # Point toward the next waypoint (wrap around for the last one)
        nx, ny = waypoints_xy[(i + 1) % n]
        yaw = compute_yaw(x, y, nx, ny)

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = node.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        poses.append(p)
    return poses


class TrackNavigator(Node):
    def __init__(self):
        super().__init__('track_navigator')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def run(self):
        self.get_logger().info('Waiting for Nav2 FollowWaypoints server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server ready. Starting autonomous laps.')

        lap = 0
        while rclpy.ok():
            if NUM_LAPS is not None and lap >= NUM_LAPS:
                self.get_logger().info(f'Completed {NUM_LAPS} lap(s). Done.')
                break

            lap += 1
            self.get_logger().info(f'--- Lap {lap} ---')

            poses = build_poses(TRACK_WAYPOINTS_XY, self)
            goal = FollowWaypoints.Goal()
            goal.poses = poses

            send_future = self._action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_cb,
            )
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if not goal_handle.accepted:
                self.get_logger().error(
                    'Goal rejected by Nav2. Is the navigation stack running '
                    'and is the initial pose set?'
                )
                break

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            missed = list(result.missed_waypoints)
            if missed:
                self.get_logger().warn(f'Lap {lap} done. Missed waypoints: {missed}')
            else:
                self.get_logger().info(f'Lap {lap} complete! All waypoints reached.')

    def _feedback_cb(self, feedback_msg):
        wp = feedback_msg.feedback.current_waypoint
        total = len(TRACK_WAYPOINTS_XY)
        self.get_logger().info(
            f'  -> waypoint {wp + 1}/{total}',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Navigation interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

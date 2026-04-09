#!/usr/bin/env python3
"""
Track Navigator - Autonomous lap driving with Nav2 FollowWaypoints.

Sends a loop of PoseStamped waypoints to Nav2's waypoint follower, making the
car drive laps around a track continuously.

Prerequisites:
  1. Navigation stack running:
       ros2 launch av_navigation navigation.launch.py map:=<your_map>
  2. Initial pose set via EITHER:
       a. Launch parameters: initial_pose_x, initial_pose_y, initial_pose_yaw
       b. RViz2: click "2D Pose Estimate" and drag on map
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
# These are actually run on the robot itself so don't worry if they're not resolved
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints


# ---------------------------------------------------------------------------
# Waypoints: (x, y) in the map frame.
# Yaw is auto-computed as the direction toward the next waypoint.
# Jacob test
# ---------------------------------------------------------------------------
# Tarun test
# Aranya test1
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

        # Optional programmatic initial pose (NaN = not set, use RViz2 instead)
        self.declare_parameter('initial_pose_x', float('nan'))
        self.declare_parameter('initial_pose_y', float('nan'))
        self.declare_parameter('initial_pose_yaw', float('nan'))
        self.declare_parameter('amcl_timeout', 30.0)

        # AMCL readiness tracking
        self._amcl_pose_received = False
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, 10)

        # Publisher for programmatic initial pose
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # TF listener for diagnostics
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    # ------------------------------------------------------------------
    # AMCL readiness helpers
    # ------------------------------------------------------------------

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        """Record that at least one valid AMCL pose has been received."""
        if not self._amcl_pose_received:
            self.get_logger().info(
                f'AMCL pose received: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f}). Localization is active.')
        self._amcl_pose_received = True

    def _publish_initial_pose(self):
        """If initial_pose params are set, publish to /initialpose for AMCL."""
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        yaw = self.get_parameter('initial_pose_yaw').value

        if math.isnan(x) or math.isnan(y) or math.isnan(yaw):
            self.get_logger().info(
                'No initial_pose parameters set. '
                'Waiting for initial pose from RViz2 or another source.')
            return

        self.get_logger().info(
            f'Publishing initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25    # x variance
        msg.pose.covariance[7] = 0.25    # y variance
        msg.pose.covariance[35] = 0.068  # yaw variance
        self._initial_pose_pub.publish(msg)

    def _wait_for_amcl(self) -> bool:
        """Block until AMCL publishes at least one pose, with timeout."""
        timeout = self.get_parameter('amcl_timeout').value
        self.get_logger().info(
            f'Waiting for AMCL pose on /amcl_pose (timeout: {timeout:.0f}s)...')
        start = time.time()
        republish_interval = 5.0
        last_republish = start
        while rclpy.ok() and not self._amcl_pose_received:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - start
            if elapsed >= timeout:
                self.get_logger().error(
                    f'Timed out after {timeout:.0f}s waiting for AMCL pose. '
                    'Check that: (1) the navigation stack is running '
                    '(ros2 launch av_navigation navigation.launch.py), '
                    '(2) the initial pose has been set in RViz2, or '
                    'pass initial_pose_x/y/yaw parameters.')
                return False
            if time.time() - last_republish >= republish_interval:
                self._publish_initial_pose()
                self._log_diagnostic_info()
                last_republish = time.time()
        return True

    def _log_diagnostic_info(self):
        """Log diagnostic info about Nav2 readiness state."""
        try:
            self._tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            tf_ok = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            tf_ok = False
        try:
            self._tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            odom_ok = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            odom_ok = False
        self.get_logger().warn(
            f'Nav2 readiness: '
            f'AMCL pose: {"received" if self._amcl_pose_received else "NOT received"}, '
            f'map->odom TF: {"OK" if tf_ok else "MISSING"}, '
            f'odom->base_link TF: {"OK" if odom_ok else "MISSING"}')

    # ------------------------------------------------------------------

    def run(self):
        # 1. Optionally publish programmatic initial pose
        self._publish_initial_pose()

        # 2. Wait for AMCL to confirm localization is active
        if not self._wait_for_amcl():
            self.get_logger().error('Cannot proceed without AMCL localization. Exiting.')
            return

        # 3. Wait for the FollowWaypoints action server
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
                self._log_diagnostic_info()
                self.get_logger().warn(
                    'Goal rejected by Nav2 — retrying in 3s. '
                    'See diagnostic info above for details.')
                time.sleep(3.0)
                continue

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

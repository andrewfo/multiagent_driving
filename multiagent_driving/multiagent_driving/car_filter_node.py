#!/usr/bin/env python3
"""
Filters laser scan points that correspond to known car positions received
via /swarm_poses, preventing Nav2 obstacle layers from double-counting
cars that are already tracked by the SwarmLayer costmap plugin.

Subscriptions
-------------
/scan          (LaserScan)  – raw lidar data
/swarm_poses   (PoseArray)  – other cars' poses from websocket client
amcl_pose      (PoseWithCovarianceStamped) – this car's localisation

Publications
------------
/scan_filtered (LaserScan)  – scan with car-occupied points removed

Parameters
----------
car_margin : float – buffer added to each side of the car OBB (default: 0.05 m)
            Effective filter box is (0.3 + 2×margin) × (0.09 + 2×margin).
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import tf2_ros



# Real car footprint half-extents (matches nav2.yaml)
_CAR_HALF_LEN = 0.15   # 0.3 m / 2
_CAR_HALF_WID = 0.045  # 0.09 m / 2


def _point_in_obb(px: float, py: float,
                  cx: float, cy: float, yaw: float,
                  half_len: float, half_wid: float) -> bool:
    """Return True if world-frame point (px, py) is inside the oriented bounding box."""
    dx = px - cx
    dy = py - cy
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return abs(local_x) <= half_len and abs(local_y) <= half_wid


class CarFilterNode(Node):
    def __init__(self):
        super().__init__("car_filter_node")

        self.declare_parameter("car_margin", 0.05)
        self.car_margin = self.get_parameter("car_margin").value

        # --- State --------------------------------------------------------
        self._lock = threading.Lock()
        self._car_poses: list[tuple[float, float, float]] = []  # (x, y, yaw) map frame
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_pose = False
        self._last_poses_time: float = 0.0   # monotonic time of last /swarm_poses msg
        self._scans_filtered = 0             # points suppressed in the last report window
        self._scans_processed = 0            # scans processed in the last report window

        # --- TF -----------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Subscribers --------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, sensor_qos
        )
        self.create_subscription(
            PoseArray, "/swarm_poses", self._swarm_cb, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._pose_cb, 10
        )

        # --- Publisher ----------------------------------------------------
        self.scan_pub = self.create_publisher(LaserScan, "/scan_filtered", sensor_qos)

        # --- Diagnostic timer (every 5 s) ---------------------------------
        self.create_timer(5.0, self._diagnostic_cb)

        self.get_logger().info(
            f"Car filter active — suppressing OBB scan points "
            f"(margin={self.car_margin:.3f} m)"
        )

    # ---------------------------------------------------------- callbacks

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        """Cache this car's pose for scan-to-map projection."""
        p = msg.pose.pose
        yaw = _quaternion_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w,
        )
        with self._lock:
            self._robot_x = p.position.x
            self._robot_y = p.position.y
            self._robot_yaw = yaw
            self._have_pose = True

    def _swarm_cb(self, msg: PoseArray):
        """Cache known car poses (x, y, yaw) in map frame."""
        poses = [
            (p.position.x, p.position.y,
             _quaternion_to_yaw(p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w))
            for p in msg.poses
        ]
        with self._lock:
            self._car_poses = poses
            self._last_poses_time = time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        """Filter scan points near known cars and republish."""
        with self._lock:
            cars = self._car_poses
            if not cars or not self._have_pose:
                # Nothing to filter — pass through unchanged
                self._scans_processed += 1
                self.scan_pub.publish(msg)
                return
            robot_x = self._robot_x
            robot_y = self._robot_y
            robot_yaw = self._robot_yaw

        # OBB half-extents with configured margin on each side
        hl = _CAR_HALF_LEN + self.car_margin
        hw = _CAR_HALF_WID + self.car_margin
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        filtered_ranges = list(msg.ranges)
        angle = msg.angle_min
        points_suppressed = 0

        for i in range(len(filtered_ranges)):
            r = filtered_ranges[i]
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # Project scan point into map frame
            local_x = r * math.cos(angle)
            local_y = r * math.sin(angle)
            map_x = robot_x + cos_yaw * local_x - sin_yaw * local_y
            map_y = robot_y + sin_yaw * local_x + cos_yaw * local_y

            # Check against each known car OBB (x, y, yaw)
            for cx, cy, cyaw in cars:
                if _point_in_obb(map_x, map_y, cx, cy, cyaw, hl, hw):
                    # Replace with max range so obstacle layer ignores it
                    # (raytracing still clears the space behind)
                    filtered_ranges[i] = msg.range_max + 1.0
                    points_suppressed += 1
                    break

            angle += msg.angle_increment

        with self._lock:
            self._scans_filtered += points_suppressed
            self._scans_processed += 1

        # Publish filtered scan with same header/metadata
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered_ranges
        out.intensities = list(msg.intensities) if msg.intensities else []
        self.scan_pub.publish(out)


    def _diagnostic_cb(self):
        """Periodic health check: warn if swarm_poses is absent or stale."""
        with self._lock:
            cars = list(self._car_poses)
            last_t = self._last_poses_time
            suppressed = self._scans_filtered
            processed = self._scans_processed
            self._scans_filtered = 0
            self._scans_processed = 0

        age = time.monotonic() - last_t if last_t > 0.0 else float("inf")

        if last_t == 0.0:
            self.get_logger().warn(
                "No /swarm_poses received yet — scan is passing through unfiltered. "
                "Check that websocket_client_node is running and connected."
            )
        elif age > 3.0:
            self.get_logger().warn(
                f"/swarm_poses last received {age:.1f}s ago (>3s stale) — "
                "filtering is disabled. WebSocket may be disconnected."
            )
        else:
            self.get_logger().info(
                f"car_filter OK | neighbors={len(cars)} | "
                f"scans={processed} | points_suppressed={suppressed}"
            )


def _quaternion_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = CarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

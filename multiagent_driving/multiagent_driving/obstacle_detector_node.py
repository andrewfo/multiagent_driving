#!/usr/bin/env python3
"""
Detects obstacles from filtered lidar scans by clustering nearby points
and publishing cluster centroids as a PoseArray on /obstacles.

This is the missing "faucet" that feeds the existing websocket pipeline:
  /obstacles → websocket_client_node → websocket server → other cars'
  /swarm_obstacles → SwarmLayer → costmap lethal cells

Subscriptions
-------------
/scan_filtered  (LaserScan)                – lidar with car points removed
amcl_pose       (PoseWithCovarianceStamped) – this car's localisation

Publications
------------
/obstacles      (PoseArray)  – centroid of each detected obstacle cluster
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")

        # Parameters
        self.declare_parameter("cluster_distance", 0.3)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("publish_rate", 10.0)

        self.cluster_distance = self.get_parameter("cluster_distance").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value
        publish_rate = self.get_parameter("publish_rate").value

        # State
        self._lock = threading.Lock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_pose = False
        self._latest_scan: LaserScan | None = None

        # Subscribers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(
            LaserScan, "/scan_filtered", self._scan_cb, sensor_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._pose_cb, 10
        )

        # Publisher
        self.obstacle_pub = self.create_publisher(PoseArray, "/obstacles", 10)

        # Timer-driven publish to throttle output
        period = 1.0 / publish_rate
        self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f"Obstacle detector active — cluster_distance={self.cluster_distance:.2f}m, "
            f"min_cluster_size={self.min_cluster_size}, rate={publish_rate:.1f}Hz"
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
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

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._latest_scan = msg

    def _timer_cb(self):
        with self._lock:
            scan = self._latest_scan
            if scan is None or not self._have_pose:
                return
            robot_x = self._robot_x
            robot_y = self._robot_y
            robot_yaw = self._robot_yaw
            self._latest_scan = None  # consume it

        # Convert scan to map-frame XY points
        points = _scan_to_map_points(scan, robot_x, robot_y, robot_yaw)
        if not points:
            return

        # Cluster and compute centroids
        clusters = _cluster_points(points, self.cluster_distance)
        centroids = [
            _centroid(c) for c in clusters if len(c) >= self.min_cluster_size
        ]

        if not centroids:
            return

        # Publish
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for cx, cy in centroids:
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.obstacle_pub.publish(msg)


def _quaternion_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _scan_to_map_points(
    scan: LaserScan, robot_x: float, robot_y: float, robot_yaw: float
) -> list[tuple[float, float]]:
    """Convert valid scan ranges to (x, y) points in map frame."""
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    points = []
    angle = scan.angle_min

    for r in scan.ranges:
        if not math.isinf(r) and not math.isnan(r) and scan.range_min <= r <= scan.range_max:
            local_x = r * math.cos(angle)
            local_y = r * math.sin(angle)
            map_x = robot_x + cos_yaw * local_x - sin_yaw * local_y
            map_y = robot_y + sin_yaw * local_x + cos_yaw * local_y
            points.append((map_x, map_y))
        angle += scan.angle_increment

    return points


def _cluster_points(
    points: list[tuple[float, float]], threshold: float
) -> list[list[tuple[float, float]]]:
    """Simple single-linkage clustering by Euclidean distance."""
    if not points:
        return []

    threshold_sq = threshold * threshold
    used = [False] * len(points)
    clusters = []

    for i in range(len(points)):
        if used[i]:
            continue
        # BFS to find all connected points
        cluster = [points[i]]
        used[i] = True
        queue = [i]
        while queue:
            curr = queue.pop(0)
            cx, cy = points[curr]
            for j in range(len(points)):
                if used[j]:
                    continue
                dx = points[j][0] - cx
                dy = points[j][1] - cy
                if dx * dx + dy * dy <= threshold_sq:
                    used[j] = True
                    cluster.append(points[j])
                    queue.append(j)
        clusters.append(cluster)

    return clusters


def _centroid(points: list[tuple[float, float]]) -> tuple[float, float]:
    n = len(points)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    return (sx / n, sy / n)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

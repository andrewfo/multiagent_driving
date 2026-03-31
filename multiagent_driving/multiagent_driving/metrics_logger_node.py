#!/usr/bin/env python3
"""
Logs collision-avoidance experiment metrics to a CSV file.

Detects two classes of safety events from the raw lidar scan and tracks lap
timing from AMCL pose, writing every event to disk in real time.

Subscriptions
-------------
/scan      (LaserScan)                  – raw lidar (before car_filter)
amcl_pose  (PoseWithCovarianceStamped)  – this car's localisation

Publications
------------
(none — write-only logger)

Parameters
----------
near_miss_threshold  : float – min scan range triggering a near-miss  (default: 0.35 m)
collision_threshold  : float – min scan range triggering a collision proxy (default: 0.20 m)
lap_start_x          : float – x coord of lap-start trigger point (default: -0.3)
lap_start_y          : float – y coord of lap-start trigger point (default: -0.25)
lap_trigger_radius   : float – must come within this distance to count a lap (default: 0.5 m)
output_file          : str   – CSV path; empty string → auto-generate in /tmp

CSV columns
-----------
timestamp_sec, event_type, value, car_x, car_y

event_type values
-----------------
  near_miss       – value = minimum scan range (m)
  collision_proxy – value = minimum scan range (m); subset of near_miss events
  lap_start       – value = lap number
  lap_complete    – value = lap duration in seconds
"""

import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger_node")

        # --- Parameters ---------------------------------------------------
        self.declare_parameter("near_miss_threshold", 0.35)
        self.declare_parameter("collision_threshold", 0.20)
        self.declare_parameter("lap_start_x", -0.3)
        self.declare_parameter("lap_start_y", -0.25)
        self.declare_parameter("lap_trigger_radius", 0.5)
        self.declare_parameter("output_file", "")

        self._near_miss_thresh = self.get_parameter("near_miss_threshold").value
        self._collision_thresh = self.get_parameter("collision_threshold").value
        self._lap_x = self.get_parameter("lap_start_x").value
        self._lap_y = self.get_parameter("lap_start_y").value
        self._lap_radius_sq = self.get_parameter("lap_trigger_radius").value ** 2
        output_file = self.get_parameter("output_file").value

        # --- CSV setup ----------------------------------------------------
        if not output_file:
            ts = time.strftime("%Y%m%d_%H%M%S")
            output_file = f"/tmp/roboracer_metrics_{ts}.csv"

        self._csv_path = output_file
        self._csv_file = open(output_file, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ["timestamp_sec", "event_type", "value", "car_x", "car_y"]
        )
        self._csv_file.flush()

        # --- State --------------------------------------------------------
        self._lock = threading.Lock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._have_pose = False

        # Lap tracking
        self._lap_count = 0
        self._lap_start_wall: float | None = None  # wall time of current lap start
        self._in_trigger_zone = False               # debounce: True while inside radius

        # Event counters (reset each status period)
        self._near_miss_count = 0
        self._collision_count = 0

        # --- QoS ----------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # --- Subscribers --------------------------------------------------
        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._pose_cb, 10
        )

        # --- Status timer -------------------------------------------------
        self.create_timer(10.0, self._status_cb)

        self.get_logger().info(
            f"Metrics logger started — writing to {self._csv_path}\n"
            f"  near_miss_threshold={self._near_miss_thresh:.2f}m  "
            f"collision_threshold={self._collision_thresh:.2f}m  "
            f"lap_trigger_radius={self.get_parameter('lap_trigger_radius').value:.2f}m"
        )

    # ---------------------------------------------------------------- callbacks

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        with self._lock:
            self._robot_x = p.position.x
            self._robot_y = p.position.y
            self._have_pose = True

        self._check_lap_trigger(p.position.x, p.position.y)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            if not self._have_pose:
                return
            car_x = self._robot_x
            car_y = self._robot_y

        # Find minimum valid range in this scan
        min_range = float("inf")
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r):
                if msg.range_min <= r <= msg.range_max:
                    if r < min_range:
                        min_range = r

        if math.isinf(min_range):
            return  # no valid returns

        now = self.get_clock().now().nanoseconds * 1e-9

        if min_range <= self._collision_thresh:
            self._log_event(now, "collision_proxy", min_range, car_x, car_y)
            with self._lock:
                self._collision_count += 1
            with self._lock:
                self._near_miss_count += 1
        elif min_range <= self._near_miss_thresh:
            self._log_event(now, "near_miss", min_range, car_x, car_y)
            with self._lock:
                self._near_miss_count += 1

    def _check_lap_trigger(self, x: float, y: float):
        """Detect entry/exit of the lap-start zone and record lap events."""
        dx = x - self._lap_x
        dy = y - self._lap_y
        in_zone = (dx * dx + dy * dy) <= self._lap_radius_sq

        now_wall = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9

        with self._lock:
            was_in_zone = self._in_trigger_zone
            self._in_trigger_zone = in_zone

        if in_zone and not was_in_zone:
            # Crossed into lap-start zone
            with self._lock:
                self._lap_count += 1
                lap_num = self._lap_count
                prev_start = self._lap_start_wall
                self._lap_start_wall = now_wall

            self._log_event(now_ros, "lap_start", float(lap_num), x, y)
            self.get_logger().info(f"Lap {lap_num} started")

            if prev_start is not None and lap_num > 1:
                duration = now_wall - prev_start
                self._log_event(now_ros, "lap_complete", round(duration, 3), x, y)
                self.get_logger().info(
                    f"Lap {lap_num - 1} complete — {duration:.2f}s"
                )

    def _log_event(
        self, timestamp: float, event_type: str, value: float, x: float, y: float
    ):
        """Write one row to the CSV file (thread-safe via GIL on list append)."""
        self._csv_writer.writerow(
            [f"{timestamp:.3f}", event_type, f"{value:.4f}",
             f"{x:.3f}", f"{y:.3f}"]
        )
        self._csv_file.flush()

    def _status_cb(self):
        with self._lock:
            laps = self._lap_count
            near_misses = self._near_miss_count
            collisions = self._collision_count
            self._near_miss_count = 0
            self._collision_count = 0

        self.get_logger().info(
            f"[metrics] laps={laps}  near_misses(10s)={near_misses}  "
            f"collision_proxies(10s)={collisions}  csv={self._csv_path}"
        )

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

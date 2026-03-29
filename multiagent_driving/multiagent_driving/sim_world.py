#!/usr/bin/env python3
"""
Lightweight 2D simulation node for multi-car costmap visualization.

Replaces the physical car + Unity simulator + websocket pipeline.
Publishes fake sensor data so Nav2, SwarmLayer, car_filter, and
track_navigator run unmodified on top.

Usage (via launch file):
    ros2 launch multiagent_driving sim.launch.py map:=home
"""

import math
import os

import numpy as np
from PIL import Image
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    PointStamped,
    Quaternion,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros


# ---------------------------------------------------------------------------
# Waypoints (copied from track_navigator.py so sim is self-contained)
# ---------------------------------------------------------------------------
TRACK_WAYPOINTS_XY = [
    (-0.3, -0.25),
    (1.56, 0.09),
    (3.67, -0.39),
    (5.24, -1.7),
    (5.24, -2.97),
    (4.84, -3.53),
    (4.27, -3.66),
    (2.60, -3.07),
    (1.68, -2.60),
    (0.69, -2.04),
    (-0.79, 0.5),
    (-0.29, -1.63),
]


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _quat_to_yaw(q: Quaternion) -> float:
    return 2.0 * math.atan2(q.z, q.w)


class GhostCar:
    """A simulated car that follows waypoints via linear interpolation."""

    def __init__(self, waypoints, speed, start_index):
        self.waypoints = waypoints
        self.speed = speed
        self.n = len(waypoints)
        # Position along the segment between waypoints
        self.seg_idx = start_index % self.n
        self.seg_t = 0.0  # 0..1 progress along current segment
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self._update_pose()

    def _update_pose(self):
        ax, ay = self.waypoints[self.seg_idx]
        bx, by = self.waypoints[(self.seg_idx + 1) % self.n]
        self.x = ax + (bx - ax) * self.seg_t
        self.y = ay + (by - ay) * self.seg_t
        self.yaw = math.atan2(by - ay, bx - ax)

    def step(self, dt):
        ax, ay = self.waypoints[self.seg_idx]
        bx, by = self.waypoints[(self.seg_idx + 1) % self.n]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len < 1e-6:
            self.seg_idx = (self.seg_idx + 1) % self.n
            self.seg_t = 0.0
        else:
            self.seg_t += (self.speed * dt) / seg_len
            while self.seg_t >= 1.0:
                self.seg_t -= 1.0
                self.seg_idx = (self.seg_idx + 1) % self.n
                ax, ay = self.waypoints[self.seg_idx]
                bx, by = self.waypoints[(self.seg_idx + 1) % self.n]
                seg_len = math.hypot(bx - ax, by - ay)
                if seg_len < 1e-6:
                    self.seg_t = 0.0
                    break
        self._update_pose()


class SimWorld(Node):
    def __init__(self):
        super().__init__('sim_world')

        # ---------- parameters ----------
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('num_ghost_cars', 2)
        self.declare_parameter('ghost_speed', 0.3)
        self.declare_parameter('ego_start_x', TRACK_WAYPOINTS_XY[0][0])
        self.declare_parameter('ego_start_y', TRACK_WAYPOINTS_XY[0][1])
        self.declare_parameter('ego_start_yaw', 0.0)
        self.declare_parameter('lidar_rate', 10.0)
        self.declare_parameter('odom_rate', 20.0)

        map_yaml_path = self.get_parameter('map_yaml').value
        num_ghosts = self.get_parameter('num_ghost_cars').value
        ghost_speed = self.get_parameter('ghost_speed').value
        self.ego_x = self.get_parameter('ego_start_x').value
        self.ego_y = self.get_parameter('ego_start_y').value
        self.ego_yaw = self.get_parameter('ego_start_yaw').value
        lidar_rate = self.get_parameter('lidar_rate').value
        odom_rate = self.get_parameter('odom_rate').value

        # ---------- load occupancy grid ----------
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        if map_yaml_path and os.path.isfile(map_yaml_path):
            self._load_map(map_yaml_path)
        else:
            self.get_logger().warn(f'Map YAML not found: {map_yaml_path}. Lidar will return max range.')

        # ---------- ego car state ----------
        self.wheelbase = 0.324
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        # ---------- ghost cars ----------
        n_wp = len(TRACK_WAYPOINTS_XY)
        self.ghosts = []
        for i in range(num_ghosts):
            start_idx = int((i + 1) * n_wp / (num_ghosts + 1))
            self.ghosts.append(GhostCar(TRACK_WAYPOINTS_XY, ghost_speed, start_idx))

        # ---------- injected obstacles ----------
        self.obstacles = []  # list of (x, y)

        # ---------- TF broadcaster ----------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- publishers ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.swarm_pub = self.create_publisher(PoseArray, '/swarm_poses', 10)
        self.obs_pub = self.create_publisher(PoseArray, '/swarm_obstacles', 10)

        # Publish the map as OccupancyGrid so nav2 map_server isn't strictly required
        # for standalone testing (though the launch file starts map_server anyway).
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # ---------- subscribers ----------
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked_cb, 10)

        # ---------- services ----------
        self.create_service(Empty, '/clear_obstacles', self._clear_obs_cb)

        # ---------- timers ----------
        self.create_timer(1.0 / odom_rate, self._odom_tick)
        self.create_timer(1.0 / lidar_rate, self._lidar_tick)
        self.create_timer(0.1, self._swarm_tick)  # 10 Hz

        # Publish the map once after a short delay so subscribers are ready
        self.create_timer(1.0, self._publish_map_once)
        self._map_published = False

        self.get_logger().info(
            f'SimWorld ready: {num_ghosts} ghost cars, '
            f'ego at ({self.ego_x:.2f}, {self.ego_y:.2f})')

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------
    def _load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        pgm_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.map_resolution = float(meta['resolution'])
        origin = meta['origin']
        self.map_origin_x = float(origin[0])
        self.map_origin_y = float(origin[1])
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))

        img = np.array(Image.open(pgm_path).convert('L'))
        # PGM: 255=free, 0=occupied.  Flip Y so row 0 = bottom (map origin).
        img = np.flipud(img)
        self.map_height, self.map_width = img.shape
        # Convert to boolean: True = occupied
        self.map_data = (img / 255.0) < (1.0 - occupied_thresh)
        self.get_logger().info(
            f'Map loaded: {self.map_width}x{self.map_height}, '
            f'res={self.map_resolution}m')

    def _publish_map_once(self):
        if self._map_published or self.map_data is None:
            return
        self._map_published = True
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info = MapMetaData()
        og.info.resolution = self.map_resolution
        og.info.width = self.map_width
        og.info.height = self.map_height
        og.info.origin.position.x = self.map_origin_x
        og.info.origin.position.y = self.map_origin_y
        # Convert boolean occupied grid to int8 occupancy values
        occ = np.where(self.map_data, 100, 0).astype(np.int8).flatten()
        og.data = occ.tolist()
        self.map_pub.publish(og)
        self.get_logger().info('Published OccupancyGrid on /map')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def _clicked_cb(self, msg: PointStamped):
        px, py = msg.point.x, msg.point.y
        # If clicking near an existing obstacle, remove it
        for i, (ox, oy) in enumerate(self.obstacles):
            if math.hypot(px - ox, py - oy) < 0.3:
                self.obstacles.pop(i)
                self.get_logger().info(f'Removed obstacle at ({ox:.2f}, {oy:.2f})')
                return
        self.obstacles.append((px, py))
        self.get_logger().info(
            f'Added obstacle at ({px:.2f}, {py:.2f}) '
            f'[{len(self.obstacles)} total]')

    def _clear_obs_cb(self, request, response):
        n = len(self.obstacles)
        self.obstacles.clear()
        self.get_logger().info(f'Cleared {n} obstacles')
        return response

    # ------------------------------------------------------------------
    # Ego car kinematics (differential drive approximation of cmd_vel)
    # ------------------------------------------------------------------
    def _odom_tick(self):
        dt = 1.0 / self.get_parameter('odom_rate').value
        # Simple unicycle model (matches how Nav2 outputs cmd_vel)
        self.ego_yaw += self.cmd_wz * dt
        self.ego_x += self.cmd_vx * math.cos(self.ego_yaw) * dt
        self.ego_y += self.cmd_vx * math.sin(self.ego_yaw) * dt

        now = self.get_clock().now().to_msg()
        q = _yaw_to_quat(self.ego_yaw)

        # --- TF: map -> odom (identity) ---
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map_odom)

        # --- TF: odom -> base_link ---
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = self.ego_x
        t_odom_base.transform.translation.y = self.ego_y
        t_odom_base.transform.rotation = q
        self.tf_broadcaster.sendTransform(t_odom_base)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.ego_x
        odom.pose.pose.position.y = self.ego_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.cmd_vx
        odom.twist.twist.angular.z = self.cmd_wz
        self.odom_pub.publish(odom)

        # --- AMCL pose (perfect localization) ---
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp = now
        amcl.header.frame_id = 'map'
        amcl.pose.pose.position.x = self.ego_x
        amcl.pose.pose.position.y = self.ego_y
        amcl.pose.pose.orientation = q
        self.amcl_pub.publish(amcl)

    # ------------------------------------------------------------------
    # Ghost cars + obstacle publishing
    # ------------------------------------------------------------------
    def _swarm_tick(self):
        dt = 0.1  # 10 Hz
        now = self.get_clock().now().to_msg()

        # Step ghost cars
        for g in self.ghosts:
            g.step(dt)

        # Publish /swarm_poses
        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = 'map'
        for g in self.ghosts:
            p = Pose()
            p.position.x = g.x
            p.position.y = g.y
            p.orientation = _yaw_to_quat(g.yaw)
            pa.poses.append(p)
        self.swarm_pub.publish(pa)

        # Publish /swarm_obstacles
        oa = PoseArray()
        oa.header.stamp = now
        oa.header.frame_id = 'map'
        for ox, oy in self.obstacles:
            p = Pose()
            p.position.x = ox
            p.position.y = oy
            p.orientation.w = 1.0
            oa.poses.append(p)
        self.obs_pub.publish(oa)

    # ------------------------------------------------------------------
    # Lidar raycasting
    # ------------------------------------------------------------------
    def _lidar_tick(self):
        now = self.get_clock().now().to_msg()

        # Hokuyo UST-10LX parameters
        angle_min = -2.356194  # -135 deg
        angle_max = 2.356194   #  135 deg
        num_rays = 1080
        angle_inc = (angle_max - angle_min) / num_rays
        range_min = 0.06
        range_max = 10.0
        step_size = self.map_resolution  # march in map-cell steps

        ranges = []
        for i in range(num_rays):
            angle = angle_min + i * angle_inc
            ray_angle = self.ego_yaw + angle
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)
            r = range_min
            hit = False
            while r <= range_max:
                px = self.ego_x + r * cos_a
                py = self.ego_y + r * sin_a
                # Check map occupancy
                if self.map_data is not None:
                    mx = int((px - self.map_origin_x) / self.map_resolution)
                    my = int((py - self.map_origin_y) / self.map_resolution)
                    if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                        if self.map_data[my, mx]:
                            hit = True
                            break
                    elif r > range_min + step_size:
                        # Outside map bounds
                        break
                # Check ghost car circles
                for g in self.ghosts:
                    if math.hypot(px - g.x, py - g.y) < 0.15:
                        hit = True
                        break
                # Check injected obstacles
                if not hit:
                    for ox, oy in self.obstacles:
                        if math.hypot(px - ox, py - oy) < 0.15:
                            hit = True
                            break
                if hit:
                    break
                r += step_size
            ranges.append(r if hit else range_max + 1.0)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

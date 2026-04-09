#!/usr/bin/env python3
"""
ROS2 node that bridges the local car's state to the centralized WebSocket
server and publishes other cars' poses on /swarm_poses for the costmap plugin.
Also receives commands from a centralized commander and publishes them locally.

Parameters
----------
server_ip   : str   – WebSocket server IP   (default: "localhost")
server_port : int   – WebSocket server port  (default: 8765)

Subscriptions
-------------
amcl_pose          (PoseWithCovarianceStamped) – this car's localisation
/odom              (Odometry)                  – odometry for linear velocity
/obstacles         (PoseArray)                 – local obstacle detections (optional)

Publications
------------
/swarm_poses       (PoseArray)                 – other cars' poses (consumed by SwarmLayer)
/swarm_obstacles   (PoseArray)                 – obstacles reported by other cars (optional)
/commander_cmd     (String)                    – commands from the centralized commander
"""

import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import websockets


class WebsocketClientNode(Node):
    def __init__(self):
        super().__init__("websocket_client_node")

        # --- ROS parameters ------------------------------------------------
        self.declare_parameter("server_ip", "localhost")
        self.declare_parameter("server_port", 8765)

        server_ip = self.get_parameter("server_ip").get_parameter_value().string_value
        server_port = self.get_parameter("server_port").get_parameter_value().integer_value
        self.ws_url = f"ws://{server_ip}:{server_port}"

        # Car ID: explicit parameter takes precedence, then namespace, then fallback
        self.declare_parameter("car_id", "")
        car_id_param = self.get_parameter("car_id").get_parameter_value().string_value
        self.car_id = car_id_param or self.get_namespace().strip("/") or "car_default"

        # --- Publishers -----------------------------------------------------
        self.swarm_pub = self.create_publisher(PoseArray, "/swarm_poses", 10)
        self.obstacle_pub = self.create_publisher(PoseArray, "/swarm_obstacles", 10)
        self.cmd_pub = self.create_publisher(String, "/commander_cmd", 10)

        # --- State ----------------------------------------------------------
        self.neighbor_poses: dict[str, dict] = {}       # car_id -> {x, y, yaw}
        self.neighbor_obstacles: dict[str, list] = {}   # car_id -> [{x, y}, ...]
        self.neighbor_timestamps: dict[str, float] = {} # car_id -> monotonic time
        self.local_obstacles: list[dict] = []
        self._vx: float = 0.0                           # linear velocity from odom
        self._status: str = "idle"                       # navigating / idle
        self._current_goal: dict | None = None           # {x, y} or None
        self._last_payload: dict | None = None          # cached for heartbeat
        self._lock = threading.Lock()

        # --- WebSocket (asyncio loop in background thread) ------------------
        self._ws = None             # current websockets connection
        self._ws_connected = False
        self._loop = asyncio.new_event_loop()
        t = threading.Thread(target=self._loop.run_forever, daemon=True)
        t.start()
        asyncio.run_coroutine_threadsafe(self._ws_connect_loop(), self._loop)

        # --- Subscriptions --------------------------------------------------
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._pose_cb, 10
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )
        self.create_subscription(
            PoseArray, "/obstacles", self._obstacle_cb, 10
        )

        # --- Timers ---------------------------------------------------------
        self.create_timer(0.1, self._publish_swarm_poses)       # 10 Hz
        self.create_timer(1.0, self._cleanup_stale_neighbors)   # stale check
        self.create_timer(0.5, self._heartbeat_cb)              # heartbeat

        self.get_logger().info(
            f"[{self.car_id}] WebSocket client targeting {self.ws_url}"
        )

    # ------------------------------------------------------------------ ws

    async def _ws_connect_loop(self):
        """Continuously connect to the server, reconnecting on any failure."""
        while True:
            try:
                async with websockets.connect(self.ws_url) as ws:
                    self._ws = ws
                    self._ws_connected = True
                    self.get_logger().info(
                        f"[{self.car_id}] Connected to {self.ws_url}"
                    )
                    async for raw in ws:
                        self._on_message(raw)
            except Exception as exc:
                self._ws = None
                self._ws_connected = False
                self.get_logger().warn(
                    f"[{self.car_id}] WebSocket disconnected: {exc}. Retrying in 2s..."
                )
            await asyncio.sleep(2.0)

    def _on_message(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        # --- Command messages from the commander ---
        if data.get("type") == "command":
            msg = String()
            msg.data = raw
            self.cmd_pub.publish(msg)
            self.get_logger().info(
                f"[{self.car_id}] Received command: {data.get('action', '?')}"
            )
            return

        # --- Peer state messages ---
        cid = data.get("car_id")
        if cid is None or cid == self.car_id:
            return

        with self._lock:
            self.neighbor_poses[cid] = {
                "x": data.get("x", 0.0),
                "y": data.get("y", 0.0),
                "yaw": data.get("yaw", 0.0),
            }
            self.neighbor_timestamps[cid] = time.monotonic()
            obstacles = data.get("obstacles")
            if obstacles:
                self.neighbor_obstacles[cid] = obstacles
            elif cid in self.neighbor_obstacles:
                del self.neighbor_obstacles[cid]

    def _ws_send(self, payload: dict):
        """Thread-safe send from ROS callbacks into the asyncio loop."""
        if not self._ws_connected or self._ws is None:
            return
        asyncio.run_coroutine_threadsafe(
            self._ws.send(json.dumps(payload)), self._loop
        )

    # -------------------------------------------------------------- ROS cb

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        """Forward this car's pose, velocity, status, and obstacles to the server."""
        if not self._ws_connected:
            return

        p = msg.pose.pose
        yaw = _quaternion_to_yaw(p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w)
        payload: dict = {
            "car_id": self.car_id,
            "x": round(p.position.x, 3),
            "y": round(p.position.y, 3),
            "yaw": round(yaw, 4),
            "vx": round(self._vx, 3),
            "status": self._status,
        }

        if self._current_goal is not None:
            payload["current_goal"] = self._current_goal

        if self.local_obstacles:
            payload["obstacles"] = self.local_obstacles

        self._last_payload = payload
        self._ws_send(payload)

    def _heartbeat_cb(self):
        """Re-send last known state so peers don't expire us as stale."""
        if not self._ws_connected:
            return
        payload = self._last_payload
        if payload is None:
            return
        self._ws_send(payload)

    def _odom_cb(self, msg: Odometry):
        """Update linear velocity and derive status from it."""
        self._vx = msg.twist.twist.linear.x
        self._status = "navigating" if abs(self._vx) > 0.05 else "idle"

    def _obstacle_cb(self, msg: PoseArray):
        """Cache local obstacle detections to include in the next broadcast."""
        self.local_obstacles = [
            {"x": round(p.position.x, 3), "y": round(p.position.y, 3)}
            for p in msg.poses
        ]

    def _cleanup_stale_neighbors(self):
        """Remove neighbors not heard from in over 2 seconds."""
        now = time.monotonic()
        with self._lock:
            stale = [cid for cid, ts in self.neighbor_timestamps.items()
                     if now - ts > 2.0]
            for cid in stale:
                self.neighbor_poses.pop(cid, None)
                self.neighbor_obstacles.pop(cid, None)
                self.neighbor_timestamps.pop(cid, None)
                self.get_logger().info(f"Removed stale neighbor: {cid}")

    def set_current_goal(self, x: float, y: float):
        """Set the current goal (called externally by a goal executor)."""
        self._current_goal = {"x": round(x, 3), "y": round(y, 3)}

    def clear_current_goal(self):
        """Clear the current goal (e.g. on completion or cancellation)."""
        self._current_goal = None

    # ---------------------------------------------------------- publishing

    def _publish_swarm_poses(self):
        """Publish neighbor car poses and obstacles as PoseArrays."""
        now = self.get_clock().now().to_msg()

        with self._lock:
            # --- car poses ---
            pose_msg = PoseArray()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = "map"
            for info in self.neighbor_poses.values():
                p = Pose()
                p.position.x = float(info["x"])
                p.position.y = float(info["y"])
                q = _yaw_to_quaternion(info["yaw"])
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                pose_msg.poses.append(p)
            self.swarm_pub.publish(pose_msg)

            # --- obstacles reported by other cars ---
            obs_msg = PoseArray()
            obs_msg.header.stamp = now
            obs_msg.header.frame_id = "map"
            for obs_list in self.neighbor_obstacles.values():
                for o in obs_list:
                    p = Pose()
                    p.position.x = float(o.get("x", 0.0))
                    p.position.y = float(o.get("y", 0.0))
                    obs_msg.poses.append(p)
            self.obstacle_pub.publish(obs_msg)


# ---------------------------------------------------------------- helpers

def _quaternion_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# ------------------------------------------------------------------ main

def main(args=None):
    rclpy.init(args=args)
    node = WebsocketClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

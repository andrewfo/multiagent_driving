# multiagent_driving

Multi-robot F1TENTH package. An ego car drives autonomously around a track using Nav2 + MPPI while other cars share their poses over WebSocket so the `SwarmLayer` costmap plugin can mark them as obstacles — enabling real-time collision avoidance.

---

## First time setup

Clone the workspace, then clone this package into `src/`:

```bash
git clone https://github.com/ut-av/roboracer_ws ~/roboracer_ws
cd ~/roboracer_ws/src
git clone https://github.com/andrewfo/multiagent_driving.git
cd ~/roboracer_ws
./scripts/checkout.sh
./container build
./container shell
make
source install/setup.bash
```

**Note:** `multiagent_driving` is a separate repo and is not included when you clone `roboracer_ws` — you must clone it separately into `src/`.

The `swarm_costmap_plugin` is a C++ ament_cmake package inside `multiagent_driving`. `make` / `colcon build` compiles it automatically. After editing `swarm_layer.cpp`, rebuild and re-source before the costmap picks up the changes:

```bash
colcon build --packages-select swarm_costmap_plugin
source install/setup.bash
```

---

## Simulation mode (recommended for development)

Replaces the physical car, lidar, and WebSocket pipeline with a single lightweight Python node. Nav2, SwarmLayer, car_filter, and track_navigator all run unmodified on top.

### Launch

```bash
ros2 launch multiagent_driving sim.launch.py
```

With options:

```bash
ros2 launch multiagent_driving sim.launch.py \
  map:=gdc_3n \
  num_ghost_cars:=3 \
  ghost_speed:=0.4
```

| Argument | Default | Description |
|---|---|---|
| `map` | `home` | Map name (must exist in `av_navigation/maps/`) |
| `num_ghost_cars` | `2` | Number of ghost cars (1–4) |
| `ghost_speed` | `0.3` | Base ghost car speed (m/s) |
| `ego_start_x/y/yaw` | first waypoint | Ego car starting pose |

RViz opens automatically with `rviz2/sim.rviz`. The ego car starts driving laps immediately.

### What the sim publishes

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/scan` | `LaserScan` | 10 Hz | Raycast lidar — hits map walls and ghost car OBBs |
| `/odom` | `Odometry` | 20 Hz | Ego car odometry |
| `/amcl_pose` | `PoseWithCovarianceStamped` | 20 Hz | Perfect localization |
| `/map` | `OccupancyGrid` | once | Static map (latched) |
| `/swarm_poses` | `PoseArray` | 10 Hz | Ghost car poses (x, y, yaw) in map frame |
| `/swarm_obstacles` | `PoseArray` | 10 Hz | Manually placed point obstacles |
| `/ghost_car_footprints` | `MarkerArray` | 10 Hz | Oriented bounding box per ghost car |
| `/ghost_car_N/current_goal` | `PoseStamped` | 1 Hz | Next waypoint target for ghost car N |
| `/ghost_car_N/plan` | `nav_msgs/Path` | 1 Hz | Remaining lap waypoints for ghost car N |

### Ghost car behaviour

Ghost cars follow `TRACK_WAYPOINTS_XY` (defined in `sim_world.py`) with three realism features:

**Curvature-aware speed** — turning angle at each waypoint is precomputed. Speed scales from 1.0× (straight) down to 0.4× (sharp corner), smoothed with an exponential approach per step.

**Staggered base speeds** — fixed multipliers per car so they spread out naturally:

| Ghost index | Speed multiplier |
|---|---|
| 0 | 1.00× |
| 1 | 0.85× |
| 2 | 1.10× |
| 3 | 0.95× |

**Lateral perturbations** — ±0.02 m sinusoidal oscillation at 0.5 Hz perpendicular to heading (different phase per car), preventing identical lidar returns on repeated laps.

### Ghost car physical size

All layers use the real car footprint from `nav2.yaml`:

```
[[0.15, 0.045], [0.15, -0.045], [-0.15, -0.045], [-0.15, 0.045]]  # 0.3 m × 0.09 m
```

- **Lidar** (`sim_world.py`) — OBB hit test per ghost car, so returns look like real F1TENTH cars
- **Costmap** (`swarm_layer.cpp`) — stamps the exact rotated 0.3×0.09 m rectangle of `LETHAL_OBSTACLE` cells in the local costmap so MPPI avoids them
- **Scan filter** (`car_filter_node.py`) — suppresses scan points inside the OBB + 0.05 m margin per side

### Injecting obstacles

Click anywhere on the map with the **Publish Point** tool in RViz. Click near an existing obstacle to remove it. Clear all:

```bash
ros2 service call /clear_obstacles std_srvs/srv/Empty
```

---

## On-car mode (physical hardware)

Two separate launches are used: one for the WebSocket server (run once, on one machine) and one per car.

### Step 1 — Start the WebSocket relay server

Run this on one car or a dedicated laptop on the same network:

```bash
ros2 run multiagent_driving websocket_server
```

Note the machine's IP address. Every car needs to reach it on port 8765.

### Step 2 — Start the Nav2 stack on each car

```bash
MAP=gdc_3n tmuxinator start navstack
```

Set the initial pose in RViz2 (**2D Pose Estimate**, click+drag on the map).

### Step 3 — Launch all per-car nodes

Run on each car, giving each a **unique `car_id`**:

```bash
# Car 1
ros2 launch multiagent_driving multiagent.launch.py \
  server_ip:=192.168.1.100 \
  car_id:=car1

# Car 2 (on the second car)
ros2 launch multiagent_driving multiagent.launch.py \
  server_ip:=192.168.1.100 \
  car_id:=car2
```

This launches `track_navigator`, `websocket_client`, `car_filter`, and `obstacle_detector`.

> **Do NOT use ROS namespaces to distinguish cars.** Namespacing causes relative
> topic subscriptions (`amcl_pose`, `follow_waypoints` action server) to resolve
> incorrectly. Use `car_id` instead.

| Argument | Default | Description |
|---|---|---|
| `mode` | `multi` | `multi` = full stack, `single` = track_navigator only, `server` = relay server only |
| `car_id` | `car_default` | **Unique ID for this car** (e.g. `car1`, `car2`). Must differ between cars. |
| `server_ip` | `localhost` | IP of the machine running `websocket_server` |
| `server_port` | `8765` | WebSocket port |
| `car_margin` | `0.05` | Extra margin (m) added to each side of the OBB when filtering lidar |
| `log_metrics` | `false` | Set to `true` to launch the metrics logger |
| `metrics_output_file` | `` | CSV path for metrics; empty = auto-generate in `/tmp` |
| `near_miss_threshold` | `0.35` | Scan range (m) that triggers a near-miss event |

### Single-car mode (no sharing)

To run just the ego car with no multi-car features:

```bash
ros2 launch multiagent_driving multiagent.launch.py mode:=single
```

---

## Architecture

```
─────── Sim (sim_world.py) ─────────────────────────────────────────────────────
sim_world
  ├── /scan          ─────────────────────────────────────────────┐
  ├── /odom, /amcl_pose                                           │
  └── /swarm_poses   ─────────────────────────────────────┐       │
                                                           │       │
─────── Hardware (per car) ──────────────────────────────────────────────────────
websocket_client_node                                      │       │
  ├── broadcasts this car's pose to websocket_server       │       │
  └── /swarm_poses (from server) ───────────────────────── ┘       │
                                                                   │
car_filter_node                                                    │
  ├── /scan ──────────────────────────────────────────────────────  ┤
  ├── /swarm_poses (suppresses scan points inside known car OBBs)   │
  └── /scan_filtered ─────────────────────────────────────────────► │
                                                                    │
obstacle_detector_node                                              │
  ├── /scan_filtered (clusters points not on static map)            │
  └── /obstacles (centroid PoseArray) ──► websocket_client ──► peers│
                                                                    │
Nav2 local_costmap                                                  │
  ├── voxel_layer ◄── /scan_filtered ─────────────────────────────  ┘
  ├── swarm_layer ◄── /swarm_poses (marks other cars as LETHAL, map→odom TF applied)
  └── inflation_layer
        │
        ▼
  MPPI controller (CostCritic reads local_costmap → real-time avoidance)
        │
  track_navigator (FollowWaypoints → Nav2 goal sequence)
```

**Why local costmap?** MPPI's `CostCritic` samples trajectories against the **local costmap** at runtime. Putting `SwarmLayer` in the global costmap only affects path planning (NavfnPlanner) — MPPI would see a clear costmap and drive straight into other cars. With `SwarmLayer` in the local costmap, MPPI actively steers away from marked cars in real time.

**SwarmLayer TF transform** — `/swarm_poses` is published in the `map` frame (from AMCL). The local costmap uses the `odom` frame. `SwarmLayer` calls `tf2::transform` on each received pose before stamping it onto the grid, so obstacle positions are always correct even as AMCL drift accumulates.

**car_filter purpose** — without it, ghost car / peer car scan returns would land on `voxel_layer` AND `swarm_layer`, creating a double-counted obstacle and inflating the cost. `car_filter` removes scan points inside each known car's OBB before `/scan_filtered` reaches the voxel layer.

---

## Node reference

| Node | Executable | Description |
|---|---|---|
| `sim_world` | `sim_world` | Sim-only: publishes /scan, /odom, /amcl_pose, ghost car topics |
| `websocket_server` | `websocket_server` | Relay server — run once on one machine |
| `websocket_client` | `websocket_client` | Per-car: bridges AMCL pose to server, publishes /swarm_poses |
| `car_filter` | `car_filter` | Per-car: removes ghost/peer car points from /scan → /scan_filtered |
| `obstacle_detector` | `obstacle_detector` | Per-car: clusters /scan_filtered, publishes /obstacles for sharing |
| `track_navigator` | `track_navigator` | Per-car: drives laps via Nav2 FollowWaypoints |
| `metrics_logger` | `metrics_logger` | Optional: logs near-misses, collision proxies, lap times to CSV |

---

## Setting waypoints

Edit `TRACK_WAYPOINTS_XY` in **both** `sim_world.py` (used by the sim) and `track_navigator.py` (used on hardware) — both lists must match. Each entry is `(x, y)` in the map frame; yaw is computed automatically from consecutive points.

To find coordinates:
- Use the **Publish Point** tool in RViz2 and read the `/clicked_point` topic, or
- Echo `/amcl_pose` while driving the car manually

---

## Metrics logging

The metrics logger writes a CSV in real time with collision-avoidance experiment data.

```bash
ros2 launch multiagent_driving multiagent.launch.py \
  log_metrics:=true \
  metrics_output_file:=/tmp/run1.csv
```

CSV columns: `timestamp_sec, event_type, value, car_x, car_y`

| `event_type` | `value` | Trigger |
|---|---|---|
| `near_miss` | min scan range (m) | any scan return < `near_miss_threshold` (default 0.35 m) |
| `collision_proxy` | min scan range (m) | any scan return < 0.20 m (subset of near_miss) |
| `lap_start` | lap number | ego car enters lap-start zone |
| `lap_complete` | lap duration (s) | ego car completes a lap |

The logger reads raw `/scan` (before filtering) so ghost car returns count toward near-miss events.

---

## Tuning

**Ghost car speed:** `ghost_speed` launch arg sets the base m/s. Per-car multipliers are `_SPEED_MULTIPLIERS` in `sim_world.py`.

**Costmap footprint:** `half_len` / `half_wid` in `swarm_layer.cpp` must match the `footprint` in `av_navigation/config/nav2.yaml`. Both are currently `0.15 m × 0.045 m` (half-extents of the 0.3×0.09 m car).

**Lidar filter margin:** `car_margin` ROS param on `car_filter_node` (default 0.05 m). Increase if peer scan returns still bleed through; decrease if the filter eats too much of the track.

**MPPI collision cost:** `CostCritic.cost_weight` in `av_navigation/config/nav2.yaml` (currently 3.81). Higher values make MPPI steer away from other cars earlier but may cause jerky paths.

**Nav2 max ego speed:**
```yaml
# av_navigation/config/nav2.yaml
FollowPath:
  vx_max: 0.5  # m/s
```
Rebuild `av_navigation` after changing: `colcon build --packages-select av_navigation`.

---

## Pushing your changes

All commits go to the `multiagent_driving` repo, **not** the root `roboracer_ws` repo:

```bash
cd ~/roboracer_ws/src/multiagent_driving
git pull
git add .
git commit -m "Your commit message"
git push
```

Always pull before committing. If you get a divergent-branches error: `git pull --rebase origin main`.

**Do not** commit from `~/roboracer_ws` root — that is the course workspace repo.

## Pulling teammate changes

```bash
cd ~/roboracer_ws/src/multiagent_driving
git pull
cd ~/roboracer_ws
colcon build --packages-select multiagent_driving swarm_costmap_plugin
source install/setup.bash
```

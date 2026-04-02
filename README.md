# multiagent_driving

Multi-robot F1TENTH package. Runs a Nav2 ego car autonomously around a track while simulated ghost cars stress-test the shared `SwarmLayer` costmap plugin.

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

The `swarm_costmap_plugin` is a C++ ament_cmake package inside `multiagent_driving`. `make` / `colcon build` will compile it automatically. If you change `swarm_layer.cpp` you must rebuild and re-source before the costmap picks up the changes:

```bash
colcon build --packages-select swarm_costmap_plugin
source install/setup.bash
```

---

## Simulation mode (recommended for development)

Replaces the physical car, Unity simulator, and websocket pipeline with a single lightweight Python node. Nav2, SwarmLayer, car_filter, and track_navigator all run unmodified on top.

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

### What the sim publishes

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/scan` | `LaserScan` | 10 Hz | Raycast lidar — hits map walls and ghost car OBBs |
| `/odom` | `Odometry` | 20 Hz | Ego car odometry |
| `/amcl_pose` | `PoseWithCovarianceStamped` | 20 Hz | Perfect localization |
| `/map` | `OccupancyGrid` | once | Static map (latched) |
| `/swarm_poses` | `PoseArray` | 10 Hz | Ghost car poses (x, y, yaw) in map frame |
| `/swarm_obstacles` | `PoseArray` | 10 Hz | Manually placed point obstacles |
| `/ghost_car_footprints` | `MarkerArray` | 10 Hz | Oriented bounding box outline per ghost car |
| `/ghost_car_N/current_goal` | `PoseStamped` | 1 Hz | Next waypoint target for ghost car N |
| `/ghost_car_N/plan` | `nav_msgs/Path` | 1 Hz | Remaining lap waypoints for ghost car N |

### Ghost car behaviour

Ghost cars follow `TRACK_WAYPOINTS_XY` (defined in `sim_world.py`) with three realism features:

**Curvature-aware speed** — turning angle at each waypoint is precomputed. Speed scales from 1.0× (straight) down to 0.4× base speed (sharp corner), smoothed with an exponential approach (`v += 0.1 * (target - v)` per 0.1 s step).

**Staggered base speeds** — each ghost gets a fixed multiplier so they spread out naturally and produce passing opportunities:

| Ghost index | Speed multiplier |
|---|---|
| 0 | 1.00× |
| 1 | 0.85× |
| 2 | 1.10× |
| 3 | 0.95× |

**Lateral perturbations** — ±0.02 m sinusoidal oscillation at 0.5 Hz perpendicular to heading (different phase per car), preventing identical lidar returns on repeated laps.

### Ghost car physical size

All three layers use the real car footprint from `nav2.yaml`:

```
[[0.15, 0.045], [0.15, -0.045], [-0.15, -0.045], [-0.15, 0.045]]  # 0.3 m × 0.09 m
```

- **Lidar** (`sim_world.py`) — OBB hit test instead of a circle, so returns look like real F1TENTH cars
- **Costmap** (`swarm_layer.cpp`) — stamps the exact rotated 0.3×0.09 m rectangle of `LETHAL_OBSTACLE` cells
- **Scan filter** (`car_filter_node.py`) — suppresses scan points inside the OBB + 0.05 m margin per side

### Injecting obstacles

Click anywhere on the map with the **Publish Point** tool in RViz. Click near an existing obstacle to remove it. Clear all obstacles:

```bash
ros2 service call /clear_obstacles std_srvs/srv/Empty
```

---

## On-car mode (physical hardware)

### Nav stack only (no ghost cars)

```bash
MAP=gdc_3n tmuxinator start navstack
```

Set the initial pose in RViz2 (**2D Pose Estimate**, click+drag), then:

```bash
ros2 run multiagent_driving track_navigator
```

### With ghost cars via websocket

Start `websocket_client_node` on each car so they share poses over the network. The `SwarmLayer` and `car_filter` pick up `/swarm_poses` from the websocket client — no other changes needed.

---

## Architecture

```
sim_world (sim only)          websocket_client (hardware only)
     │                                  │
     ├─ /scan ──────────────────────────┤
     ├─ /odom, /amcl_pose               │
     └─ /swarm_poses ──────────────────►SwarmLayer (global_costmap)
                                        │
                                 car_filter_node
                                 /scan → /scan_filtered → obstacle_layer
                                        │
                                  track_navigator
                                  (Nav2 goal sequence)
```

**SwarmLayer** (`swarm_costmap_plugin`) is a `nav2_costmap_2d` plugin registered in `nav2.yaml` under `global_costmap`. It subscribes to `/swarm_poses` and stamps oriented rectangular footprints of `LETHAL_OBSTACLE` on the costmap each update cycle.

**car_filter_node** removes laser scan returns that fall inside a known ghost car's OBB (with 0.05 m margin), preventing the obstacle layer from double-counting cars already handled by SwarmLayer.

---

## Setting waypoints

Edit `TRACK_WAYPOINTS_XY` in `sim_world.py` (used by the sim) and `track_navigator.py` (used by the ego car) — both lists must match. Each entry is `(x, y)` in the map frame; yaw is computed automatically from consecutive points.

To find coordinates: use the **Publish Point** tool in RViz2 and read the `/clicked_point` topic, or echo `/amcl_pose` while driving manually.

---

## Tuning

**Ghost car speed:**  adjust `ghost_speed` launch argument (base m/s). Individual multipliers are `_SPEED_MULTIPLIERS` in `sim_world.py`.

**Costmap footprint:** `half_len` / `half_wid` constants in `swarm_layer.cpp` must match `nav2.yaml`'s `footprint`. Currently both are `0.15 m × 0.045 m` (half-extents of the 0.3×0.09 m car).

**Filter margin:** `car_margin` ROS param on `car_filter_node` (default 0.05 m). Increase if ghost car scan returns still bleed through; decrease if the filter eats too much of the track.

**Nav2 max speed:** in `av_navigation/config/nav2.yaml`:
```yaml
vx_max: 0.5  # m/s
```
Rebuild `av_navigation` after changing.

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

Always pull before committing to avoid conflicts. If you get a divergent-branches error: `git pull --rebase origin main`.

**Do not** commit from `~/roboracer_ws` root — that is the course workspace repo and your changes won't go to the right place.

## Pulling teammate changes

```bash
cd ~/roboracer_ws/src/multiagent_driving
git pull
cd ~/roboracer_ws
colcon build --packages-select multiagent_driving swarm_costmap_plugin
source install/setup.bash
```

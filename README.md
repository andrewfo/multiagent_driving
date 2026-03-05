# multiagent_driving

Package for getting the Roboracer to drive around a track autonomously using Nav2's waypoint follower.

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

**Note:** `multiagent_driving` is a separate repo and is not included when you clone `roboracer_ws` — you must clone it separately into the `src/` directory.

## How to run

Start the nav stack (swap `gdc_3n` for whatever map you're using):

```bash
MAP=gdc_3n tmuxinator start navstack
```

Open RViz2, click **2D Pose Estimate**, and click+drag on the map to set where the car is and which way it's facing. AMCL needs this before it can localize.

Then run the navigator:

```bash
ros2 run multiagent_driving track_navigator
```

## Setting waypoints

Edit `TRACK_WAYPOINTS_XY` in `multiagent_driving/track_navigator.py` with your track coordinates. Each entry is `(x, y)` in the map frame — yaw is computed automatically.

To get coordinates: drive the car manually and echo `/amcl_pose`, or use the **Publish Point** tool in RViz2 and click on the map.

## Speed

In `src/av_navigation/av_navigation/config/nav2.yaml`, change:

```yaml
vx_max: 0.5  # m/s
```

Rebuild av_navigation after.

## Pushing your changes

All commits go to the `multiagent_driving` repo, **not** the root `roboracer_ws` repo:

```bash
cd ~/roboracer_ws/src/multiagent_driving
git add .
git commit -m "Your commit message"
git push
```

**Important:** Do not commit or push from the `~/roboracer_ws` root directory — that is the course workspace repo (`ut-av/roboracer_ws`) and your changes won't go to the right place.

## Pulling teammate changes

```bash
cd ~/roboracer_ws/src/multiagent_driving
git pull
cd ~/roboracer_ws
colcon build --packages-select multiagent_driving
source install/setup.bash
```

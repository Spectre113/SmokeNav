# SmokeNav 



- **Simulation assets**: `src/project_sim/`
  - Robot Xacro: `src/project_sim/urdf/robot.urdf.xacro`
  - Gazebo worlds: `src/project_sim/worlds/`
- **ROS 2 packages** live under `src/` and are built with `colcon`.

## Implemented packages

- **`project_sim`**: Gazebo Classic bringup + robot spawn
  - Launches `custom-flat.world`
  - Spawns the diff-drive robot from Xacro
  - Runs `robot_state_publisher`
  - Publishes a static TF `map -> odom` (convenience for tools expecting `map`)

- **`project_nav`**: reactive navigation
  - Uses a LaserScan topic to compute free sectors and publishes `cmd_vel`
  - Intended behavior: **continuous obstacle avoidance / wandering** (no global goal yet)

- **`project_smoke`**: smoke degradation (LaserScan)
  - Subscribes `/scan` and republishes degraded scan on `/scan_smoked`
  - Controlled by `density` in \([0..1]\)

- **`human_detector`** (from `detection` branch): thermal + radar fusion pipeline
  - Radar clustering + thermal blob detection + fusion node
  - Refer to its launch: `src/human_detector/launch/human_detection_launch.launch.py`

- **`project_detection`**: simple Gazebo model-state “detection stub”
  - Publishes `/human_pose` by reading `/gazebo/model_states` for models named `human_*`
  - Useful as a ground-truth baseline and for end-to-end plumbing

- **`project_eval`**: lightweight CSV metrics logger
  - Logs: odom path length, cmd_vel count, and whether a human pose was received
  - Writes CSV under `./logs/` when using scenario launches

## Expected simulation topics (core)

- **Motion**: `/cmd_vel` (in), `/odom` (out), TF tree including `odom -> base_link`
- **LiDAR**: `/scan`
- **IMU**: `/imu/data`
- **RGBD camera**: `rgb/image_rect_color`, `depth/image_rect_raw`, `depth/color/points`
- **Thermal camera (simulated camera)**: `thermal/image_raw`
- **Radar (approx ray → pointcloud)**: `radar/points`
- **Ultrasonic**: `ultrasonic/front` (`sensor_msgs/Range`)
- **Smoke-degraded scan**: `/scan_smoked`

## Build (workspace root)

```bash
colcon build --symlink-install
source install/setup.bash
```


## Launch commands (copy/paste)

### 1) Gazebo + robot only

```bash
source install/setup.bash
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim sim_bringup.launch.py
```

### 2) Gazebo + reactive navigation

```bash
source install/setup.bash
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim sim_with_nav.launch.py
```

### 3) Gazebo + nav + smoke degradation + (detection stub)

```bash
source install/setup.bash
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim sim_with_smoke.launch.py density:=0.7
```

### 4) Scenario presets (also writes CSV logs)

```bash
source install/setup.bash
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim scenario_clear.launch.py
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim scenario_moderate.launch.py
ROS_LOG_DIR=$PWD/.roslog ros2 launch project_sim scenario_dense.launch.py
```

CSV outputs (relative to repo root):
- `logs/metrics_clear.csv`
- `logs/metrics_moderate.csv`
- `logs/metrics_dense.csv`

### 5) Human detection (thermal + radar fusion)

First install Python deps required by `human_detector` (if you haven’t yet):

```bash
./setup.sh
```

Then launch the detection pipeline (it expects thermal + radar topics):

```bash
source install/setup.bash
ROS_LOG_DIR=$PWD/.roslog ros2 launch human_detector human_detection_launch.launch.py
```

## What you should see (sanity checklist)

- **Robot moves by itself** when `project_nav` is running (reactive wandering).
- Increasing `density` makes navigation **less stable** (more scan dropouts/shorter effective range).
- `project_detection` publishes `/human_pose` if the world contains a model named `human_*`.
- `project_eval` writes CSV logs when you use `scenario_*.launch.py`.

## Next project goals / tasks

- **Smoke modeling improvements**
  - Degrade additional modalities (camera/thermal/radar) using a shared `smoke_density` parameter
  - Introduce controllable dropouts and spurious detections for radar/thermal

- **Navigation upgrade**
  - Replace reactive wandering with goal-based navigation (Nav2 + AMCL + map, or SLAM)
  - Add RViz goal interface and proper `map` frame handling (AMCL instead of static `map -> odom`)

- **Detection realism**
  - Make thermal simulate “hot human target” more explicitly (temperature map / emissive human model)
  - Improve radar simulation beyond ray pointcloud (Doppler/velocity is not simulated currently)

- **Evaluation**
  - Add repeatable scenario runner (multiple seeds, multiple humans, moving targets)
  - Metrics: success rate, collisions, time-to-detect, precision/recall vs ground truth

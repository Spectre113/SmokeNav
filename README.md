# SmokeNav (ROS 2 Humble + Gazebo Classic)

This project simulates a mobile robot that navigates toward a target (a human in Gazebo), avoids obstacles, and "consumes" the target when it gets close enough.

## 1. Project Structure

- `src/project_sim` - world, robot spawn, full-stack launch
- `src/project_nav` - navigation and obstacle avoidance
- `src/project_detection` - Gazebo target detection + target marker
- `src/human_localization` - target tracking/adaptation into `/target_info`
- `src/project_smoke` - lidar smoke filtering

## 2. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (`gazebo_ros`)
- `colcon`

Example base installation:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions
```

## 3. Initial Setup

```bash
cd ~/AMR_ws/src/SmokeNav-test
bash setup.sh
source ~/ros2_venv/bin/activate
```

## 4. Build

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Run Full Stack

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

This launches:
- Gazebo world + robot spawn
- smoke filter
- detector + target marker
- localization + adapter
- sector analyzer + goal-aware navigation

## 6. Important Behavior

- The robot follows the target using `/target_info`.
- At close range, the target is hidden for navigation (to prevent spinning around it).
- When `consume_distance` is reached, the target is removed from Gazebo:
  - `human_0` is deleted
  - `goal_target_marker` is deleted
  - the target does not reappear in the same run

## 7. Where to Change Human Position

World file:

- `src/project_sim/worlds/custom-flat.world`

Look for:

```xml
<model name="human_0">
  <pose>1.5 0.0 0.0 0 0 0</pose>
```

`pose` format: `x y z roll pitch yaw`.

After editing:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --packages-select project_sim --symlink-install
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

## 8. Useful Diagnostics

```bash
ros2 topic echo /target_info --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /gazebo/model_states --once
```

## 9. If Gazebo Is "Not Responding" / `gzserver exit code 255`

Most common reason: an old Gazebo process is still running (master port already occupied).

```bash
pkill -f gzclient
pkill -f gzserver
pkill -f 'spawn_entity.py'
pkill -f 'sim_full_stack.launch.py'

source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start
```

Then launch again (see section 5).

## 10. Current Target Launch Parameters

In `src/project_detection/launch/detection_from_gazebo.launch.py`:

- `consume_on_reach: true`
- `consume_distance: 0.75`
- `target_entity_name: "human_0"`

To make the target disappear earlier, decrease `consume_distance`.

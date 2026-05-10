# SmokeNav (ROS 2 Humble + Gazebo Classic)

This project simulates a mobile robot that navigates toward a target (a human in Gazebo), avoids obstacles, and "consumes" the target when it gets close enough.

## 1. Project Structure

- `src/project_sim` - world, robot spawn, full-stack launch
- `src/project_nav` - multi-sensor sector fusion, navigation, and obstacle avoidance
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
- LiDAR/radar/ultrasonic sector analyzer + target-optional goal-aware navigation

## 6. Important Behavior

- The robot follows the target using `/target_info` when a confident target is available.
- If the target is missing, the robot keeps navigating safely using obstacle sectors instead of stopping.
- Navigation consumes fused obstacle sectors from LiDAR, RGB-D point clouds, radar point clouds, and the front ultrasonic range sensor when those topics are available.
- In narrow passages, the controller keeps moving if the front sector is clear and side distances are still above the robot clearance threshold.
- In narrow passages, target steering is temporarily reduced so the robot does not turn into a nearby wall while trying to face a person.
- At close range, the target is hidden for navigation (to prevent spinning around it).
- When `consume_distance` is reached, the target is removed from Gazebo:
  - `human_0` is deleted
  - `goal_target_marker` is deleted
  - the target does not reappear in the same run

## 7. Navigation / Detection Interface

Obstacle data for navigation:

- `/scan_smoked` or `/scan` (`sensor_msgs/LaserScan`) - LiDAR ranges.
- `/radar/points` (`sensor_msgs/PointCloud2`) - radar obstacle points, expected as `x` forward, `y` left, `z` up.
- `/camera/depth/color/points` (`sensor_msgs/PointCloud2`) - RGB-D obstacle points in ROS optical-frame convention: `z` forward, `x` right, `y` down.
- `/ultrasonic/front` (`sensor_msgs/Range`) - short-range front safety distance.

Human target data for navigation:

- Preferred detector output: `/humans` (`geometry_msgs/PoseArray`) with poses in a TF-connected frame such as `map` or `base_link`.
- Current adapter output consumed by navigation: `/target_info` (`std_msgs/Float32MultiArray`) as `[detected, angle_rad, distance_m, confidence]`.
- `detected`: `1.0` for a visible target, `0.0` for no target or hidden close target.
- `angle_rad`: target bearing in `base_link`; positive means left, negative means right.
- `distance_m`: planar distance from robot base to target.
- `confidence`: `0.0..1.0`; navigation accepts targets at `>= 0.4` by default.

Detection sensors such as thermal/RGB/mmWave should publish human detections into this target interface. Obstacle point clouds are separate inputs and should not be mixed with `/target_info`.

## 8. Where to Change Human Position

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

## 9. Useful Diagnostics

```bash
ros2 topic echo /target_info --once
ros2 topic echo /free_sectors --once
ros2 topic echo /sector_distances --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /gazebo/model_states --once
```

Passage sanity check:

- If `Decision` contains `THROUGH_PASSAGE` and the front distance is above `0.75`, `/cmd_vel.linear.x` should stay meaningfully above zero.
- If it drops to about `0.01`, the robot is still being slowed by target alignment and the launch is probably using an old build.

## 10. If Gazebo Is "Not Responding" / `gzserver exit code 255`

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

## 11. Current Target Launch Parameters

In `src/project_detection/launch/detection_from_gazebo.launch.py`:

- `consume_on_reach: true`
- `consume_distance: 0.75`
- `target_entity_name: "human_0"`

To make the target disappear earlier, decrease `consume_distance`.

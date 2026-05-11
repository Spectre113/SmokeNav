# SmokeNav (ROS 2 Humble + Gazebo Classic)

SmokeNav is a Gazebo Classic simulation of a mobile robot that searches for a human target, moves toward it when detected, avoids obstacles, and keeps exploring when the target is not currently visible.

The navigation stack is intentionally lightweight, but it is no longer a raw three-sector demo. The current stack does the following:

- transforms obstacle sensor data into `base_link` with TF2;
- fuses LiDAR, radar, depth-camera, and ultrasonic data robustly;
- publishes legacy 3-sector obstacle data for compatibility;
- publishes 9 detailed navigation sectors for smoother exploration and passage handling;
- publishes a rolling local occupancy grid/costmap;
- publishes a bounded persistent global occupancy map in `map`;
- plans a reachable global path to frontier areas when no target is visible;
- publishes runtime metrics for navigation and perception;
- keeps exploring if no target is available.

## 1. Packages

- `src/project_sim` - Gazebo world, robot URDF, robot spawn, full-stack launch files.
- `src/project_nav` - obstacle fusion, local/global maps, exploration, target-aware navigation.
- `src/project_detection` - Gazebo human detector and target marker.
- `src/human_localization` - target localization and adapter to `/target_info`.
- `src/project_smoke` - LiDAR smoke degradation filter and smoke-density publication.

## 2. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic with `gazebo_ros`
- `colcon`

Install base dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions
```

## 3. Build

Build from the project root, not from `~/AMR_ws`, because this workspace can contain duplicate package names outside this project.

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For the packages touched by the current navigation stack:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --packages-select project_nav project_smoke project_sim --symlink-install
source install/setup.bash
```

## 4. Run Full Stack

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

This starts:

- Gazebo world and robot spawn;
- `robot_state_publisher` and required static transforms;
- smoke filter `/scan -> /scan_smoked`;
- Gazebo human detector;
- human localization and `/target_info` adapter;
- target marker in Gazebo;
- sector analyzer with TF2, robust fusion, 9 sectors, local costmap, and global map;
- goal-aware navigation controller.

## 5. Navigation Architecture

### 5.1 Perception And Fusion

Node:

- `project_nav/sector_analyzer_node.py`

Inputs:

- `/scan_smoked` or `/scan` (`sensor_msgs/LaserScan`)
- `/radar/points` (`sensor_msgs/PointCloud2`)
- `/camera/depth/color/points` (`sensor_msgs/PointCloud2`)
- `/ultrasonic/front` (`sensor_msgs/Range`)
- `/smoke/density` (`std_msgs/Float32`, optional)

Processing:

- Every sensor point is transformed into `base_link` with TF2.
- Points outside the configured front field of view are ignored for sector decisions.
- Each sector is fused per source using a percentile, not raw minimum distance.
- Source estimates are fused again by percentile, which makes isolated noisy points less destructive.
- Depth point clouds are sampled across the image instead of using only the first points of the cloud.
- A fixed-size global occupancy grid is updated in `map` with log-odds.
- Free cells are traced from the robot to each observed obstacle using ray tracing.
- Long/no-hit LiDAR and ultrasonic rays are also mapped as free space.
- Frontier cells are detected where known free space touches unknown space.
- Frontier cells are clustered, and exploration targets the best cluster instead of a single noisy cell.
- Inside each frontier cluster, the best concrete frontier cell is selected instead of using the cluster centroid.
- Large ring-shaped frontiers are capped in score so they do not trap the robot in one room.
- The current frontier is kept until reached or invalidated, reducing local oscillation.
- A Dijkstra planner searches through known free cells and publishes the next lookahead waypoint on the path, so the robot does not aim through walls at a distant frontier.

Outputs:

- `/free_sectors` (`std_msgs/Int32MultiArray`): legacy 3 sectors `[left, center, right]`.
- `/sector_distances` (`std_msgs/Float32MultiArray`): legacy 3-sector distances `[left, center, right]`.
- `/free_sectors_detailed` (`std_msgs/Int32MultiArray`): detailed sector free flags.
- `/sector_distances_detailed` (`std_msgs/Float32MultiArray`): detailed sector distances.
- `/local_costmap` (`nav_msgs/OccupancyGrid`): rolling local obstacle map in `base_link`.
- `/global_map` (`nav_msgs/OccupancyGrid`): persistent bounded occupancy map in `map`.
- `/exploration_path` (`nav_msgs/Path`): planned global path through known free cells to the selected frontier boundary.
- `/exploration_hint` (`std_msgs/Float32MultiArray`): `[valid, angle_rad, distance_m, score]` lookahead hint toward the next waypoint on `/exploration_path`.
- `/sensor_fusion_metrics` (`std_msgs/String`): JSON metrics from the fusion layer.

Default detailed sector count is `9`.

The global map is not unbounded SLAM. It is a fixed-size occupancy grid, currently `30 x 30 m` at `0.10 m/cell`, so memory use stays bounded. Unknown cells remain `-1`, free cells become `0`, and occupied cells approach `100`.

### 5.2 Target-Aware Navigation

Node:

- `project_nav/goal_aware_nav_node.py`

Inputs:

- `/free_sectors`
- `/sector_distances`
- `/sector_distances_detailed`
- `/exploration_hint`
- `/target_info`
- `/odom`
- `/sensor_fusion_metrics`
- `/smoke/density`

Target interface:

- `/target_info` (`std_msgs/Float32MultiArray`)
- Format: `[detected, angle_rad, distance_m, confidence]`
- `angle_rad > 0`: target is left.
- `angle_rad < 0`: target is right.
- Target is accepted when confidence is at least `target_confidence_threshold`.

Behavior:

- If a confident target exists, the robot moves toward it.
- If the target disappears briefly, target memory can keep the last target for a short time.
- If no target is available, the robot keeps exploring instead of stopping.
- Exploration first uses `/exploration_hint` from the global frontier map.
- If no fresh global frontier is available, exploration falls back to `/sector_distances_detailed`.
- Narrow-passage mode reduces overreaction to close side walls if the front is passable.
- Stuck recovery reverses and turns when commanded motion produces too little odometry progress.

Outputs:

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/navigation_metrics` (`std_msgs/String`): JSON runtime metrics.

## 6. Key Parameters

Main launch files:

- `src/project_sim/launch/sim_full_stack.launch.py`
- `src/project_nav/launch/nav_with_scan.launch.py`

Important perception parameters:

- `base_frame`: target frame for all obstacle data, default `base_link`.
- `enable_tf_transform`: enables TF2 transformation of sensor data.
- `allow_tf_fallback`: if `false`, data without TF is rejected instead of silently used in the wrong frame.
- `num_detailed_sectors`: detailed sector count, default `9`.
- `front_safe_distance`: front sector free threshold.
- `side_safe_distance`: left/right sector free threshold.
- `detailed_safe_distance`: detailed-sector free threshold.
- `fusion_percentile`: percentile used to fuse source estimates.
- `source_percentile`: percentile used inside each sensor source.
- `radar_min_support`: minimum radar points required before radar contributes a sector estimate.
- `depth_min_support`: minimum depth points required before depth contributes a sector estimate.
- `publish_costmap`: enables `/local_costmap`.
- `costmap_resolution`: local costmap resolution.
- `costmap_inflation_radius`: obstacle inflation radius in the local costmap.
- `publish_global_map`: enables `/global_map` and `/exploration_hint`.
- `global_frame`: frame for the persistent map, default `map`.
- `global_map_resolution`: global map resolution, default `0.10`.
- `global_map_width_m`: global map width in meters.
- `global_map_height_m`: global map height in meters.
- `global_map_origin_x`: global map origin x in `map`.
- `global_map_origin_y`: global map origin y in `map`.
- `global_hit_log_odds`: occupancy confidence added for obstacle hits.
- `global_miss_log_odds`: free-space confidence added along sensor rays.
- `frontier_min_distance`: nearest frontier distance accepted for exploration.
- `frontier_max_distance`: farthest frontier distance accepted for exploration.
- `frontier_max_abs_angle_deg`: max heading offset accepted for global exploration.
- `frontier_min_cluster_size`: minimum frontier cluster size.
- `frontier_cluster_weight`: score weight for larger frontier clusters.
- `frontier_cluster_score_cap`: maximum score contribution from cluster size.
- `frontier_distance_weight`: score weight that helps avoid tiny nearby frontiers.
- `frontier_current_bonus`: hysteresis bonus for staying with the current frontier.
- `frontier_reached_distance`: distance where a frontier is considered reached.
- `frontier_keep_radius`: radius for matching the current frontier to new clusters.
- `exploration_path_lookahead_m`: distance ahead on the global path used for `/exploration_hint`.
- `frontier_min_path_distance`: minimum planned path distance before a frontier is considered; this suppresses tiny local loops.
- `frontier_max_path_distance`: maximum planned path distance accepted for an exploration target.

Important navigation parameters:

- `front_safe_distance`: distance where front starts being considered comfortable.
- `front_blocked_distance`: distance where forward motion is blocked.
- `side_safe_distance`: side clearance used by avoidance.
- `wall_caution_distance`: distance where wall avoidance begins.
- `wall_critical_distance`: distance where wall avoidance becomes strong.
- `wall_stop_distance`: distance where non-passage forward motion is stopped.
- `passage_mode_enabled`: enables passage logic.
- `passage_front_clear_distance`: minimum front distance for passage mode.
- `passage_min_side_distance`: minimum side distance allowed in passage mode.
- `passage_danger_alpha_cap`: maximum slowdown from danger while in passage mode.
- `passage_linear_speed`: minimum forward speed while in passage mode.
- `search_linear_speed`: speed used while exploring without target.
- `use_global_exploration`: use `/exploration_hint` when no target exists.
- `exploration_hint_timeout`: max age of a frontier hint.
- `exploration_turn_gain`: turn gain toward the best detailed sector.
- `exploration_center_bias`: penalty for choosing hard-left/hard-right exploration sectors.

Current simulation-oriented values are intentionally less conservative than real-robot safety values. This is a Gazebo test stack.

## 7. Metrics

### 7.1 Sensor Fusion Metrics

Topic:

```bash
ros2 topic echo /sensor_fusion_metrics
```

JSON fields include:

- `active_sources`: sensor sources currently contributing data.
- `source_counts`: number of accepted observations per source.
- `free_sectors`: legacy 3-sector free flags.
- `sector_distances_m`: legacy 3-sector distances.
- `detailed_sector_count`: detailed sector count.
- `detailed_sector_distances_m`: detailed sector distances.
- `min_clearance_m`: closest accepted obstacle point.
- `occupied_cells`: occupied cells in `/local_costmap`.
- `global_known_cells`: known cells in `/global_map`.
- `frontier_distance_m`: distance to the currently published lookahead point.
- `exploration_path_length_m`: planned path length to the selected frontier boundary.
- `exploration_path_cells`: number of cells in the current exploration path.
- `smoke_density`: current smoke density if available.
- `frame`: fusion frame, normally `base_link`.
- `global_frame`: map frame, normally `map`.

### 7.2 Navigation Metrics

Topic:

```bash
ros2 topic echo /navigation_metrics
```

JSON fields include:

- `decision`: current navigation decision state.
- `path_length_m`: odometry path length since node start.
- `stuck_events`: number of stuck recoveries.
- `target_reached_events`: number of target reach events.
- `last_time_to_target_sec`: last measured target acquisition time.
- `collision_risk_events`: count of close-clearance risk transitions.
- `min_clearance_seen_m`: minimum clearance seen since node start.
- `sensor_min_clearance_m`: latest clearance from fusion metrics.
- `smoke_density`: current smoke density if available.
- `target_detected`: current target detection flag.
- `target_confidence`: current target confidence.
- `exploration_hint_valid`: whether global frontier exploration is currently active.
- `exploration_hint_angle`: heading toward the selected frontier in `base_link`.
- `exploration_hint_distance`: distance to the selected frontier.
- `cmd_linear_x`: commanded forward velocity.
- `cmd_angular_z`: commanded yaw velocity.

## 8. Useful Diagnostics

Basic checks:

```bash
ros2 topic echo /target_info --once
ros2 topic echo /free_sectors --once
ros2 topic echo /sector_distances --once
ros2 topic echo /free_sectors_detailed --once
ros2 topic echo /sector_distances_detailed --once
ros2 topic echo /exploration_hint --once
ros2 topic echo /cmd_vel --once
```

Metrics:

```bash
ros2 topic echo /sensor_fusion_metrics --once
ros2 topic echo /navigation_metrics --once
ros2 topic echo /smoke/density --once
```

Costmap:

```bash
ros2 topic echo /local_costmap --once
ros2 topic echo /global_map --once
ros2 topic echo /exploration_path --once
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link radar_link
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 run tf2_ros tf2_echo base_link ultrasonic_front_link
```

If `sector_analyzer_node` warns `TF unavailable ...` and `allow_tf_fallback` is `false`, that sensor data is rejected. This is intentional: using obstacle points in the wrong frame is worse than dropping them.

## 9. Common Decisions In Logs

`goal_aware_nav_node` prints decisions when they change.

Common decisions:

- `GO_TO_TARGET_SMOOTH`: target is visible, obstacles are not constraining motion.
- `GO_TO_TARGET_WITH_AVOID`: target is visible, avoidance is active.
- `GO_TO_TARGET_THROUGH_PASSAGE`: target is visible, passage mode is active.
- `EXPLORE_CLEAR`: no target, path is clear.
- `EXPLORE_WITH_AVOID`: no target, exploration with obstacle avoidance.
- `EXPLORE_THROUGH_PASSAGE`: no target, moving through a passage.
- `AVOID_NO_TARGET`: no target, obstacle avoidance dominates.
- `RECOVERY_STUCK`: odometry progress was too low, recovery maneuver is active.
- `STOP_TIMEOUT`: perception data is stale.
- `STOP_TRAPPED`: front and sides are too constrained.
- `STOP_TARGET_REACHED`: target reached according to distance hint.

## 10. Smoke

`project_smoke` degrades LiDAR by noise, max-range reduction, and random dropouts.

It also publishes the active density:

- `/smoke/density` (`std_msgs/Float32`)

Launch argument:

```bash
ros2 launch project_sim sim_full_stack.launch.py density:=0.5
```

Scenario launch files can also pass density, for example clear/moderate/dense scenarios if present in `src/project_sim/launch`.

## 11. Human Target Position

World file:

- `src/project_sim/worlds/custom-flat.world`

Look for:

```xml
<model name="human_0">
  <pose>1.5 0.0 0.0 0 0 0</pose>
```

Pose format:

```text
x y z roll pitch yaw
```

After changing the world:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --packages-select project_sim --symlink-install
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

## 12. Target Consumption

Target consumption is configured in:

- `src/project_detection/launch/detection_from_gazebo.launch.py`

Important parameters:

- `consume_on_reach`
- `consume_distance`
- `target_entity_name`

When the robot gets close enough, the detector can delete:

- `human_0`
- `goal_target_marker`

## 13. Gazebo Cleanup

If Gazebo is stuck, or `gzserver` exits because an old process is still running:

```bash
pkill -f gzclient
pkill -f gzserver
pkill -f 'spawn_entity.py'
pkill -f 'sim_full_stack.launch.py'

source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start
```

Then run again:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

## 14. Development Checks

Compile changed Python files:

```bash
cd ~/AMR_ws/src/SmokeNav-test
python3 -m py_compile \
  src/project_nav/project_nav/sector_analyzer_node.py \
  src/project_nav/project_nav/goal_aware_nav_node.py \
  src/project_smoke/project_smoke/scan_smoke_filter.py
```

Run package tests:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source install/setup.bash
colcon test --packages-select project_nav project_smoke --event-handlers console_direct+
```

Rebuild the active stack:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --packages-select project_nav project_smoke project_sim --symlink-install
source install/setup.bash
```

#!/bin/bash

# Smoke density parameter (default 0.0)
SMOKE_DENSITY=${1:-0.0}

# Kill any previous instances
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher
pkill -f static_transform_publisher
pkill -f spawn_entity
pkill -f scan_smoke_filter
pkill -f human_localization
pkill -f human_pose_adapter
pkill -f project_nav
sleep 2

# Source workspace
source ~/ros2_ws/install/setup.bash

# Get package paths
SIM_DIR=$(ros2 pkg prefix project_sim --share)
WORLD_PATH="$SIM_DIR/worlds/custom-flat.world"
XACRO_PATH="$SIM_DIR/urdf/robot.urdf.xacro"

echo "========================================="
echo "Starting SmokeNav Full Stack"
echo "Smoke Density: $SMOKE_DENSITY"
echo "========================================="

# Terminal 1: Gazebo Server
gzserver "$WORLD_PATH" \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so &
GZSERVER_PID=$!
echo "[1/8] Gazebo Server started (PID: $GZSERVER_PID)"
sleep 2

# Terminal 2: Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro $XACRO_PATH)" \
  -p use_sim_time:=true &
RSP_PID=$!
echo "[2/8] Robot State Publisher started (PID: $RSP_PID)"
sleep 2

# Terminal 3: TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  --ros-args -p use_sim_time:=true &
TF_PID=$!
echo "[3/8] TF Publisher started (PID: $TF_PID)"
sleep 3

# Terminal 4: Spawn Robot
ros2 run gazebo_ros spawn_entity.py -entity smokenav_robot \
  -topic robot_description -x 0 -y 0 -z 0.1 \
  --ros-args -p use_sim_time:=true &
SPAWN_PID=$!
echo "[4/8] Robot spawned (PID: $SPAWN_PID)"
sleep 3

# Smoke Filter
ros2 run project_smoke scan_smoke_filter --ros-args \
  -p use_sim_time:=true \
  -p density:=$SMOKE_DENSITY &
SMOKE_PID=$!
echo "[4b/8] Smoke filter started (PID: $SMOKE_PID, density=$SMOKE_DENSITY)"
sleep 2

# Terminal 5: Human Detector
ros2 launch human_detector human_detector.launch.py &
DETECTOR_PID=$!
echo "[5/8] Human Detector started (PID: $DETECTOR_PID)"
sleep 3

# Terminal 6: Localization + Adapter
ros2 run human_localization human_localization \
  --ros-args -p use_sim_time:=true &
LOCALIZATION_PID=$!
sleep 1
ros2 run human_localization human_pose_adapter \
  --ros-args -p use_sim_time:=true &
ADAPTER_PID=$!
echo "[6/8] Localization + Adapter started (PIDs: $LOCALIZATION_PID, $ADAPTER_PID)"
sleep 2

# Terminal 7: Navigation
ros2 launch project_nav nav_with_scan.launch.py scan_topic:=/scan_smoked &
NAV_PID=$!
echo "[7/8] Navigation started (PID: $NAV_PID)"
sleep 2

# Terminal 8: GUI
gzclient &
GZCLIENT_PID=$!
echo "[8/8] Gazebo GUI started (PID: $GZCLIENT_PID)"

echo ""
echo "========================================="
echo "Full stack running!"
echo "========================================="
echo ""
echo "Verification commands:"
echo "  ros2 topic echo /humans"
echo "  ros2 topic echo /target_info"
echo "  ros2 topic echo /cmd_vel"
echo ""
echo "Press Ctrl+C to stop everything..."

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $GZSERVER_PID $RSP_PID $TF_PID $SPAWN_PID $SMOKE_PID \
          $DETECTOR_PID $LOCALIZATION_PID $ADAPTER_PID $NAV_PID $GZCLIENT_PID 2>/dev/null
    wait
    echo "Done."
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT SIGTERM

# Wait for any process to exit
wait
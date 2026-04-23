# Human Localization Module

This module implements human localization based on detection outputs in the SmokeNav project.

It receives human detection candidates, performs tracking and filtering, and publishes the confirmed human position in the global map frame.

---

## Overview

The localization module subscribes to the detection topic and performs:

- temporal filtering of detections
- nearest-neighbor tracking
- false positive rejection via confirmation
- confidence estimation
- coordinate transformation to the global frame

The final output includes detection state, confidence, and human position.

---

## Architecture

Input:
- `/humans` (PoseArray) – detection results from sensor fusion

Processing:
- tracking using nearest-neighbor association
- confirmation based on multiple observations
- confidence update over time

Output:
- `/human_localization/detected` (Bool)
- `/human_localization/confidence` (Float32)
- `/human_localization/pose` (PoseStamped, frame: `map`)

---

## Topics

### Subscribed

| Topic   | Type        | Description |
|--------|------------|------------|
| `/humans` | `geometry_msgs/PoseArray` | Detected human candidates |

---

### Published

| Topic | Type | Description |
|------|------|------------|
| `/human_localization/detected` | `std_msgs/Bool` | Whether a human is confirmed |
| `/human_localization/confidence` | `std_msgs/Float32` | Confidence level of detection |
| `/human_localization/pose` | `geometry_msgs/PoseStamped` | Estimated human position in `map` frame |

---

## Method

### Tracking

A simple nearest-neighbor tracking algorithm is used:
- detections are matched to existing tracks by distance
- unmatched detections create new tracks

### Confirmation

A track is confirmed if:
- it is detected in multiple consecutive frames (`hits >= 3`)

This reduces false positives.

### Confidence

Confidence increases with repeated detections and decreases when detections are missed.

### Localization

The position of the confirmed human is:
- first estimated in the robot frame (`base_link`)
- then transformed into the global frame (`map`) using TF2

---

## Running the Module

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select human_localization
source install/setup.bash
```

---

### 2. Run localization node

```bash
ros2 run human_localization human_localization
```

---

### 3. Provide TF (example)

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

---

### 4. Test with simulated detections

```bash
ros2 topic pub -r 5 /humans geometry_msgs/msg/PoseArray "{header: {frame_id: 'base_link'}, poses: [{position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}"
```

---

### 5. Monitor output

```bash
ros2 topic echo /human_localization/detected
ros2 topic echo /human_localization/confidence
ros2 topic echo /human_localization/pose
```

---

## Example Output

```text
detected: true
confidence: 1.0

pose:
  frame_id: map
  position:
    x: 2.0
    y: 1.0
    z: 0.0
```

---

## Notes

- The module assumes availability of TF between `base_link` and `map`
- Tracking is designed for single-target scenarios (can be extended)
- The current implementation prioritizes simplicity and robustness

---

## Future Work

- multi-target tracking
- Kalman filter-based motion model
- integration with real sensors (thermal / radar)
- improved probabilistic data association

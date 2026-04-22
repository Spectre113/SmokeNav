# Human Detection Package

## Overview

Human detection using **thermal camera** + **mmWave radar** fusion for autonomous mobile robots in smoke-filled environments.

## Pipeline

### Radar Pipeline
```
/radar/pointcloud (PointCloud2) → DBSCAN clustering → cluster centers → /radar/human_clusters
```
- Converts PointCloud2 to XYZ array
- Filters by max range (10m)
- DBSCAN clustering (eps=0.3, min_samples=5)
- Publishes flattened 3D cluster centers: `[x1, y1, z1, x2, y2, z2, ...]` in robot frame

### Thermal Pipeline
```
/thermal/image_raw (Image) → temperature threshold → bounding boxes → /thermal/human_positions
```
- Temperature threshold: 30.0-37.5°C
- Contour detection with area and aspect ratio filtering
- Publishes normalized 2D positions: `[x1, y1, x2, y2, ...]` (values 0-1)

### Fusion Pipeline
```
/thermal/human_positions + /radar/human_clusters → 3D→2D projection → spatial matching → /humans
```
- Projects radar 3D points to 2D image plane
- Matches by pixel distance (< 50px)
- Publishes 3D human positions as PoseArray

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| **Input** |
| `/radar/pointcloud` | `sensor_msgs/PointCloud2` | mmWave radar point cloud |
| `/thermal/image_raw` | `sensor_msgs/Image` | Thermal camera feed |
| **Intermediate** |
| `/radar/human_clusters` | `std_msgs/Float32MultiArray` | Radar cluster centers `[x,y,z, ...]` |
| `/thermal/human_positions` | `std_msgs/Float32MultiArray` | Normalized blob centers `[x,y, ...]` (0-1) |
| `/thermal/human_boxes` | `visualization_msgs/MarkerArray` | Bounding boxes for RViz |
| **Output** |
| `/humans` | `geometry_msgs/PoseArray` | Fused human positions (3D) |

## Nodes

| Node | Function |
|------|----------|
| `radar_detection_node` | Clusters radar points using DBSCAN |
| `thermal_detection_node` | Detects humans via temperature thresholding |
| `fusion_node` | Projects and matches detections |

## Parameters

### Radar Node
| Parameter | Default |
|-----------|---------|
| `cluster_epsilon` | 0.3 |
| `cluster_min_points` | 5 |

### Thermal Node
| Parameter | Default |
|-----------|---------|
| `temp_min` | 30.0 |
| `temp_max` | 37.5 |
| `min_area` | 100 |
| `max_area` | 5000 |
| `min_aspect_ratio` | 0.3 |
| `max_aspect_ratio` | 3.0 |
| `debug` | False |

## Dependencies

- ROS2 Humble
- OpenCV (`cv_bridge`)
- scikit-learn
- numpy

## Usage

### 1. Link
```bash
# Create symlink to ROS2 workspace
ln -s ~/SmokeNav/src/human_detector ~/ros2_ws/src/human_detector
```

### 2. Build
```bash
cd ~/ros2_ws
colcon build --packages-select human_detector --symlink-install
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
cd ~/SmokeNav
./setup.sh
source ~/ros2_venv/bin/activate
```

### 4. Run the Pipeline
```bash
# Terminal 1: Radar processing
ros2 run human_detector radar_detection_node

# Terminal 2: Thermal processing
ros2 run human_detector thermal_detection_node

# Terminal 3: Sensor fusion
ros2 run human_detector fusion_node
```

## References

- Cai et al. ["Robust Human Detection under Visual Degradation via Thermal and mmWave Radar Fusion"](https://arxiv.org/pdf/2307.03623) (2023)
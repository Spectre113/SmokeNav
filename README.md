# Human Detection Package

## Overview

Human detection using **thermal camera** + **mmWave radar** fusion for autonomous mobile robots in smoke-filled environments. Implements probabilistic Bayesian sensor fusion:
```
P(Human | Radar, Thermal) ∝ P(Radar | Human) × P(Thermal | Human) × P(Human)
```

## Pipeline

### Radar Pipeline
```
/radar/points (PointCloud2) → DBSCAN clustering → cluster centers → /radar/human_clusters
```
- Converts PointCloud2 to XYZ array
- Filters by max range (10m)
- DBSCAN clustering (eps=0.3, min_points=2)
- Publishes flattened 3D cluster centers: `[x1, y1, z1, x2, y2, z2, ...]` in `radar_link` frame

### Thermal Pipeline
```
/thermal/image_raw (Image) → temperature threshold → bounding boxes → /thermal/human_positions
```
- Temperature conversion: `celsius = pixel / 255 * 20 + 20`
- Temperature threshold: 36.0–40.5°C (emissive white human ≈ 40°C)
- Contour detection with area (>10px²) and aspect ratio (0.1–10.0) filtering
- Publishes normalized 2D positions: `[x1, y1, x2, y2, ...]` (values 0–1)

### Fusion Pipeline
```
/thermal/human_positions + /radar/human_clusters → frame transform → 3D→2D projection → spatial matching → /humans
```
- Transforms radar detections from `radar_link` to `thermal_optical_frame`
- Projects radar 3D points to 2D thermal image plane (320×240, f=277px)
- Greedy nearest-neighbor association within 150px
- Bayesian confidence scoring with radar range/cluster quality, thermal temp/size likelihoods
- Publishes fused 3D human positions as `PoseArray` in `base_link` frame

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| **Input** |
| `/radar/points` | `sensor_msgs/PointCloud2` | mmWave radar point cloud |
| `/thermal/image_raw` | `sensor_msgs/Image` | Thermal camera feed (320×240 mono8) |
| **Intermediate** |
| `/radar/human_clusters` | `std_msgs/Float32MultiArray` | Radar cluster centers `[x,y,z, ...]` |
| `/radar/cluster_metadata` | `std_msgs/Float32MultiArray` | Cluster info `[n_pts, radius, ...]` |
| `/thermal/human_positions` | `std_msgs/Float32MultiArray` | Normalized blob centers `[x,y, ...]` (0–1) |
| `/thermal/human_boxes` | `visualization_msgs/MarkerArray` | Bounding boxes for RViz |
| `/thermal/detection_metadata` | `std_msgs/Float32MultiArray` | Blob area and temp info |
| **Output** |
| `/humans` | `geometry_msgs/PoseArray` | Fused human positions in `base_link` frame |

## Nodes

| Node | Function |
|------|----------|
| `radar_detection_node` | Clusters radar points using DBSCAN |
| `thermal_detection_node` | Detects humans via temperature thresholding |
| `fusion_node` | Probabilistic radar+thermal fusion with frame transform |

## Parameters

### Radar Node
| Parameter | Default | Description |
|-----------|---------|-------------|
| `cluster_epsilon` | 0.3 | DBSCAN neighborhood radius (m) |
| `cluster_min_points` | 2 | Minimum points per cluster |

### Thermal Node
| Parameter | Default | Description |
|-----------|---------|-------------|
| `temp_min` | 36.0 | Minimum detection temp (°C) |
| `temp_max` | 40.5 | Maximum detection temp (°C) |
| `min_area` | 10 | Minimum blob area (px²) |
| `max_area` | 100000 | Maximum blob area (px²) |
| `min_aspect_ratio` | 0.1 | Minimum width/height |
| `max_aspect_ratio` | 10.0 | Maximum width/height |
| `debug` | False | Publish debug images |

### Fusion Node
| Parameter | Default | Description |
|-----------|---------|-------------|
| `match_distance_px` | 80 | Max pixel distance for radar-thermal association |
| `confidence_threshold` | 0.0 | Minimum posterior confidence |
| `prior_human_probability` | 0.5 | Prior P(H) |
| `radar_range_sigma` | 1.5 | Radar reliability decay with distance |
| `radar_cluster_weight` | 0.7 | Balance of cluster quality vs range |
| `thermal_temp_sigma` | 2.0 | Temperature match tightness |
| `thermal_size_sigma` | 0.3 | Size match tightness |
| `spatial_sigma_px` | 20 | Spatial agreement tightness (px) |

## Dependencies

- ROS2 Humble
- OpenCV (`cv_bridge`)
- scikit-learn
- numpy

## Usage

### 1. Link
```bash
ln -s ~/my_projects/SmokeNav/src/human_detector ~/ros2_ws/src/human_detector
```

### 2. Build
```bash
cd ~/ros2_ws
colcon build --packages-select human_detector --symlink-install
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
cd ~/my_projects/SmokeNav
./setup.sh
source ~/ros2_venv/bin/activate
```

### 4. Run the Pipeline
```bash
# All three nodes:
ros2 launch human_detector human_detector.launch.py

# Or individually:
ros2 run human_detector radar_detection_node
ros2 run human_detector thermal_detection_node
ros2 run human_detector fusion_node
```

### 5. Debug
```bash
# View thermal image
ros2 run rqt_image_view rqt_image_view /thermal/image_raw

# Check detections
ros2 topic echo /thermal/human_positions
ros2 topic echo /radar/human_clusters
ros2 topic echo /humans
```

## References

- Cai et al. ["Robust Human Detection under Visual Degradation via Thermal and mmWave Radar Fusion"](https://arxiv.org/pdf/2307.03623) (2023)

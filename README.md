# Human Detection Package

## Overview

This package will performs human detection using **thermal camera** + **mmWave radar** fusion. Designed for low-compute platforms (Raspberry Pi 4, Jetson Nano).

Uses thresholding + clustering + simple fusion.

---

## How It Works

```
Thermal image ──→ Threshold (30-40°C) ──→ Blobs (contours)
                          │
Radar pointcloud ──→ DBSCAN clustering ──→ Clusters
                          │
                          ▼
              Overlap detection ──→ Human detections
```

| Sensor | Method | Output |
|--------|--------|--------|
| Thermal | Temperature threshold + contour detection | Blobs in image frame |
| Radar | DBSCAN + size filtering | Clusters in robot frame |
| Fusion | Spatial overlap + IoU | Human positions + confidence |

---

## Package Structure

```
human_detection/
├── thermal_node.py          # Threshold + contours
├── radar_node.py            # DBSCAN clustering
├── fusion_node.py           # Overlap detection
├── utils/
│   ├── calibration.py       # Camera/radar transforms
│   └── transforms.py        # Coordinate conversion
├── launch/
│   └── detection.launch.py
├── config/
│   └── params.yaml
└── package.xml
```

---

## Dependencies

- ROS2 (Humble/Jazzy)
- OpenCV (`cv_bridge`)
- scikit-learn (DBSCAN)
- numpy

---

## References

- Cai et al. ["Robust Human Detection under Visual Degradation via Thermal and mmWave Radar Fusion"](https://arxiv.org/pdf/2307.03623) (2023) - fusion concept
- DBSCAN: Ester et al. (1996)

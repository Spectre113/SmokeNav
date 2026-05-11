#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
from sklearn.cluster import DBSCAN

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - depends on the ROS installation.
    point_cloud2 = None


class RadarClustering:
    """Convert PointCloud2 radar detections into moving-object clusters."""

    def __init__(
        self,
        epsilon: float = 0.3,
        min_points: int = 5,
        max_range: float = 5.0,
        min_velocity_mps: float = 0.05,
    ) -> None:
        self.epsilon = float(epsilon)
        self.min_points = int(min_points)
        self.max_range = float(max_range)
        self.min_velocity_mps = float(min_velocity_mps)

    def pointcloud2_to_xyzv_array(self, msg):
        """Convert PointCloud2 into an ``N x 4`` array: ``x, y, z, velocity``."""
        if point_cloud2 is None:
            return np.empty((0, 4), dtype=np.float32)

        field_names = {field.name for field in msg.fields}
        requested_fields = ['x', 'y', 'z']
        has_velocity = 'velocity' in field_names
        if has_velocity:
            requested_fields.append('velocity')

        points = []
        for point in point_cloud2.read_points(
            msg,
            field_names=tuple(requested_fields),
            skip_nans=True,
        ):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            velocity = float(point[3]) if has_velocity else 0.0
            points.append((x, y, z, velocity))

        if not points:
            return np.empty((0, 4), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def process(self, msg):
        """Convert -> filter moving points -> cluster -> return centers + metadata."""
        points = self.pointcloud2_to_xyzv_array(msg)
        if len(points) == 0:
            return np.array([], dtype=np.float32), []

        ranges = np.linalg.norm(points[:, :2], axis=1)
        moving_mask = np.abs(points[:, 3]) >= self.min_velocity_mps
        range_mask = ranges <= self.max_range
        filtered = points[np.logical_and(moving_mask, range_mask)]

        if len(filtered) < self.min_points:
            return np.array([], dtype=np.float32), []

        xyz = filtered[:, :3]
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_points).fit(xyz)
        labels = clustering.labels_

        clusters = []
        metadata = []
        for label in sorted(set(labels)):
            if label == -1:
                continue

            cluster_points = filtered[labels == label]
            if len(cluster_points) < self.min_points:
                continue

            center = np.mean(cluster_points[:, :3], axis=0)
            clusters.append(center)

            distances_from_center = np.linalg.norm(cluster_points[:, :3] - center, axis=1)
            radius = float(np.max(distances_from_center)) if len(distances_from_center) else 0.0
            mean_velocity = float(np.mean(cluster_points[:, 3]))

            metadata.append({
                'num_points': len(cluster_points),
                'cluster_radius': radius,
                'mean_velocity': mean_velocity,
            })

        result = []
        for center in clusters:
            result.extend([float(center[0]), float(center[1]), float(center[2])])

        return np.asarray(result, dtype=np.float32), metadata

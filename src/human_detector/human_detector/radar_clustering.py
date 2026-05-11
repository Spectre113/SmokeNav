#!/usr/bin/env python3

import numpy as np
import sys
import os

# Add your virtual environment's site-packages to the path
venv_path = os.path.expanduser('~/ros2_venv/lib/python3.10/site-packages')
if os.path.exists(venv_path) and venv_path not in sys.path:
    sys.path.insert(0, venv_path)
    print(f"Added {venv_path} to sys.path")

from sklearn.cluster import DBSCAN
import struct

class RadarClustering:
    """
    Handles radar point cloud processing and clustering.
    """
    
    def __init__(self, epsilon=0.3, min_points=5, max_range=10.0):
        self.epsilon = epsilon
        self.min_points = min_points
        self.max_range = max_range
    
    def pointcloud2_to_xyz_array(self, msg):
        """
        Convert ROS PointCloud2 message to numpy array of (x, y, z) points.
        """
        field_map = {}
        for field in msg.fields:
            field_map[field.name] = field.offset
        
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            return np.array([])
        
        points = []
        data = msg.data
        
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            
            x = struct.unpack_from('<f', data, offset + field_map['x'])[0]
            y = struct.unpack_from('<f', data, offset + field_map['y'])[0]
            z = struct.unpack_from('<f', data, offset + field_map['z'])[0]
            
            points.append([x, y, z])
        
        return np.array(points)
    
    def cluster_points(self, points):
        """
        Cluster points using DBSCAN.
        Returns list of cluster centers.
        """
        if len(points) < self.min_points:
            return []
        
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_points).fit(points)
        labels = clustering.labels_
        
        unique_labels = set(labels)
        clusters = []
        
        for label in unique_labels:
            if label == -1:
                continue
            
            cluster_points = points[labels == label]
            
            if len(cluster_points) < self.min_points:
                continue
            
            center = np.mean(cluster_points, axis=0)
            clusters.append(center)
        
        return clusters
    
    def process(self, msg):
        """Full pipeline: convert -> cluster.
        Returns: (centers_flat, metadata_list)
            centers_flat: [x1,y1,z1, x2,y2,z2, ...]
            metadata_list: [{'num_points': N, 'cluster_radius': R}, ...]
        """
        points = self.pointcloud2_to_xyz_array(msg)
        
        if len(points) == 0:
            return np.array([]), []
        
        # Filter by max range
        distances = np.linalg.norm(points[:, :2], axis=1)
        points = points[distances <= self.max_range]
        
        if len(points) < self.min_points:
            return np.array([]), []
        
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_points).fit(points)
        labels = clustering.labels_
        
        unique_labels = set(labels)
        clusters = []
        metadata = []
        
        for label in unique_labels:
            if label == -1:
                continue
            
            cluster_points = points[labels == label]
            
            if len(cluster_points) < self.min_points:
                continue
            
            center = np.mean(cluster_points, axis=0)
            clusters.append(center)
            
            # Calculate cluster radius (max distance from center)
            distances_from_center = np.linalg.norm(cluster_points - center, axis=1)
            radius = float(np.max(distances_from_center)) if len(distances_from_center) > 0 else 0.0
            
            metadata.append({
                'num_points': len(cluster_points),
                'cluster_radius': radius
            })
        
        # Flatten centers
        result = []
        for c in clusters:
            result.extend([float(c[0]), float(c[1]), float(c[2])])
        
        return np.array(result), metadata
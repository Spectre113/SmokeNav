#!/usr/bin/env python3

import numpy as np
import struct

class RadarClustering:
    """
    Handles radar point cloud processing and clustering.
    """
    
    def __init__(self, epsilon=0.45, min_points=3, max_range=10.0):
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
        Cluster points using a simple radius-based connected-components pass.
        Returns a list of cluster centers.
        """
        if len(points) < self.min_points:
            return []

        remaining = set(range(len(points)))
        clusters = []

        while remaining:
            seed = remaining.pop()
            cluster_indices = {seed}
            queue = [seed]

            while queue:
                current = queue.pop()
                current_point = points[current]

                neighbors = []
                for index in list(remaining):
                    if np.linalg.norm(points[index] - current_point) <= self.epsilon:
                        neighbors.append(index)

                for index in neighbors:
                    remaining.remove(index)
                    cluster_indices.add(index)
                    queue.append(index)

            if len(cluster_indices) >= self.min_points:
                cluster_points = points[list(cluster_indices)]
                clusters.append(np.mean(cluster_points, axis=0))

        return clusters
    
    def process(self, msg):
        """
        Full pipeline: convert -> cluster
        Returns array of cluster centers.
        """
        points = self.pointcloud2_to_xyz_array(msg)
        
        if len(points) == 0:
            return np.array([])
        
        points = points[np.linalg.norm(points[:, :2], axis=1) <= self.max_range]

        if len(points) < self.min_points:
            return np.array([])

        clusters = self.cluster_points(points)
        
        if len(clusters) == 0:
            # Fallback: expose the closest radar returns as a small candidate set.
            order = np.argsort(np.linalg.norm(points[:, :2], axis=1))
            fallback = points[order[:min(len(points), self.min_points)]]
            return np.array(fallback).reshape(-1)
        
        # Flatten: [x1,y1,z1, x2,y2,z2, ...]
        result = []
        for c in clusters:
            result.extend([c[0], c[1], c[2]])
        
        return np.array(result)
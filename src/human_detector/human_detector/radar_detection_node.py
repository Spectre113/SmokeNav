#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import numpy as np
from .radar_clustering import RadarClustering

class RadarDetectionNode(Node):
    """
    Node for radar point cloud processing.
    Subscribes to radar point cloud, performs clustering, publishes cluster centers.
    """
    
    def __init__(self):
        super().__init__('radar_detection_node')
        
        # Parameters
        self.declare_parameter('cluster_epsilon', 0.45)
        self.declare_parameter('cluster_min_points', 3)
        
        epsilon = self.get_parameter('cluster_epsilon').value
        min_points = self.get_parameter('cluster_min_points').value
        
        # Initialize clustering module
        self.clustering = RadarClustering(epsilon=epsilon, min_points=min_points)
        
        # Subscriber: radar point cloud from dataset/simulation
        self.subscription = self.create_subscription(
            PointCloud2,
            '/radar/points',
            self.radar_callback,
            10
        )
        
        # Publisher: cluster centers (3D positions in robot frame)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/radar/human_clusters',
            10
        )
        
        self.get_logger().info('Radar Detection Node started')
        self.get_logger().info(f'Parameters: epsilon={epsilon}, min_points={min_points}')
    
    def radar_callback(self, msg: PointCloud2):
        """
        Called whenever radar point cloud arrives.
        Processes point cloud and publishes cluster centers.
        """
        # Process the point cloud
        cluster_centers = self.clustering.process(msg)
        
        # Create and publish message
        cluster_msg = Float32MultiArray()
        
        if len(cluster_centers) == 0:
            cluster_msg.data = []
            self.get_logger().debug('No clusters found')
        else:
            cluster_msg.data = cluster_centers.tolist()
            stride = 5 if len(cluster_centers) % 5 == 0 else 3
            num_clusters = len(cluster_centers) // stride
            # Format with 2 decimal places
            formatted = [round(x, 2) for x in cluster_centers]
            self.get_logger().info(f'Published {num_clusters} clusters: {formatted}')
        
        self.publisher.publish(cluster_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadarDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down radar detection node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

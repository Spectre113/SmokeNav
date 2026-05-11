#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from .radar_clustering import RadarClustering


class RadarDetectionNode(Node):
    """Cluster only moving mmWave detections and publish their centers."""

    def __init__(self):
        super().__init__('radar_detection_node')

        self.declare_parameter('cluster_epsilon', 0.3)
        self.declare_parameter('cluster_min_points', 2)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('min_velocity_mps', 0.003)

        epsilon = self.get_parameter('cluster_epsilon').value
        min_points = self.get_parameter('cluster_min_points').value
        max_range_m = self.get_parameter('max_range_m').value
        min_velocity_mps = self.get_parameter('min_velocity_mps').value

        self.clustering = RadarClustering(
            epsilon=epsilon,
            min_points=min_points,
            max_range=max_range_m,
            min_velocity_mps=min_velocity_mps,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/radar/points',
            self.radar_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/radar/human_clusters',
            10
        )

        self.metadata_pub = self.create_publisher(
            Float32MultiArray,
            '/radar/cluster_metadata',
            10
        )

        self.get_logger().info('Radar Detection Node started')
        self.get_logger().info(
            'Parameters: '
            f'epsilon={epsilon}, min_points={min_points}, '
            f'max_range={max_range_m}, min_velocity={min_velocity_mps}'
        )

    def radar_callback(self, msg: PointCloud2):
        cluster_centers, metadata = self.clustering.process(msg)
        cluster_msg = Float32MultiArray()

        if len(cluster_centers) == 0:
            cluster_msg.data = []
            self.get_logger().debug('No moving radar clusters found')
        else:
            cluster_msg.data = cluster_centers.tolist()
            num_clusters = len(cluster_centers) // 3
            formatted = [round(x, 2) for x in cluster_centers]
            velocities = [round(m['mean_velocity'], 2) for m in metadata]
            self.get_logger().info(
                f'Published {num_clusters} moving clusters: {formatted} '
                f'with radial velocities {velocities}'
            )

        self.publisher.publish(cluster_msg)

        if len(metadata) > 0:
            meta_msg = Float32MultiArray()
            for m in metadata:
                meta_msg.data.extend([
                    float(m['num_points']),
                    float(m['cluster_radius']),
                    float(m['mean_velocity']),
                ])
            self.metadata_pub.publish(meta_msg)
        else:
            self.metadata_pub.publish(Float32MultiArray())


def main(args=None):
    rclpy.init(args=args)
    node = RadarDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down radar detection node')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose

from human_detector.probabilistic_fusion import (
    ProbabilisticFusion,
    FusionConfig,
    RadarDetection,
    ThermalDetection,
)


class FusionNode(Node):
    """ROS2 node for probabilistic radar+thermal human detection fusion."""

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('match_distance_px', 80.0)
        self.declare_parameter('confidence_threshold', 0.0)
        self.declare_parameter('prior_human_probability', 0.5)
        self.declare_parameter('radar_only_confidence_scale', 0.35)
        self.declare_parameter('radar_range_sigma', 1.5)
        self.declare_parameter('radar_cluster_weight', 0.7)
        self.declare_parameter('radar_motion_weight', 0.45)
        self.declare_parameter('radar_velocity_sigma', 0.02)
        self.declare_parameter('thermal_temp_sigma', 2.0)
        self.declare_parameter('thermal_size_sigma', 0.3)
        self.declare_parameter('spatial_sigma_px', 20.0)
        self.declare_parameter('publish_empty', False)

        config = FusionConfig(
            match_distance_px=float(self.get_parameter('match_distance_px').value),
            confidence_threshold=float(self.get_parameter('confidence_threshold').value),
            prior_human_probability=float(self.get_parameter('prior_human_probability').value),
            radar_only_confidence_scale=float(
                self.get_parameter('radar_only_confidence_scale').value
            ),
            radar_range_sigma=float(self.get_parameter('radar_range_sigma').value),
            radar_cluster_weight=float(self.get_parameter('radar_cluster_weight').value),
            radar_motion_weight=float(self.get_parameter('radar_motion_weight').value),
            radar_velocity_sigma=float(self.get_parameter('radar_velocity_sigma').value),
            thermal_temp_sigma=float(self.get_parameter('thermal_temp_sigma').value),
            thermal_size_sigma=float(self.get_parameter('thermal_size_sigma').value),
            spatial_sigma_px=float(self.get_parameter('spatial_sigma_px').value),
        )

        self.fusion = ProbabilisticFusion(config)
        self.publish_empty = bool(self.get_parameter('publish_empty').value)

        self._latest_radar_centers: list[tuple[float, float, float]] = []
        self._latest_radar_metadata: list[dict[str, float]] = []

        self.create_subscription(
            Float32MultiArray, '/radar/human_clusters', self.radar_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, '/radar/cluster_metadata', self.radar_metadata_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, '/thermal/human_positions', self.thermal_cb, 10
        )

        self.pub = self.create_publisher(PoseArray, '/humans', 10)

        self.get_logger().info('Probabilistic Fusion Node ready')
        self._log_config(config)

    def _log_config(self, config: FusionConfig):
        self.get_logger().info(
            f'Config: match_dist={config.match_distance_px:.0f}px, '
            f'conf_thresh={config.confidence_threshold:.2f}, prior={config.prior_human_probability:.2f}, '
            f'radar_only_scale={config.radar_only_confidence_scale:.2f}'
        )

    def radar_cb(self, msg: Float32MultiArray):
        data = msg.data
        centers = []

        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                centers.append(
                    (
                        float(data[i]),
                        float(data[i + 1]),
                        float(data[i + 2]),
                    )
                )

        self._latest_radar_centers = centers
        self._update_radar_detections()
        self.try_fuse()

    def radar_metadata_cb(self, msg: Float32MultiArray):
        data = msg.data
        metadata = []
        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                metadata.append({
                    'num_points': float(data[i]),
                    'cluster_radius': float(data[i + 1]),
                    'mean_velocity': float(data[i + 2]),
                })

        self._latest_radar_metadata = metadata
        self._update_radar_detections()
        self.try_fuse()

    def _update_radar_detections(self):
        detections = []
        for index, (x, y, z) in enumerate(self._latest_radar_centers):
            metadata = (
                self._latest_radar_metadata[index]
                if index < len(self._latest_radar_metadata)
                else {}
            )
            detections.append(
                RadarDetection(
                    x=x,
                    y=y,
                    z=z,
                    num_points=int(metadata.get('num_points', 8.0)),
                    cluster_radius=float(metadata.get('cluster_radius', 0.3)),
                    radial_velocity=float(metadata.get('mean_velocity', 0.0)),
                )
            )

        self.fusion.set_radar_detections(detections)
        self.get_logger().debug(f'Received {len(detections)} radar clusters')

    def thermal_cb(self, msg: Float32MultiArray):
        data = msg.data
        detections = []

        for i in range(0, len(data), 2):
            if i + 1 < len(data):
                detections.append(ThermalDetection(
                    norm_x=float(data[i]),
                    norm_y=float(data[i + 1]),
                    area=2000.0,
                    temp_deviation=0.0,
                ))

        self.fusion.set_thermal_detections(detections)
        self.get_logger().debug(f'Received {len(detections)} thermal detections')
        self.try_fuse()

    def try_fuse(self):
        matches = self.fusion.fuse()
        self._publish(matches)

    def _publish(self, matches):
        msg = PoseArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        for match in matches:
            pose = Pose()
            pose.position.x = match.radar.x
            pose.position.y = match.radar.y
            pose.position.z = match.radar.z
            pose.orientation.w = float(match.confidence)
            msg.poses.append(pose)

        if matches:
            summary = ', '.join(
                f'{match.source}:{match.confidence:.2f}' for match in matches
            )
            self.get_logger().info(
                f'Published {len(matches)} human(s) | {summary}'
            )
        elif self.publish_empty:
            self.get_logger().debug('No humans above confidence threshold')

        self.pub.publish(msg)

        debug = self.fusion.get_debug_info(matches)
        self.get_logger().debug(
            f'Debug: radar={debug["radar_count"]} thermal={debug["thermal_count"]} '
            f'matches={debug["matches"]} fused={debug["fused_matches"]} '
            f'radar_only={debug["radar_only_matches"]} '
            f'unmatched_r={debug["unmatched_radar"]} unmatched_t={debug["unmatched_thermal"]}'
        )


def main():
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

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

        # ── Parameters ──
        self.declare_parameter('match_distance_px', 80.0)
        self.declare_parameter('confidence_threshold', 0.0)
        self.declare_parameter('prior_human_probability', 0.5)
        self.declare_parameter('radar_range_sigma', 1.5)
        self.declare_parameter('radar_cluster_weight', 0.7)
        self.declare_parameter('thermal_temp_sigma', 2.0)
        self.declare_parameter('thermal_size_sigma', 0.3)
        self.declare_parameter('spatial_sigma_px', 20.0)
        self.declare_parameter('publish_empty', False)

        # ── Build fusion config from ROS parameters ──
        config = FusionConfig(
            match_distance_px=float(self.get_parameter('match_distance_px').value),
            confidence_threshold=float(self.get_parameter('confidence_threshold').value),
            prior_human_probability=float(self.get_parameter('prior_human_probability').value),
            radar_range_sigma=float(self.get_parameter('radar_range_sigma').value),
            radar_cluster_weight=float(self.get_parameter('radar_cluster_weight').value),
            thermal_temp_sigma=float(self.get_parameter('thermal_temp_sigma').value),
            thermal_size_sigma=float(self.get_parameter('thermal_size_sigma').value),
            spatial_sigma_px=float(self.get_parameter('spatial_sigma_px').value),
        )

        self.fusion = ProbabilisticFusion(config)
        self.publish_empty = bool(self.get_parameter('publish_empty').value)

        # ── State ──
        self._has_radar = False
        self._has_thermal = False

        # ── Subscribers ──
        self.create_subscription(
            Float32MultiArray, '/radar/human_clusters', self.radar_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/thermal/human_positions', self.thermal_cb, 10)

        # ── Publisher ──
        self.pub = self.create_publisher(PoseArray, '/humans', 10)

        self.get_logger().info('Probabilistic Fusion Node ready')
        self._log_config(config)

    def _log_config(self, config: FusionConfig):
        """Log key parameters on startup."""
        self.get_logger().info(
            f'Config: match_dist={config.match_distance_px:.0f}px, '
            f'conf_thresh={config.confidence_threshold:.2f}, '
            f'prior={config.prior_human_probability:.2f}'
        )

    # ── Callbacks ──

    def radar_cb(self, msg: Float32MultiArray):
        """Parse radar clusters: [x1, y1, z1, x2, y2, z2, ...]"""
        data = msg.data
        detections = []

        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                detections.append(RadarDetection(
                    x=float(data[i]),
                    y=float(data[i + 1]),
                    z=float(data[i + 2]),
                ))

        self.fusion.set_radar_detections(detections)
        self._has_radar = True
        self.get_logger().debug(f'Received {len(detections)} radar clusters')
        self.try_fuse()

    def thermal_cb(self, msg: Float32MultiArray):
        """Parse thermal detections: [x1_norm, y1_norm, x2_norm, y2_norm, ...]"""
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
        self._has_thermal = True
        self.get_logger().debug(f'Received {len(detections)} thermal detections')
        self.try_fuse()

    # ── Fusion trigger ──

    def try_fuse(self):
        # If thermal detects human, publish it immediately
        # Just use thermal for angle, radar for distance when both available
        matches = self.fusion.fuse()
        self._publish(matches)

    def _publish(self, matches):
        """Convert fusion matches to PoseArray and publish."""
        msg = PoseArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        for match in matches:
            pose = Pose()
            pose.position.x = match.radar.x
            pose.position.y = match.radar.y
            pose.position.z = match.radar.z
            # Encode confidence in orientation.w (0..1)
            pose.orientation.w = float(match.confidence)
            msg.poses.append(pose)

        # Log fusion results
        if matches:
            confs = ', '.join(f'{m.confidence:.2f}' for m in matches)
            self.get_logger().info(
                f'Fused {len(matches)} human(s) | confidences: [{confs}]'
            )
        elif self.publish_empty:
            self.get_logger().debug('No humans above confidence threshold')

        self.pub.publish(msg)

        # Print debug info every 10 seconds
        debug = self.fusion.get_debug_info(matches)
        self.get_logger().debug(
            f'Debug: radar={debug["radar_count"]} thermal={debug["thermal_count"]} '
            f'matches={debug["matches"]} '
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose


class FusionNode(Node):
    
    def __init__(self):
        super().__init__('fusion_node')
        
        # Latest data
        self.thermal = []  # [(x, y) in pixels]
        self.radar = []    # [(x, y, z) in meters]
        
        # Subscribers
        self.create_subscription(Float32MultiArray, '/thermal/human_positions', self.thermal_cb, 10)
        self.create_subscription(Float32MultiArray, '/radar/human_clusters', self.radar_cb, 10)
        
        # Publisher
        self.pub = self.create_publisher(PoseArray, '/humans', 10)
        
        self.get_logger().info('Fusion node ready (radar-primary, thermal-optional)')
    
    def thermal_cb(self, msg):
        self.thermal = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.fuse()
    
    def radar_cb(self, msg):
        self.radar = [(msg.data[i], msg.data[i+1], msg.data[i+2]) for i in range(0, len(msg.data), 3)]
        self.fuse()
    
    def project_radar(self, points_3d):
        """Simple projection: x-forward, y-left -> image coordinates."""
        points_2d = []
        for x, y, z in points_3d:
            if x > 0.5:  # in front
                u = 320 + (y / x) * 300  # assume 640px width, 300px focal length
                v = 256 - (z / x) * 300  # assume 512px height
                if 0 <= u < 640 and 0 <= v < 512:
                    points_2d.append((u, v))
        return points_2d
    
    def fuse(self):
        if not self.radar:
            return
        
        humans = PoseArray()
        humans.header.frame_id = 'radar_link'
        humans.header.stamp = self.get_clock().now().to_msg()
        
        radar_2d = self.project_radar(self.radar)
        thermal_hits = len(self.thermal)

        # Radar is the primary sensor here because it is the only one with
        # usable geometry in the current simulation stack. Thermal detections
        # are treated as optional confirmation/debug input.
        for index, (rx, ry, rz) in enumerate(self.radar):
            pose = Pose()
            pose.position.x = rx
            pose.position.y = ry
            pose.position.z = rz
            pose.orientation.w = 1.0
            humans.poses.append(pose)
        
        self.pub.publish(humans)
        self.get_logger().info(
            f'Detected {len(humans.poses)} radar human candidates '
            f'(thermal_hits={thermal_hits})'
        )
        
        # Clear
        self.thermal = []
        self.radar = []


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
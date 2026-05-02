#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import numpy as np


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
        
        self.get_logger().info('Fusion node ready')
    
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
        if not self.thermal or not self.radar:
            return
        
        radar_2d = self.project_radar(self.radar)
        
        humans = PoseArray()
        humans.header.frame_id = 'base_link'
        humans.header.stamp = self.get_clock().now().to_msg()
        
        # Simple matching: if radar point projects near thermal detection
        for rx, ry, rz in self.radar:
            for r2d in radar_2d:
                for tx, ty in self.thermal:
                    # tx, ty are normalized 0-1
                    t_px, t_py = tx * 640, ty * 512
                    dist = np.sqrt((t_px - r2d[0])**2 + (t_py - r2d[1])**2)
                    
                    if dist < 50:  # within 50 pixels
                        pose = Pose()
                        pose.position.x = rx
                        pose.position.y = ry
                        pose.position.z = rz
                        pose.orientation.w = 1.0
                        humans.poses.append(pose)
                        break
        
        self.pub.publish(humans)
        self.get_logger().info(f'Detected {len(humans.poses)} humans')
        
        # Clear
        self.thermal = []
        self.radar = []


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
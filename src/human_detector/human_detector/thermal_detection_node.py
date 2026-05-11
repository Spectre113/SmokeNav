#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray
import cv2
from cv_bridge import CvBridge

from human_detector.thermal_processing import ThermalBoundingBoxDetector


class ThermalDetectionNode(Node):
    """Thermal image human detection using bounding boxes."""
    
    def __init__(self):
        super().__init__('thermal_detection_node')
        
        # Parameters
        self.declare_parameter('temp_min', 36.0)
        self.declare_parameter('temp_max', 40.5)
        self.declare_parameter('min_area', 10)
        self.declare_parameter('max_area', 100000)
        self.declare_parameter('min_aspect_ratio', 0.1)
        self.declare_parameter('max_aspect_ratio', 10.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('camera_frame_id', 'thermal_camera_frame')
        
        self.detector = ThermalBoundingBoxDetector(
            temp_min=self.get_parameter('temp_min').value,
            temp_max=self.get_parameter('temp_max').value,
            min_area=self.get_parameter('min_area').value,
            max_area=self.get_parameter('max_area').value,
            min_aspect_ratio=self.get_parameter('min_aspect_ratio').value,
            max_aspect_ratio=self.get_parameter('max_aspect_ratio').value
        )
        
        self.debug = self.get_parameter('debug').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(Image, '/thermal/image_raw', self.callback, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/thermal/human_boxes', 10)
        self.position_pub = self.create_publisher(Float32MultiArray, '/thermal/human_positions', 10)
        self.meta_pub = self.create_publisher(Float32MultiArray, '/thermal/detection_metadata', 10)
        
        if self.debug:
            self.debug_pub = self.create_publisher(Image, '/thermal/debug_image', 10)
        
        self.get_logger().info('Thermal Detection Node started')
    
    def callback(self, msg: Image):
        """Process thermal image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            boxes, debug_image = self.detector.process_frame(cv_image, debug=self.debug)
            
            if boxes:
                # Publish markers
                markers = self.detector.create_markers(
                    boxes, self.camera_frame_id, self.get_clock().now().to_msg(),
                    msg.width, msg.height
                )
                self.marker_pub.publish(markers)
                
                # Publish normalized positions
                centers = self.detector.get_centers(boxes)
                pos_msg = Float32MultiArray()
                pos_msg.data = [coord for c in centers for coord in (c[0]/msg.width, c[1]/msg.height)]
                self.position_pub.publish(pos_msg)
                
                # Publish metadata: [area1, temp_dev1, area2, temp_dev2, ...]
                meta_msg = Float32MultiArray()
                for (x, y, w, h) in boxes:
                    area = float(w * h)
                    temp_dev = 0.0  # placeholder — needs thermal_processing to return temperature
                    meta_msg.data.extend([area, temp_dev])
                self.meta_pub.publish(meta_msg)
                
                self.get_logger().debug(f'Detected {len(boxes)} humans')
            else:
                # Publish empty messages when nothing detected
                self.marker_pub.publish(MarkerArray())
                self.position_pub.publish(Float32MultiArray())
                self.meta_pub.publish(Float32MultiArray())
            
            if self.debug and debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
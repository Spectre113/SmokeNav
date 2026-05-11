#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray
import cv2
import numpy as np

from human_detector.thermal_processing import ThermalBoundingBoxDetector


class ThermalDetectionNode(Node):
    """Thermal image human detection using bounding boxes."""
    
    def __init__(self):
        super().__init__('thermal_detection_node')
        
        # Parameters
        self.declare_parameter('temp_min', 30.0)
        self.declare_parameter('temp_max', 37.5)
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_area', 5000)
        self.declare_parameter('min_aspect_ratio', 0.3)
        self.declare_parameter('max_aspect_ratio', 3.0)
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
        
        # Subscribers
        self.create_subscription(Image, '/thermal/image_raw', self.callback, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/thermal/human_boxes', 10)
        self.position_pub = self.create_publisher(Float32MultiArray, '/thermal/human_positions', 10)
        
        if self.debug:
            self.debug_pub = self.create_publisher(Image, '/thermal/debug_image', 10)
        
        self.get_logger().info('Thermal Detection Node started')
    
    def callback(self, msg: Image):
        """Process thermal image."""
        try:
            cv_image = self.image_to_array(msg)
            boxes, debug_image = self.detector.process_frame(cv_image, debug=self.debug)
            
            if boxes:
                # Publish markers
                markers = self.detector.create_markers(
                    boxes, self.camera_frame_id, self.get_clock().now().to_msg(),
                    msg.width, msg.height
                )
                self.marker_pub.publish(markers)
                
                # Publish normalized boxes:
                # [center_u, center_v, width, height, area] repeated.
                centers = self.detector.get_centers(boxes)
                pos_msg = Float32MultiArray()
                detections = []
                for (x, y, w, h), center in zip(boxes, centers):
                    detections.extend([
                        center[0] / msg.width,
                        center[1] / msg.height,
                        w / msg.width,
                        h / msg.height,
                        (w * h) / float(msg.width * msg.height),
                    ])
                pos_msg.data = detections
                self.position_pub.publish(pos_msg)
                
                self.get_logger().debug(f'Detected {len(boxes)} humans')
            else:
                self.marker_pub.publish(MarkerArray())
                self.position_pub.publish(Float32MultiArray())
            
            if self.debug and debug_image is not None:
                self.debug_pub.publish(self.array_to_bgr_image(debug_image, msg))
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def image_to_array(self, msg: Image) -> np.ndarray:
        """Convert common Gazebo camera encodings without cv_bridge."""
        encoding = (msg.encoding or '').lower()
        if encoding in ('mono8', '8uc1', 'l8', 'passthrough', ''):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            return data.reshape((msg.height, msg.step))[:, :msg.width]

        if encoding in ('mono16', '16uc1'):
            data = np.frombuffer(msg.data, dtype=np.uint16)
            cols = msg.step // 2
            return data.reshape((msg.height, cols))[:, :msg.width]

        if encoding == '32fc1':
            data = np.frombuffer(msg.data, dtype=np.float32)
            cols = msg.step // 4
            return data.reshape((msg.height, cols))[:, :msg.width]

        if encoding in ('rgb8', 'bgr8'):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
            code = cv2.COLOR_RGB2GRAY if encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
            return cv2.cvtColor(image, code)

        raise ValueError(f'Unsupported thermal image encoding: {msg.encoding}')

    def array_to_bgr_image(self, image: np.ndarray, source: Image) -> Image:
        out = Image()
        out.header = source.header
        out.height = int(image.shape[0])
        out.width = int(image.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = image.astype(np.uint8).tobytes()
        return out


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

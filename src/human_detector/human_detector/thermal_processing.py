#!/usr/bin/env python3

import numpy as np
import cv2
from typing import List, Tuple, Optional
from visualization_msgs.msg import Marker, MarkerArray


class ThermalBoundingBoxDetector:
    """Detects humans in thermal images using temperature thresholding and bounding boxes."""
    
    def __init__(self, 
                 temp_min: float = 30.0,
                 temp_max: float = 37.5,
                 min_area: int = 100,
                 max_area: int = 5000,
                 min_aspect_ratio: float = 0.3,
                 max_aspect_ratio: float = 3.0):
        
        self.temp_min = temp_min
        self.temp_max = temp_max
        self.min_area = min_area
        self.max_area = max_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
    
    def apply_threshold(self, thermal_data: np.ndarray) -> np.ndarray:
        """Create binary mask for human temperature range."""
        if thermal_data.dtype == np.uint16:
            temp_celsius = thermal_data * 0.1
        elif thermal_data.dtype == np.float32:
            temp_celsius = thermal_data
        else:
            # Assume 0-255 normalized, scale to 20-40°C range
            temp_celsius = thermal_data / 255.0 * 20.0 + 20.0
        
        mask = (temp_celsius >= self.temp_min) & (temp_celsius <= self.temp_max)
        return mask.astype(np.uint8) * 255
    
    def detect_bounding_boxes(self, thermal_data: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Find bounding boxes around temperature threshold regions."""
        binary = self.apply_threshold(thermal_data)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        boxes = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            
            if area < self.min_area or area > self.max_area:
                continue
            
            aspect_ratio = w / h if h > 0 else 0
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue
            
            boxes.append((x, y, w, h))
        
        return boxes
    
    def create_markers(self, boxes: List[Tuple[int, int, int, int]], 
                      frame_id: str, timestamp, image_width: int, image_height: int) -> MarkerArray:
        """Create MarkerArray from bounding boxes."""
        marker_array = MarkerArray()
        
        for i, (x, y, w, h) in enumerate(boxes):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "thermal_detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = (x + w/2.0) / image_width
            marker.pose.position.y = (y + h/2.0) / image_height
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = w / image_width
            marker.scale.y = h / image_height
            marker.scale.z = 0.1
            
            marker.color.r = 1.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def get_centers(self, boxes: List[Tuple[int, int, int, int]]) -> List[Tuple[float, float]]:
        """Get center points of bounding boxes."""
        return [(x + w/2.0, y + h/2.0) for x, y, w, h in boxes]
    
    def process_frame(self, thermal_data: np.ndarray, debug: bool = False):
        """Process thermal frame and return bounding boxes."""
        boxes = self.detect_bounding_boxes(thermal_data)
        
        debug_image = None
        if debug:
            if thermal_data.dtype == np.uint16:
                debug_image = ((thermal_data * 0.1 - 20) / 20 * 255).astype(np.uint8)
            else:
                debug_image = thermal_data.astype(np.uint8)
            debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
            
            for x, y, w, h in boxes:
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        return boxes, debug_image
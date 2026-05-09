import math
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class SectorAnalyzerNode(Node):
    def __init__(self) -> None:
        super().__init__('sector_analyzer_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/free_sectors')
        self.declare_parameter('distance_topic', '/sector_distances')

        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_outer_angle_deg', 90.0)

        self.declare_parameter('front_safe_distance', 0.8)
        self.declare_parameter('side_safe_distance', 0.6)

        self.declare_parameter('use_inf_as_free', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('warn_if_no_scan', True)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)

        self.front_half_angle_deg = float(self.get_parameter('front_half_angle_deg').value)
        self.side_outer_angle_deg = float(self.get_parameter('side_outer_angle_deg').value)

        self.front_safe_distance = float(self.get_parameter('front_safe_distance').value)
        self.side_safe_distance = float(self.get_parameter('side_safe_distance').value)

        self.use_inf_as_free = bool(self.get_parameter('use_inf_as_free').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.warn_if_no_scan = bool(self.get_parameter('warn_if_no_scan').value)

        self.free_pub = self.create_publisher(Int32MultiArray, self.output_topic, 10)
        self.distance_pub = self.create_publisher(Float32MultiArray, self.distance_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.latest_scan: LaserScan | None = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sector_info)

        self._no_scan_warned = False

        self.get_logger().info(
            'Sector analyzer node started '
            f'(scan_topic={self.scan_topic}, '
            f'output_topic={self.output_topic}, '
            f'distance_topic={self.distance_topic})'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self._no_scan_warned = False

    def publish_sector_info(self) -> None:
        free_msg = Int32MultiArray()
        distance_msg = Float32MultiArray()

        if self.latest_scan is None:
            free_msg.data = [0, 0, 0]
            distance_msg.data = [0.0, 0.0, 0.0]

            self.free_pub.publish(free_msg)
            self.distance_pub.publish(distance_msg)

            if self.warn_if_no_scan and not self._no_scan_warned:
                self.get_logger().warn('No scan received yet, publishing blocked sectors')
                self._no_scan_warned = True
            return

        left_min = self.get_sector_min_distance(
            self.latest_scan,
            math.radians(self.front_half_angle_deg),
            math.radians(self.side_outer_angle_deg)
        )
        center_min = self.get_sector_min_distance(
            self.latest_scan,
            -math.radians(self.front_half_angle_deg),
            math.radians(self.front_half_angle_deg)
        )
        right_min = self.get_sector_min_distance(
            self.latest_scan,
            -math.radians(self.side_outer_angle_deg),
            -math.radians(self.front_half_angle_deg)
        )

        left_free = self.is_free(left_min, self.side_safe_distance)
        center_free = self.is_free(center_min, self.front_safe_distance)
        right_free = self.is_free(right_min, self.side_safe_distance)

        free_msg.data = [int(left_free), int(center_free), int(right_free)]
        self.free_pub.publish(free_msg)

        distance_msg.data = [
            self.safe_distance_value(left_min, self.latest_scan.range_max),
            self.safe_distance_value(center_min, self.latest_scan.range_max),
            self.safe_distance_value(right_min, self.latest_scan.range_max),
        ]
        self.distance_pub.publish(distance_msg)

        self.get_logger().info(
            'Sectors: '
            f'left_min={self.format_distance(left_min)}, '
            f'center_min={self.format_distance(center_min)}, '
            f'right_min={self.format_distance(right_min)} '
            f'-> free={free_msg.data}'
        )

    def is_free(self, min_distance: float, threshold: float) -> bool:
        if math.isinf(min_distance):
            return self.use_inf_as_free
        return min_distance > threshold

    def safe_distance_value(self, value: float, fallback: float) -> float:
        if math.isinf(value) or math.isnan(value):
            return float(fallback)
        return float(value)

    def get_sector_min_distance(
        self,
        scan: LaserScan,
        angle_start: float,
        angle_end: float
    ) -> float:
        values: List[float] = []

        total_ranges = len(scan.ranges)
        if total_ranges == 0:
            return float('inf')

        start_idx = self.angle_to_index(scan, angle_start)
        end_idx = self.angle_to_index(scan, angle_end)

        start_idx = max(0, min(start_idx, total_ranges - 1))
        end_idx = max(0, min(end_idx, total_ranges - 1))

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        for i in range(start_idx, end_idx + 1):
            r = scan.ranges[i]

            if math.isnan(r):
                continue

            if math.isinf(r):
                if self.use_inf_as_free:
                    continue
                values.append(scan.range_max)
                continue

            if r < scan.range_min or r > scan.range_max:
                continue

            values.append(r)

        if not values:
            return float('inf')

        return min(values)

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0:
            return 0
        return int(round((angle_rad - scan.angle_min) / scan.angle_increment))

    def format_distance(self, value: float) -> str:
        if math.isinf(value):
            return 'inf'
        if math.isnan(value):
            return 'nan'
        return f'{value:.2f}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SectorAnalyzerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
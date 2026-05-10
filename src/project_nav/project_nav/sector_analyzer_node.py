import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, Range
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - depends on the ROS installation.
    point_cloud2 = None


SectorDistances = Tuple[float, float, float]


class SectorAnalyzerNode(Node):
    def __init__(self) -> None:
        super().__init__('sector_analyzer_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('radar_topic', '/radar/points')
        self.declare_parameter('depth_points_topic', '/camera/depth/color/points')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/front')
        self.declare_parameter('output_topic', '/free_sectors')
        self.declare_parameter('distance_topic', '/sector_distances')

        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_radar', True)
        self.declare_parameter('enable_depth_camera', True)
        self.declare_parameter('enable_ultrasonic', True)

        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_outer_angle_deg', 90.0)

        self.declare_parameter('front_safe_distance', 0.8)
        self.declare_parameter('side_safe_distance', 0.6)

        self.declare_parameter('use_inf_as_free', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('warn_if_no_sensor_data', True)

        self.declare_parameter('radar_min_range', 0.2)
        self.declare_parameter('radar_max_range', 15.0)
        self.declare_parameter('radar_min_z', -0.5)
        self.declare_parameter('radar_max_z', 1.5)
        self.declare_parameter('depth_min_range', 0.2)
        self.declare_parameter('depth_max_range', 8.0)
        self.declare_parameter('depth_min_height', -0.4)
        self.declare_parameter('depth_max_height', 1.2)
        self.declare_parameter('fallback_range', 3.5)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.radar_topic = str(self.get_parameter('radar_topic').value)
        self.depth_points_topic = str(self.get_parameter('depth_points_topic').value)
        self.ultrasonic_topic = str(self.get_parameter('ultrasonic_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)

        self.enable_lidar = bool(self.get_parameter('enable_lidar').value)
        self.enable_radar = bool(self.get_parameter('enable_radar').value)
        self.enable_depth_camera = bool(
            self.get_parameter('enable_depth_camera').value
        )
        self.enable_ultrasonic = bool(self.get_parameter('enable_ultrasonic').value)

        self.front_half_angle_deg = float(self.get_parameter('front_half_angle_deg').value)
        self.side_outer_angle_deg = float(self.get_parameter('side_outer_angle_deg').value)

        self.front_safe_distance = float(self.get_parameter('front_safe_distance').value)
        self.side_safe_distance = float(self.get_parameter('side_safe_distance').value)

        self.use_inf_as_free = bool(self.get_parameter('use_inf_as_free').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.warn_if_no_sensor_data = bool(self.get_parameter('warn_if_no_sensor_data').value)

        self.radar_min_range = float(self.get_parameter('radar_min_range').value)
        self.radar_max_range = float(self.get_parameter('radar_max_range').value)
        self.radar_min_z = float(self.get_parameter('radar_min_z').value)
        self.radar_max_z = float(self.get_parameter('radar_max_z').value)
        self.depth_min_range = float(self.get_parameter('depth_min_range').value)
        self.depth_max_range = float(self.get_parameter('depth_max_range').value)
        self.depth_min_height = float(self.get_parameter('depth_min_height').value)
        self.depth_max_height = float(self.get_parameter('depth_max_height').value)
        self.fallback_range = float(self.get_parameter('fallback_range').value)

        self.free_pub = self.create_publisher(Int32MultiArray, self.output_topic, 10)
        self.distance_pub = self.create_publisher(Float32MultiArray, self.distance_topic, 10)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_radar: Optional[PointCloud2] = None
        self.latest_depth: Optional[PointCloud2] = None
        self.latest_ultrasonic: Optional[Range] = None

        self.last_scan_time = None
        self.last_radar_time = None
        self.last_depth_time = None
        self.last_ultrasonic_time = None

        if self.enable_lidar:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        if self.enable_radar and point_cloud2 is not None:
            self.create_subscription(PointCloud2, self.radar_topic, self.radar_callback, 10)
        elif self.enable_radar:
            self.get_logger().warn('Radar input disabled: sensor_msgs_py is not available')

        if self.enable_depth_camera and point_cloud2 is not None:
            self.create_subscription(
                PointCloud2,
                self.depth_points_topic,
                self.depth_callback,
                10
            )
        elif self.enable_depth_camera:
            self.get_logger().warn(
                'Depth-camera input disabled: sensor_msgs_py is not available'
            )

        if self.enable_ultrasonic:
            self.create_subscription(Range, self.ultrasonic_topic, self.ultrasonic_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sector_info)
        self._no_sensor_warned = False

        self.get_logger().info(
            'Sector analyzer started '
            f'(lidar={self.enable_lidar}:{self.scan_topic}, '
            f'radar={self.enable_radar}:{self.radar_topic}, '
            f'depth={self.enable_depth_camera}:{self.depth_points_topic}, '
            f'ultrasonic={self.enable_ultrasonic}:{self.ultrasonic_topic})'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()
        self._no_sensor_warned = False

    def radar_callback(self, msg: PointCloud2) -> None:
        self.latest_radar = msg
        self.last_radar_time = self.get_clock().now()
        self._no_sensor_warned = False

    def depth_callback(self, msg: PointCloud2) -> None:
        self.latest_depth = msg
        self.last_depth_time = self.get_clock().now()
        self._no_sensor_warned = False

    def ultrasonic_callback(self, msg: Range) -> None:
        self.latest_ultrasonic = msg
        self.last_ultrasonic_time = self.get_clock().now()
        self._no_sensor_warned = False

    def publish_sector_info(self) -> None:
        source_distances: List[SectorDistances] = []
        active_sources: List[str] = []

        if self.is_fresh(self.last_scan_time) and self.latest_scan is not None:
            source_distances.append(self.distances_from_scan(self.latest_scan))
            active_sources.append('lidar')

        if self.is_fresh(self.last_radar_time) and self.latest_radar is not None:
            source_distances.append(self.distances_from_radar(self.latest_radar))
            active_sources.append('radar')

        if self.is_fresh(self.last_depth_time) and self.latest_depth is not None:
            source_distances.append(self.distances_from_depth(self.latest_depth))
            active_sources.append('depth')

        if self.is_fresh(self.last_ultrasonic_time) and self.latest_ultrasonic is not None:
            source_distances.append(self.distances_from_ultrasonic(self.latest_ultrasonic))
            active_sources.append('ultrasonic')

        if not source_distances:
            self.publish_blocked()
            if self.warn_if_no_sensor_data and not self._no_sensor_warned:
                self.get_logger().warn('No fresh ranging data, publishing blocked sectors')
                self._no_sensor_warned = True
            return

        left_min, center_min, right_min = self.fuse_distances(source_distances)

        free_msg = Int32MultiArray()
        free_msg.data = [
            int(self.is_free(left_min, self.side_safe_distance)),
            int(self.is_free(center_min, self.front_safe_distance)),
            int(self.is_free(right_min, self.side_safe_distance)),
        ]
        self.free_pub.publish(free_msg)

        distance_msg = Float32MultiArray()
        distance_msg.data = [
            self.safe_distance_value(left_min),
            self.safe_distance_value(center_min),
            self.safe_distance_value(right_min),
        ]
        self.distance_pub.publish(distance_msg)

        self.get_logger().info(
            'Sectors: '
            f'left={self.format_distance(left_min)}, '
            f'center={self.format_distance(center_min)}, '
            f'right={self.format_distance(right_min)} '
            f'-> free={free_msg.data}, sources={active_sources}'
        )

    def publish_blocked(self) -> None:
        free_msg = Int32MultiArray()
        free_msg.data = [0, 0, 0]
        self.free_pub.publish(free_msg)

        distance_msg = Float32MultiArray()
        distance_msg.data = [0.0, 0.0, 0.0]
        self.distance_pub.publish(distance_msg)

    def distances_from_scan(self, scan: LaserScan) -> SectorDistances:
        left_min = self.get_scan_sector_min_distance(
            scan,
            math.radians(self.front_half_angle_deg),
            math.radians(self.side_outer_angle_deg),
        )
        center_min = self.get_scan_sector_min_distance(
            scan,
            -math.radians(self.front_half_angle_deg),
            math.radians(self.front_half_angle_deg),
        )
        right_min = self.get_scan_sector_min_distance(
            scan,
            -math.radians(self.side_outer_angle_deg),
            -math.radians(self.front_half_angle_deg),
        )

        return left_min, center_min, right_min

    def distances_from_radar(self, cloud: PointCloud2) -> SectorDistances:
        left_min = float('inf')
        center_min = float('inf')
        right_min = float('inf')

        if point_cloud2 is None:
            return left_min, center_min, right_min

        for point in point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if z < self.radar_min_z or z > self.radar_max_z:
                continue

            distance = math.hypot(x, y)
            if distance < self.radar_min_range or distance > self.radar_max_range:
                continue

            angle = math.atan2(y, x)
            sector = self.angle_to_sector(angle)
            if sector == 'left':
                left_min = min(left_min, distance)
            elif sector == 'center':
                center_min = min(center_min, distance)
            elif sector == 'right':
                right_min = min(right_min, distance)

        return left_min, center_min, right_min

    def distances_from_depth(self, cloud: PointCloud2) -> SectorDistances:
        left_min = float('inf')
        center_min = float('inf')
        right_min = float('inf')

        if point_cloud2 is None:
            return left_min, center_min, right_min

        for point in point_cloud2.read_points(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            # ROS optical frame convention: x right, y down, z forward.
            forward = float(point[2])
            left = -float(point[0])
            height = -float(point[1])

            if height < self.depth_min_height or height > self.depth_max_height:
                continue

            distance = math.hypot(forward, left)
            if distance < self.depth_min_range or distance > self.depth_max_range:
                continue

            angle = math.atan2(left, forward)
            sector = self.angle_to_sector(angle)
            if sector == 'left':
                left_min = min(left_min, distance)
            elif sector == 'center':
                center_min = min(center_min, distance)
            elif sector == 'right':
                right_min = min(right_min, distance)

        return left_min, center_min, right_min

    def distances_from_ultrasonic(self, msg: Range) -> SectorDistances:
        if math.isnan(msg.range):
            return float('inf'), float('inf'), float('inf')

        if math.isinf(msg.range):
            center = float('inf') if self.use_inf_as_free else msg.max_range
        elif msg.range < msg.min_range or msg.range > msg.max_range:
            center = float('inf')
        else:
            center = float(msg.range)

        return float('inf'), center, float('inf')

    def fuse_distances(self, distances: List[SectorDistances]) -> SectorDistances:
        left = min(d[0] for d in distances)
        center = min(d[1] for d in distances)
        right = min(d[2] for d in distances)
        return left, center, right

    def is_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        dt = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return dt <= self.sensor_timeout

    def is_free(self, min_distance: float, threshold: float) -> bool:
        if math.isinf(min_distance):
            return self.use_inf_as_free
        return min_distance > threshold

    def safe_distance_value(self, value: float) -> float:
        if math.isinf(value) or math.isnan(value):
            return self.fallback_range
        return float(value)

    def get_scan_sector_min_distance(
        self,
        scan: LaserScan,
        angle_start: float,
        angle_end: float,
    ) -> float:
        values: List[float] = []

        total_ranges = len(scan.ranges)
        if total_ranges == 0:
            return float('inf')

        start_idx = self.scan_angle_to_index(scan, angle_start)
        end_idx = self.scan_angle_to_index(scan, angle_end)

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

            values.append(float(r))

        if not values:
            return float('inf')

        return min(values)

    def angle_to_sector(self, angle_rad: float) -> Optional[str]:
        front_half = math.radians(self.front_half_angle_deg)
        side_outer = math.radians(self.side_outer_angle_deg)

        if -front_half <= angle_rad <= front_half:
            return 'center'
        if front_half < angle_rad <= side_outer:
            return 'left'
        if -side_outer <= angle_rad < -front_half:
            return 'right'
        return None

    def scan_angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
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

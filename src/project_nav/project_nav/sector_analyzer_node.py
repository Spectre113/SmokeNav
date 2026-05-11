import json
import math
from collections import defaultdict
from dataclasses import dataclass
from typing import DefaultDict, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, Range
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray, String

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - depends on the ROS installation.
    point_cloud2 = None


SectorDistances = Tuple[float, float, float]


@dataclass(frozen=True)
class Observation:
    source: str
    x: float
    y: float
    z: float
    distance: float
    angle: float


class SectorAnalyzerNode(Node):
    def __init__(self) -> None:
        super().__init__('sector_analyzer_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('radar_topic', '/radar/points')
        self.declare_parameter('depth_points_topic', '/camera/depth/color/points')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/front')
        self.declare_parameter('output_topic', '/free_sectors')
        self.declare_parameter('distance_topic', '/sector_distances')
        self.declare_parameter('detailed_output_topic', '/free_sectors_detailed')
        self.declare_parameter('detailed_distance_topic', '/sector_distances_detailed')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('metrics_topic', '/sensor_fusion_metrics')
        self.declare_parameter('smoke_density_topic', '/smoke/density')

        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_radar', True)
        self.declare_parameter('enable_depth_camera', True)
        self.declare_parameter('enable_ultrasonic', True)

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('enable_tf_transform', True)
        self.declare_parameter('allow_tf_fallback', False)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('side_outer_angle_deg', 90.0)
        self.declare_parameter('num_detailed_sectors', 9)

        self.declare_parameter('front_safe_distance', 0.65)
        self.declare_parameter('side_safe_distance', 0.25)
        self.declare_parameter('detailed_safe_distance', 0.35)

        self.declare_parameter('fusion_percentile', 35.0)
        self.declare_parameter('source_percentile', 25.0)
        self.declare_parameter('lidar_min_support', 1)
        self.declare_parameter('radar_min_support', 2)
        self.declare_parameter('depth_min_support', 4)
        self.declare_parameter('ultrasonic_min_support', 1)

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
        self.declare_parameter('max_cloud_points', 5000)

        self.declare_parameter('publish_costmap', True)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width_m', 5.0)
        self.declare_parameter('costmap_height_m', 5.0)
        self.declare_parameter('costmap_origin_x', -0.8)
        self.declare_parameter('costmap_origin_y', -2.5)
        self.declare_parameter('costmap_inflation_radius', 0.16)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.radar_topic = str(self.get_parameter('radar_topic').value)
        self.depth_points_topic = str(self.get_parameter('depth_points_topic').value)
        self.ultrasonic_topic = str(self.get_parameter('ultrasonic_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)
        self.detailed_output_topic = str(self.get_parameter('detailed_output_topic').value)
        self.detailed_distance_topic = str(self.get_parameter('detailed_distance_topic').value)
        self.costmap_topic = str(self.get_parameter('costmap_topic').value)
        self.metrics_topic = str(self.get_parameter('metrics_topic').value)
        self.smoke_density_topic = str(self.get_parameter('smoke_density_topic').value)

        self.enable_lidar = bool(self.get_parameter('enable_lidar').value)
        self.enable_radar = bool(self.get_parameter('enable_radar').value)
        self.enable_depth_camera = bool(self.get_parameter('enable_depth_camera').value)
        self.enable_ultrasonic = bool(self.get_parameter('enable_ultrasonic').value)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.enable_tf_transform = bool(self.get_parameter('enable_tf_transform').value)
        self.allow_tf_fallback = bool(self.get_parameter('allow_tf_fallback').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.front_half_angle_deg = float(self.get_parameter('front_half_angle_deg').value)
        self.side_outer_angle_deg = float(self.get_parameter('side_outer_angle_deg').value)
        self.num_detailed_sectors = max(3, int(self.get_parameter('num_detailed_sectors').value))
        if self.num_detailed_sectors % 2 == 0:
            self.num_detailed_sectors += 1

        self.front_safe_distance = float(self.get_parameter('front_safe_distance').value)
        self.side_safe_distance = float(self.get_parameter('side_safe_distance').value)
        self.detailed_safe_distance = float(self.get_parameter('detailed_safe_distance').value)

        self.fusion_percentile = float(self.get_parameter('fusion_percentile').value)
        self.source_percentile = float(self.get_parameter('source_percentile').value)
        self.min_support = {
            'lidar': int(self.get_parameter('lidar_min_support').value),
            'radar': int(self.get_parameter('radar_min_support').value),
            'depth': int(self.get_parameter('depth_min_support').value),
            'ultrasonic': int(self.get_parameter('ultrasonic_min_support').value),
        }

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
        self.max_cloud_points = int(self.get_parameter('max_cloud_points').value)

        self.publish_costmap = bool(self.get_parameter('publish_costmap').value)
        self.costmap_resolution = float(self.get_parameter('costmap_resolution').value)
        self.costmap_width_m = float(self.get_parameter('costmap_width_m').value)
        self.costmap_height_m = float(self.get_parameter('costmap_height_m').value)
        self.costmap_origin_x = float(self.get_parameter('costmap_origin_x').value)
        self.costmap_origin_y = float(self.get_parameter('costmap_origin_y').value)
        self.costmap_inflation_radius = float(
            self.get_parameter('costmap_inflation_radius').value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_warned: set[Tuple[str, str]] = set()

        self.free_pub = self.create_publisher(Int32MultiArray, self.output_topic, 10)
        self.distance_pub = self.create_publisher(Float32MultiArray, self.distance_topic, 10)
        self.detailed_free_pub = self.create_publisher(
            Int32MultiArray,
            self.detailed_output_topic,
            10,
        )
        self.detailed_distance_pub = self.create_publisher(
            Float32MultiArray,
            self.detailed_distance_topic,
            10,
        )
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, 2)
        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_radar: Optional[PointCloud2] = None
        self.latest_depth: Optional[PointCloud2] = None
        self.latest_ultrasonic: Optional[Range] = None
        self.latest_smoke_density: Optional[float] = None

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
                10,
            )
        elif self.enable_depth_camera:
            self.get_logger().warn(
                'Depth-camera input disabled: sensor_msgs_py is not available'
            )

        if self.enable_ultrasonic:
            self.create_subscription(
                Range,
                self.ultrasonic_topic,
                self.ultrasonic_callback,
                10,
            )

        self.create_subscription(
            Float32,
            self.smoke_density_topic,
            self.smoke_density_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sector_info)
        self._no_sensor_warned = False

        self.get_logger().info(
            'Sector analyzer started with TF, robust fusion, detailed sectors, and costmap '
            f'(base_frame={self.base_frame}, sectors={self.num_detailed_sectors}, '
            f'lidar={self.enable_lidar}:{self.scan_topic}, '
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

    def smoke_density_callback(self, msg: Float32) -> None:
        self.latest_smoke_density = float(msg.data)

    def publish_sector_info(self) -> None:
        observations: List[Observation] = []
        source_counts: Dict[str, int] = {}

        self.extend_observations(
            observations,
            source_counts,
            'lidar',
            self.last_scan_time,
            self.latest_scan,
            self.observations_from_scan,
        )
        self.extend_observations(
            observations,
            source_counts,
            'radar',
            self.last_radar_time,
            self.latest_radar,
            self.observations_from_radar,
        )
        self.extend_observations(
            observations,
            source_counts,
            'depth',
            self.last_depth_time,
            self.latest_depth,
            self.observations_from_depth,
        )
        self.extend_observations(
            observations,
            source_counts,
            'ultrasonic',
            self.last_ultrasonic_time,
            self.latest_ultrasonic,
            self.observations_from_ultrasonic,
        )

        if not observations:
            self.publish_blocked()
            if self.warn_if_no_sensor_data and not self._no_sensor_warned:
                self.get_logger().warn('No fresh ranging data, publishing blocked sectors')
                self._no_sensor_warned = True
            return

        left_min, center_min, right_min = self.legacy_distances_from_observations(observations)
        detailed_distances = self.detailed_distances_from_observations(observations)

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

        detailed_free_msg = Int32MultiArray()
        detailed_free_msg.data = [
            int(self.is_free(distance, self.detailed_safe_distance))
            for distance in detailed_distances
        ]
        self.detailed_free_pub.publish(detailed_free_msg)

        detailed_distance_msg = Float32MultiArray()
        detailed_distance_msg.data = [self.safe_distance_value(d) for d in detailed_distances]
        self.detailed_distance_pub.publish(detailed_distance_msg)

        occupied_cells = 0
        if self.publish_costmap:
            occupied_cells = self.publish_local_costmap(observations)

        min_clearance = min(obs.distance for obs in observations)
        self.publish_metrics(
            source_counts,
            free_msg.data,
            distance_msg.data,
            detailed_distance_msg.data,
            min_clearance,
            occupied_cells,
        )

        self.get_logger().info(
            'Sectors: '
            f'left={self.format_distance(left_min)}, '
            f'center={self.format_distance(center_min)}, '
            f'right={self.format_distance(right_min)} '
            f'-> free={free_msg.data}, detailed_free={detailed_free_msg.data}, '
            f'sources={list(source_counts.keys())}'
        )

    def extend_observations(
        self,
        observations: List[Observation],
        source_counts: Dict[str, int],
        source: str,
        stamp,
        msg,
        builder,
    ) -> None:
        if msg is None or not self.is_fresh(stamp):
            return
        items = list(builder(msg))
        if items:
            observations.extend(items)
            source_counts[source] = len(items)

    def publish_blocked(self) -> None:
        free_msg = Int32MultiArray()
        free_msg.data = [0, 0, 0]
        self.free_pub.publish(free_msg)

        distance_msg = Float32MultiArray()
        distance_msg.data = [0.0, 0.0, 0.0]
        self.distance_pub.publish(distance_msg)

        detailed_free_msg = Int32MultiArray()
        detailed_free_msg.data = [0] * self.num_detailed_sectors
        self.detailed_free_pub.publish(detailed_free_msg)

        detailed_distance_msg = Float32MultiArray()
        detailed_distance_msg.data = [0.0] * self.num_detailed_sectors
        self.detailed_distance_pub.publish(detailed_distance_msg)

    def observations_from_scan(self, scan: LaserScan) -> Iterable[Observation]:
        total_ranges = len(scan.ranges)
        if total_ranges == 0:
            return []

        observations: List[Observation] = []
        angle = scan.angle_min
        for raw_range in scan.ranges:
            if math.isnan(raw_range):
                angle += scan.angle_increment
                continue
            if math.isinf(raw_range):
                angle += scan.angle_increment
                continue
            if raw_range < scan.range_min or raw_range > scan.range_max:
                angle += scan.angle_increment
                continue

            x = float(raw_range) * math.cos(angle)
            y = float(raw_range) * math.sin(angle)
            obs = self.make_observation(
                'lidar',
                x,
                y,
                0.0,
                scan.header.frame_id,
                scan.header.stamp,
            )
            if obs is not None:
                observations.append(obs)
            angle += scan.angle_increment

        return observations

    def observations_from_radar(self, cloud: PointCloud2) -> Iterable[Observation]:
        return self.observations_from_cloud(
            cloud,
            'radar',
            self.radar_min_range,
            self.radar_max_range,
            self.radar_min_z,
            self.radar_max_z,
        )

    def observations_from_depth(self, cloud: PointCloud2) -> Iterable[Observation]:
        return self.observations_from_cloud(
            cloud,
            'depth',
            self.depth_min_range,
            self.depth_max_range,
            self.depth_min_height,
            self.depth_max_height,
        )

    def observations_from_cloud(
        self,
        cloud: PointCloud2,
        source: str,
        min_range: float,
        max_range: float,
        min_z: float,
        max_z: float,
    ) -> List[Observation]:
        observations: List[Observation] = []
        if point_cloud2 is None:
            return observations

        for index, point in enumerate(self.iter_cloud_points(cloud)):
            if index >= self.max_cloud_points:
                break

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            obs = self.make_observation(source, x, y, z, cloud.header.frame_id, cloud.header.stamp)
            if obs is None:
                continue
            if obs.z < min_z or obs.z > max_z:
                continue
            if obs.distance < min_range or obs.distance > max_range:
                continue
            observations.append(obs)

        return observations

    def iter_cloud_points(self, cloud: PointCloud2):
        if point_cloud2 is None:
            return []

        if cloud.height > 1 and cloud.width > 0 and self.max_cloud_points > 0:
            total_points = int(cloud.width * cloud.height)
            step = max(1, int(math.sqrt(total_points / max(self.max_cloud_points, 1))))
            uvs = [
                (u, v)
                for v in range(0, cloud.height, step)
                for u in range(0, cloud.width, step)
            ]
            return point_cloud2.read_points(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
                uvs=uvs,
            )

        return point_cloud2.read_points(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

    def observations_from_ultrasonic(self, msg: Range) -> Iterable[Observation]:
        if math.isnan(msg.range):
            return []
        if math.isinf(msg.range):
            if self.use_inf_as_free:
                return []
            distance = msg.max_range
        elif msg.range < msg.min_range or msg.range > msg.max_range:
            return []
        else:
            distance = float(msg.range)

        obs = self.make_observation(
            'ultrasonic',
            distance,
            0.0,
            0.0,
            msg.header.frame_id,
            msg.header.stamp,
        )
        return [obs] if obs is not None else []

    def make_observation(
        self,
        source: str,
        x: float,
        y: float,
        z: float,
        frame_id: str,
        stamp,
    ) -> Optional[Observation]:
        transformed = self.transform_point(x, y, z, frame_id, stamp)
        if transformed is None:
            return None

        base_x, base_y, base_z = transformed
        distance = math.hypot(base_x, base_y)
        if distance <= 1e-6:
            return None

        angle = math.atan2(base_y, base_x)
        if abs(angle) > math.radians(self.side_outer_angle_deg):
            return None

        return Observation(source, base_x, base_y, base_z, distance, angle)

    def transform_point(
        self,
        x: float,
        y: float,
        z: float,
        frame_id: str,
        stamp,
    ) -> Optional[Tuple[float, float, float]]:
        if not self.enable_tf_transform or not frame_id or frame_id == self.base_frame:
            return x, y, z

        key = (frame_id, self.base_frame)
        try:
            stamp_time = Time.from_msg(stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                stamp_time,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exc:  # tf2 raises several concrete exception types.
            if key not in self._tf_warned:
                self.get_logger().warn(
                    f'TF unavailable {frame_id}->{self.base_frame}: {exc}. '
                    f'fallback={self.allow_tf_fallback}'
                )
                self._tf_warned.add(key)
            if self.allow_tf_fallback:
                return x, y, z
            return None

        translation = transform.transform.translation
        rotated = self.rotate_vector(transform.transform.rotation, x, y, z)
        return (
            rotated[0] + translation.x,
            rotated[1] + translation.y,
            rotated[2] + translation.z,
        )

    def rotate_vector(self, q, x: float, y: float, z: float) -> Tuple[float, float, float]:
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-9:
            return x, y, z
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z

        return (
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
        )

    def legacy_distances_from_observations(
        self,
        observations: List[Observation],
    ) -> SectorDistances:
        buckets: Dict[str, DefaultDict[str, List[float]]] = {
            'left': defaultdict(list),
            'center': defaultdict(list),
            'right': defaultdict(list),
        }

        for obs in observations:
            sector = self.angle_to_legacy_sector(obs.angle)
            if sector is not None:
                buckets[sector][obs.source].append(obs.distance)

        return (
            self.fuse_bucket(buckets['left']),
            self.fuse_bucket(buckets['center']),
            self.fuse_bucket(buckets['right']),
        )

    def detailed_distances_from_observations(self, observations: List[Observation]) -> List[float]:
        buckets: List[DefaultDict[str, List[float]]] = [
            defaultdict(list) for _ in range(self.num_detailed_sectors)
        ]

        for obs in observations:
            index = self.angle_to_detailed_index(obs.angle)
            if index is not None:
                buckets[index][obs.source].append(obs.distance)

        return [self.fuse_bucket(bucket) for bucket in buckets]

    def fuse_bucket(self, source_values: DefaultDict[str, List[float]]) -> float:
        estimates: List[float] = []
        for source, values in source_values.items():
            support = max(1, self.min_support.get(source, 1))
            if len(values) < support:
                continue
            estimates.append(self.percentile(values, self.source_percentile))

        if not estimates:
            return float('inf')
        return self.percentile(estimates, self.fusion_percentile)

    def percentile(self, values: List[float], percentile: float) -> float:
        clean = sorted(v for v in values if math.isfinite(v))
        if not clean:
            return float('inf')
        if len(clean) == 1:
            return clean[0]
        pct = self.clamp(percentile, 0.0, 100.0) / 100.0
        index = int(round(pct * (len(clean) - 1)))
        return clean[index]

    def angle_to_legacy_sector(self, angle_rad: float) -> Optional[str]:
        front_half = math.radians(self.front_half_angle_deg)
        side_outer = math.radians(self.side_outer_angle_deg)

        if -front_half <= angle_rad <= front_half:
            return 'center'
        if front_half < angle_rad <= side_outer:
            return 'left'
        if -side_outer <= angle_rad < -front_half:
            return 'right'
        return None

    def angle_to_detailed_index(self, angle_rad: float) -> Optional[int]:
        side_outer = math.radians(self.side_outer_angle_deg)
        if angle_rad < -side_outer or angle_rad > side_outer:
            return None
        normalized = (angle_rad + side_outer) / (2.0 * side_outer)
        index = int(normalized * self.num_detailed_sectors)
        return max(0, min(self.num_detailed_sectors - 1, index))

    def publish_local_costmap(self, observations: List[Observation]) -> int:
        resolution = max(self.costmap_resolution, 1e-3)
        width = max(1, int(round(self.costmap_width_m / resolution)))
        height = max(1, int(round(self.costmap_height_m / resolution)))
        data = [0] * (width * height)
        inflation_cells = max(0, int(math.ceil(self.costmap_inflation_radius / resolution)))
        occupied_cells = 0

        for obs in observations:
            gx = int((obs.x - self.costmap_origin_x) / resolution)
            gy = int((obs.y - self.costmap_origin_y) / resolution)
            if gx < 0 or gx >= width or gy < 0 or gy >= height:
                continue
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy > inflation_cells * inflation_cells:
                        continue
                    ix = gx + dx
                    iy = gy + dy
                    if ix < 0 or ix >= width or iy < 0 or iy >= height:
                        continue
                    cell = iy * width + ix
                    if data[cell] != 100:
                        occupied_cells += 1
                    data[cell] = 100

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.base_frame
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = self.costmap_origin_x
        grid.info.origin.position.y = self.costmap_origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self.costmap_pub.publish(grid)
        return occupied_cells

    def publish_metrics(
        self,
        source_counts: Dict[str, int],
        free: List[int],
        distances: List[float],
        detailed_distances: List[float],
        min_clearance: float,
        occupied_cells: int,
    ) -> None:
        metrics = {
            'active_sources': sorted(source_counts.keys()),
            'source_counts': {
                str(source): int(count)
                for source, count in source_counts.items()
            },
            'free_sectors': [int(value) for value in free],
            'sector_distances_m': [float(value) for value in distances],
            'detailed_sector_count': self.num_detailed_sectors,
            'detailed_sector_distances_m': [
                float(value) for value in detailed_distances
            ],
            'min_clearance_m': float(min_clearance),
            'occupied_cells': int(occupied_cells),
            'smoke_density': self.latest_smoke_density,
            'frame': self.base_frame,
        }
        msg = String()
        msg.data = json.dumps(metrics, separators=(',', ':'))
        self.metrics_pub.publish(msg)

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

    def format_distance(self, value: float) -> str:
        if math.isinf(value):
            return 'inf'
        if math.isnan(value):
            return 'nan'
        return f'{value:.2f}'

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


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

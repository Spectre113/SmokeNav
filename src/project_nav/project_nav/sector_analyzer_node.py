import json
import math
import heapq
import random
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import DefaultDict, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
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
    occupied: bool = True


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
        self.declare_parameter('global_map_topic', '/global_map')
        self.declare_parameter('global_path_topic', '/exploration_path')
        self.declare_parameter('exploration_hint_topic', '/exploration_hint')
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
        self.declare_parameter('smoke_effect_seed', 4242)
        self.declare_parameter('smoke_affects_radar', True)
        self.declare_parameter('smoke_affects_depth', True)
        self.declare_parameter('smoke_affects_ultrasonic', True)
        self.declare_parameter('radar_smoke_range_scale', 0.25)
        self.declare_parameter('radar_smoke_noise_std', 0.04)
        self.declare_parameter('radar_smoke_dropout', 0.10)
        self.declare_parameter('depth_smoke_range_scale', 0.55)
        self.declare_parameter('depth_smoke_noise_std', 0.08)
        self.declare_parameter('depth_smoke_dropout', 0.30)
        self.declare_parameter('ultrasonic_smoke_range_scale', 0.35)
        self.declare_parameter('ultrasonic_smoke_noise_std', 0.03)
        self.declare_parameter('ultrasonic_smoke_dropout', 0.12)

        self.declare_parameter('publish_costmap', True)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('costmap_width_m', 5.0)
        self.declare_parameter('costmap_height_m', 5.0)
        self.declare_parameter('costmap_origin_x', -0.8)
        self.declare_parameter('costmap_origin_y', -2.5)
        self.declare_parameter('costmap_inflation_radius', 0.16)

        self.declare_parameter('publish_global_map', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('global_map_resolution', 0.10)
        self.declare_parameter('global_map_width_m', 30.0)
        self.declare_parameter('global_map_height_m', 30.0)
        self.declare_parameter('global_map_origin_x', -15.0)
        self.declare_parameter('global_map_origin_y', -15.0)
        self.declare_parameter('global_hit_log_odds', 0.85)
        self.declare_parameter('global_miss_log_odds', -0.35)
        self.declare_parameter('global_min_log_odds', -3.5)
        self.declare_parameter('global_max_log_odds', 3.5)
        self.declare_parameter('global_unknown_log_odds', 0.20)
        self.declare_parameter('max_free_ray_range', 4.5)
        self.declare_parameter('frontier_min_distance', 0.9)
        self.declare_parameter('frontier_max_distance', 8.0)
        self.declare_parameter('frontier_max_abs_angle_deg', 170.0)
        self.declare_parameter('frontier_min_cluster_size', 4)
        self.declare_parameter('frontier_cluster_weight', 0.18)
        self.declare_parameter('frontier_cluster_score_cap', 4.0)
        self.declare_parameter('frontier_distance_weight', 0.90)
        self.declare_parameter('frontier_heading_weight', 0.35)
        self.declare_parameter('frontier_unknown_weight', 0.20)
        self.declare_parameter('frontier_current_bonus', 3.0)
        self.declare_parameter('frontier_reached_distance', 0.7)
        self.declare_parameter('frontier_keep_radius', 1.2)
        self.declare_parameter('exploration_path_lookahead_m', 0.85)
        self.declare_parameter('frontier_min_path_distance', 1.2)
        self.declare_parameter('frontier_max_path_distance', 18.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.radar_topic = str(self.get_parameter('radar_topic').value)
        self.depth_points_topic = str(self.get_parameter('depth_points_topic').value)
        self.ultrasonic_topic = str(self.get_parameter('ultrasonic_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)
        self.detailed_output_topic = str(self.get_parameter('detailed_output_topic').value)
        self.detailed_distance_topic = str(self.get_parameter('detailed_distance_topic').value)
        self.costmap_topic = str(self.get_parameter('costmap_topic').value)
        self.global_map_topic = str(self.get_parameter('global_map_topic').value)
        self.global_path_topic = str(self.get_parameter('global_path_topic').value)
        self.exploration_hint_topic = str(self.get_parameter('exploration_hint_topic').value)
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
        self.smoke_effect_seed = int(self.get_parameter('smoke_effect_seed').value)
        self.smoke_affects_radar = bool(self.get_parameter('smoke_affects_radar').value)
        self.smoke_affects_depth = bool(self.get_parameter('smoke_affects_depth').value)
        self.smoke_affects_ultrasonic = bool(
            self.get_parameter('smoke_affects_ultrasonic').value
        )
        self.radar_smoke_range_scale = float(
            self.get_parameter('radar_smoke_range_scale').value
        )
        self.radar_smoke_noise_std = float(
            self.get_parameter('radar_smoke_noise_std').value
        )
        self.radar_smoke_dropout = float(
            self.get_parameter('radar_smoke_dropout').value
        )
        self.depth_smoke_range_scale = float(
            self.get_parameter('depth_smoke_range_scale').value
        )
        self.depth_smoke_noise_std = float(
            self.get_parameter('depth_smoke_noise_std').value
        )
        self.depth_smoke_dropout = float(
            self.get_parameter('depth_smoke_dropout').value
        )
        self.ultrasonic_smoke_range_scale = float(
            self.get_parameter('ultrasonic_smoke_range_scale').value
        )
        self.ultrasonic_smoke_noise_std = float(
            self.get_parameter('ultrasonic_smoke_noise_std').value
        )
        self.ultrasonic_smoke_dropout = float(
            self.get_parameter('ultrasonic_smoke_dropout').value
        )

        self.publish_costmap = bool(self.get_parameter('publish_costmap').value)
        self.costmap_resolution = float(self.get_parameter('costmap_resolution').value)
        self.costmap_width_m = float(self.get_parameter('costmap_width_m').value)
        self.costmap_height_m = float(self.get_parameter('costmap_height_m').value)
        self.costmap_origin_x = float(self.get_parameter('costmap_origin_x').value)
        self.costmap_origin_y = float(self.get_parameter('costmap_origin_y').value)
        self.costmap_inflation_radius = float(
            self.get_parameter('costmap_inflation_radius').value
        )

        self.publish_global_map = bool(self.get_parameter('publish_global_map').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.global_map_resolution = float(self.get_parameter('global_map_resolution').value)
        self.global_map_width_m = float(self.get_parameter('global_map_width_m').value)
        self.global_map_height_m = float(self.get_parameter('global_map_height_m').value)
        self.global_map_origin_x = float(self.get_parameter('global_map_origin_x').value)
        self.global_map_origin_y = float(self.get_parameter('global_map_origin_y').value)
        self.global_hit_log_odds = float(self.get_parameter('global_hit_log_odds').value)
        self.global_miss_log_odds = float(self.get_parameter('global_miss_log_odds').value)
        self.global_min_log_odds = float(self.get_parameter('global_min_log_odds').value)
        self.global_max_log_odds = float(self.get_parameter('global_max_log_odds').value)
        self.global_unknown_log_odds = float(
            self.get_parameter('global_unknown_log_odds').value
        )
        self.max_free_ray_range = float(self.get_parameter('max_free_ray_range').value)
        self.frontier_min_distance = float(self.get_parameter('frontier_min_distance').value)
        self.frontier_max_distance = float(self.get_parameter('frontier_max_distance').value)
        self.frontier_max_abs_angle_deg = float(
            self.get_parameter('frontier_max_abs_angle_deg').value
        )
        self.frontier_min_cluster_size = int(
            self.get_parameter('frontier_min_cluster_size').value
        )
        self.frontier_cluster_weight = float(
            self.get_parameter('frontier_cluster_weight').value
        )
        self.frontier_cluster_score_cap = float(
            self.get_parameter('frontier_cluster_score_cap').value
        )
        self.frontier_distance_weight = float(
            self.get_parameter('frontier_distance_weight').value
        )
        self.frontier_heading_weight = float(
            self.get_parameter('frontier_heading_weight').value
        )
        self.frontier_unknown_weight = float(
            self.get_parameter('frontier_unknown_weight').value
        )
        self.frontier_current_bonus = float(
            self.get_parameter('frontier_current_bonus').value
        )
        self.frontier_reached_distance = float(
            self.get_parameter('frontier_reached_distance').value
        )
        self.frontier_keep_radius = float(
            self.get_parameter('frontier_keep_radius').value
        )
        self.exploration_path_lookahead_m = float(
            self.get_parameter('exploration_path_lookahead_m').value
        )
        self.frontier_min_path_distance = float(
            self.get_parameter('frontier_min_path_distance').value
        )
        self.frontier_max_path_distance = float(
            self.get_parameter('frontier_max_path_distance').value
        )
        self.global_map_width = max(
            1,
            int(round(self.global_map_width_m / max(self.global_map_resolution, 1e-3))),
        )
        self.global_map_height = max(
            1,
            int(round(self.global_map_height_m / max(self.global_map_resolution, 1e-3))),
        )
        self.global_log_odds = [0.0] * (self.global_map_width * self.global_map_height)
        self.current_frontier_world: Optional[Tuple[float, float]] = None
        self.last_exploration_path_length = 0.0
        self.last_exploration_path_cells = 0
        self._smoke_rng = random.Random(self.smoke_effect_seed)

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
        self.global_map_pub = self.create_publisher(OccupancyGrid, self.global_map_topic, 2)
        self.global_path_pub = self.create_publisher(Path, self.global_path_topic, 2)
        self.exploration_hint_pub = self.create_publisher(
            Float32MultiArray,
            self.exploration_hint_topic,
            10,
        )
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

        global_known_cells = 0
        frontier_distance = 0.0
        if self.publish_global_map:
            robot_pose = self.update_and_publish_global_map(observations)
            global_known_cells = self.count_global_known_cells()
            frontier_distance = self.publish_exploration_hint(robot_pose)

        occupied_observations = [obs for obs in observations if obs.occupied]
        if occupied_observations:
            min_clearance = min(obs.distance for obs in occupied_observations)
        else:
            min_clearance = self.fallback_range
        self.publish_metrics(
            source_counts,
            free_msg.data,
            distance_msg.data,
            detailed_distance_msg.data,
            min_clearance,
            occupied_cells,
            global_known_cells,
            frontier_distance,
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
                if self.use_inf_as_free:
                    distance = min(scan.range_max, self.max_free_ray_range)
                    obs = self.scan_observation(scan, angle, distance, occupied=False)
                    if obs is not None:
                        observations.append(obs)
                angle += scan.angle_increment
                continue
            if raw_range < scan.range_min or raw_range > scan.range_max:
                angle += scan.angle_increment
                continue

            obs = self.scan_observation(scan, angle, float(raw_range), occupied=True)
            if obs is not None:
                observations.append(obs)
            angle += scan.angle_increment

        return observations

    def scan_observation(
        self,
        scan: LaserScan,
        angle: float,
        distance: float,
        occupied: bool,
    ) -> Optional[Observation]:
        x = float(distance) * math.cos(angle)
        y = float(distance) * math.sin(angle)
        return self.make_observation(
            'lidar',
            x,
            y,
            0.0,
            scan.header.frame_id,
            scan.header.stamp,
            occupied=occupied,
        )

    def observations_from_radar(self, cloud: PointCloud2) -> Iterable[Observation]:
        return self.observations_from_cloud(
            cloud,
            'radar',
            self.radar_min_range,
            self.radar_max_range,
            self.radar_min_z,
            self.radar_max_z,
            smoke_enabled=self.smoke_affects_radar,
            smoke_range_scale=self.radar_smoke_range_scale,
            smoke_noise_std=self.radar_smoke_noise_std,
            smoke_dropout=self.radar_smoke_dropout,
        )

    def observations_from_depth(self, cloud: PointCloud2) -> Iterable[Observation]:
        return self.observations_from_cloud(
            cloud,
            'depth',
            self.depth_min_range,
            self.depth_max_range,
            self.depth_min_height,
            self.depth_max_height,
            smoke_enabled=self.smoke_affects_depth,
            smoke_range_scale=self.depth_smoke_range_scale,
            smoke_noise_std=self.depth_smoke_noise_std,
            smoke_dropout=self.depth_smoke_dropout,
        )

    def observations_from_cloud(
        self,
        cloud: PointCloud2,
        source: str,
        min_range: float,
        max_range: float,
        min_z: float,
        max_z: float,
        smoke_enabled: bool,
        smoke_range_scale: float,
        smoke_noise_std: float,
        smoke_dropout: float,
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
            obs = self.apply_smoke_to_observation(
                obs,
                min_range=min_range,
                max_range=max_range,
                smoke_enabled=smoke_enabled,
                range_scale_gain=smoke_range_scale,
                noise_std_gain=smoke_noise_std,
                dropout_gain=smoke_dropout,
            )
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
                distance = min(msg.max_range, self.max_free_ray_range)
                occupied = False
            else:
                distance = msg.max_range
                occupied = True
        elif msg.range < msg.min_range or msg.range > msg.max_range:
            return []
        else:
            distance = float(msg.range)
            occupied = True

        obs = self.make_observation(
            'ultrasonic',
            distance,
            0.0,
            0.0,
            msg.header.frame_id,
            msg.header.stamp,
            occupied=occupied,
        )
        if obs is None:
            return []
        obs = self.apply_smoke_to_observation(
            obs,
            min_range=max(msg.min_range, 0.0),
            max_range=msg.max_range,
            smoke_enabled=self.smoke_affects_ultrasonic,
            range_scale_gain=self.ultrasonic_smoke_range_scale,
            noise_std_gain=self.ultrasonic_smoke_noise_std,
            dropout_gain=self.ultrasonic_smoke_dropout,
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
        occupied: bool = True,
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

        return Observation(source, base_x, base_y, base_z, distance, angle, occupied)

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

    def lookup_transform(self, target_frame: str, source_frame: str, stamp: Time):
        key = (source_frame, target_frame)
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exc:
            if key not in self._tf_warned:
                self.get_logger().warn(
                    f'TF unavailable {source_frame}->{target_frame}: {exc}'
                )
                self._tf_warned.add(key)
            return None

    def apply_transform_to_point(
        self,
        transform,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[float, float, float]:
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

    def quaternion_to_yaw(self, q) -> float:
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def current_smoke_density(self) -> float:
        if self.latest_smoke_density is None:
            return 0.0
        return self.clamp(float(self.latest_smoke_density), 0.0, 1.0)

    def apply_smoke_to_observation(
        self,
        obs: Observation,
        min_range: float,
        max_range: float,
        smoke_enabled: bool,
        range_scale_gain: float,
        noise_std_gain: float,
        dropout_gain: float,
    ) -> Optional[Observation]:
        if not smoke_enabled or not obs.occupied:
            return obs

        density = self.current_smoke_density()
        if density <= 1e-6:
            return obs

        dropout_prob = self.clamp(dropout_gain * density, 0.0, 0.95)
        if self._smoke_rng.random() < dropout_prob:
            return None

        effective_max_range = max(
            min_range,
            max_range * (1.0 - self.clamp(range_scale_gain, 0.0, 0.95) * density),
        )
        if obs.distance > effective_max_range:
            return None

        noise_std = max(0.0, noise_std_gain * density)
        noisy_distance = obs.distance + self._smoke_rng.gauss(0.0, noise_std)
        noisy_distance = self.clamp(noisy_distance, min_range, effective_max_range)

        scale = noisy_distance / max(obs.distance, 1e-6)
        return Observation(
            obs.source,
            obs.x * scale,
            obs.y * scale,
            obs.z * scale,
            noisy_distance,
            obs.angle,
            obs.occupied,
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
            if sector is not None and obs.occupied:
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
            if index is not None and obs.occupied:
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
            if not obs.occupied:
                continue
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

    def update_and_publish_global_map(
        self,
        observations: List[Observation],
    ) -> Optional[Tuple[float, float, float]]:
        transform = self.lookup_transform(self.global_frame, self.base_frame, Time())
        if transform is None:
            self.publish_empty_exploration_hint()
            return None

        robot_x, robot_y, _ = self.apply_transform_to_point(transform, 0.0, 0.0, 0.0)
        robot_yaw = self.quaternion_to_yaw(transform.transform.rotation)
        robot_cell = self.world_to_global_cell(robot_x, robot_y)
        if robot_cell is None:
            self.publish_global_occupancy_grid()
            self.publish_empty_exploration_hint()
            return None

        self.add_global_log_odds(robot_cell[0], robot_cell[1], self.global_miss_log_odds)

        for obs in observations:
            world_x, world_y, _ = self.apply_transform_to_point(
                transform,
                obs.x,
                obs.y,
                obs.z,
            )
            obstacle_cell = self.world_to_global_cell(world_x, world_y)
            if obstacle_cell is None:
                continue

            ray_cells = self.bresenham_cells(
                robot_cell[0],
                robot_cell[1],
                obstacle_cell[0],
                obstacle_cell[1],
            )
            for cell_x, cell_y in ray_cells[:-1]:
                self.add_global_log_odds(cell_x, cell_y, self.global_miss_log_odds)

            if obs.occupied:
                self.add_global_log_odds(
                    obstacle_cell[0],
                    obstacle_cell[1],
                    self.global_hit_log_odds,
                )
            else:
                self.add_global_log_odds(
                    obstacle_cell[0],
                    obstacle_cell[1],
                    self.global_miss_log_odds,
                )

        self.publish_global_occupancy_grid()
        return robot_x, robot_y, robot_yaw

    def publish_exploration_hint(
        self,
        robot_pose: Optional[Tuple[float, float, float]],
    ) -> float:
        if robot_pose is None:
            self.publish_empty_exploration_hint()
            return 0.0

        frontier = self.find_best_frontier(robot_pose)
        if frontier is None:
            self.publish_empty_exploration_hint()
            return 0.0

        angle, distance, score = frontier
        msg = Float32MultiArray()
        msg.data = [1.0, float(angle), float(distance), float(score)]
        self.exploration_hint_pub.publish(msg)
        return float(distance)

    def publish_empty_exploration_hint(self) -> None:
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.exploration_hint_pub.publish(msg)
        self.publish_exploration_path([])
        self.last_exploration_path_length = 0.0
        self.last_exploration_path_cells = 0

    def find_best_frontier(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        robot_x, robot_y, robot_yaw = robot_pose
        robot_cell = self.world_to_global_cell(robot_x, robot_y)
        if robot_cell is None:
            return None

        start_cell = self.nearest_global_free_cell(robot_cell)
        if start_cell is None:
            return None

        reachable_costs, parents = self.compute_reachable_global_cells(start_cell)
        if not reachable_costs:
            return None

        if self.current_frontier_world is not None:
            current_distance = math.hypot(
                self.current_frontier_world[0] - robot_x,
                self.current_frontier_world[1] - robot_y,
            )
            if current_distance < self.frontier_reached_distance:
                self.current_frontier_world = None

        frontier_cells = self.collect_frontier_cells()
        clusters = self.cluster_frontiers(frontier_cells)

        best: Optional[
            Tuple[float, float, float, float, float, List[Tuple[int, int]], float]
        ] = None
        best_score = -float('inf')

        for cluster in clusters:
            if len(cluster) < self.frontier_min_cluster_size:
                continue

            candidate = self.best_reachable_frontier_candidate(
                robot_pose,
                start_cell,
                reachable_costs,
                parents,
                cluster,
            )
            if candidate is None:
                continue

            angle, distance, cell_score, world_x, world_y, path_cells, path_distance = candidate
            cluster_score = min(
                math.sqrt(float(len(cluster))) * self.frontier_cluster_weight,
                self.frontier_cluster_score_cap,
            )
            score = cell_score + cluster_score

            if score > best_score:
                best_score = score
                best = (
                    angle,
                    distance,
                    score,
                    world_x,
                    world_y,
                    path_cells,
                    path_distance,
                )

        if best is None:
            self.current_frontier_world = None
            self.publish_exploration_path([])
            self.last_exploration_path_length = 0.0
            self.last_exploration_path_cells = 0
            return None

        angle, distance, score, world_x, world_y, path_cells, path_distance = best
        self.current_frontier_world = (world_x, world_y)
        self.last_exploration_path_length = path_distance
        self.last_exploration_path_cells = len(path_cells)
        self.publish_exploration_path(path_cells)
        return angle, distance, score

    def best_reachable_frontier_candidate(
        self,
        robot_pose: Tuple[float, float, float],
        start_cell: Tuple[int, int],
        reachable_costs: Dict[Tuple[int, int], float],
        parents: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
        cluster: List[Tuple[int, int]],
    ) -> Optional[Tuple[float, float, float, float, float, List[Tuple[int, int]], float]]:
        robot_x, robot_y, robot_yaw = robot_pose
        best: Optional[
            Tuple[float, float, float, float, float, List[Tuple[int, int]], float]
        ] = None
        best_score = -float('inf')
        max_angle = math.radians(self.frontier_max_abs_angle_deg)
        cluster_set = set(cluster)
        candidate_entries: set[Tuple[int, int]] = set()

        for cell_x, cell_y in cluster:
            for free_cell in self.free_neighbors(cell_x, cell_y):
                if free_cell in reachable_costs:
                    candidate_entries.add(free_cell)

        for entry_cell in candidate_entries:
            path_distance = reachable_costs[entry_cell]
            if path_distance < self.frontier_min_path_distance:
                continue
            if path_distance > self.frontier_max_path_distance:
                continue

            frontier_cell = self.best_frontier_cell_for_entry(entry_cell, cluster_set)
            if frontier_cell is None:
                continue

            cell_x, cell_y = frontier_cell
            path_cells = self.reconstruct_global_path(start_cell, entry_cell, parents)
            if not path_cells:
                continue

            lookahead_x, lookahead_y = self.path_lookahead_world(path_cells)
            heading = math.atan2(lookahead_y - robot_y, lookahead_x - robot_x)
            angle = self.normalize_angle(heading - robot_yaw)
            if abs(angle) > max_angle:
                continue

            world_x, world_y = self.global_cell_to_world(cell_x, cell_y)
            distance = math.hypot(world_x - robot_x, world_y - robot_y)
            if distance < self.frontier_min_distance:
                continue
            if distance > self.frontier_max_distance:
                continue

            unknown_neighbors = self.count_unknown_neighbors(cell_x, cell_y)
            free_neighbors = len(self.free_neighbors(cell_x, cell_y))
            lookahead_distance = math.hypot(lookahead_x - robot_x, lookahead_y - robot_y)
            score = (
                path_distance * self.frontier_distance_weight
                + distance * 0.12
                + unknown_neighbors * self.frontier_unknown_weight
                + free_neighbors * 0.15
                - abs(angle) * self.frontier_heading_weight
            )

            if self.current_frontier_world is not None:
                current_gap = math.hypot(
                    world_x - self.current_frontier_world[0],
                    world_y - self.current_frontier_world[1],
                )
                if current_gap < self.frontier_keep_radius:
                    score += self.frontier_current_bonus

            if score > best_score:
                best_score = score
                best = (
                    angle,
                    lookahead_distance,
                    score,
                    world_x,
                    world_y,
                    path_cells,
                    path_distance,
                )

        return best

    def best_frontier_cell_for_entry(
        self,
        entry_cell: Tuple[int, int],
        cluster: set[Tuple[int, int]],
    ) -> Optional[Tuple[int, int]]:
        best_cell: Optional[Tuple[int, int]] = None
        best_score = -float('inf')
        entry_x, entry_y = entry_cell

        for frontier_cell in self.frontier_neighbors(entry_x, entry_y):
            if frontier_cell not in cluster:
                continue
            cell_x, cell_y = frontier_cell
            score = (
                self.count_unknown_neighbors(cell_x, cell_y) * 2.0
                + len(self.free_neighbors(cell_x, cell_y))
            )
            if score > best_score:
                best_score = score
                best_cell = frontier_cell

        return best_cell

    def nearest_global_free_cell(
        self,
        start: Tuple[int, int],
        max_radius: int = 8,
    ) -> Optional[Tuple[int, int]]:
        if self.is_global_free(start[0], start[1]):
            return start

        queue = deque([(start[0], start[1], 0)])
        visited = {(start[0], start[1])}
        while queue:
            cell_x, cell_y, radius = queue.popleft()
            if radius > max_radius:
                continue
            if self.is_global_free(cell_x, cell_y):
                return cell_x, cell_y
            for neighbor_x, neighbor_y in self.frontier_neighbors(cell_x, cell_y):
                if (neighbor_x, neighbor_y) in visited:
                    continue
                if self.global_cell_index(neighbor_x, neighbor_y) is None:
                    continue
                visited.add((neighbor_x, neighbor_y))
                queue.append((neighbor_x, neighbor_y, radius + 1))

        return None

    def compute_reachable_global_cells(
        self,
        start: Tuple[int, int],
    ) -> Tuple[
        Dict[Tuple[int, int], float],
        Dict[Tuple[int, int], Optional[Tuple[int, int]]],
    ]:
        costs: Dict[Tuple[int, int], float] = {start: 0.0}
        parents: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        queue: List[Tuple[float, Tuple[int, int]]] = [(0.0, start)]
        resolution = max(self.global_map_resolution, 1e-3)

        while queue:
            cost, cell = heapq.heappop(queue)
            if cost > costs.get(cell, float('inf')):
                continue

            cell_x, cell_y = cell
            for neighbor_x, neighbor_y in self.frontier_neighbors(cell_x, cell_y):
                neighbor = (neighbor_x, neighbor_y)
                if not self.is_global_free(neighbor_x, neighbor_y):
                    continue
                if neighbor_x != cell_x and neighbor_y != cell_y:
                    if not self.is_global_free(neighbor_x, cell_y):
                        continue
                    if not self.is_global_free(cell_x, neighbor_y):
                        continue
                    step = resolution * math.sqrt(2.0)
                else:
                    step = resolution

                new_cost = cost + step
                if new_cost >= costs.get(neighbor, float('inf')):
                    continue
                costs[neighbor] = new_cost
                parents[neighbor] = cell
                heapq.heappush(queue, (new_cost, neighbor))

        return costs, parents

    def reconstruct_global_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        parents: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
    ) -> List[Tuple[int, int]]:
        if goal not in parents:
            return []

        path = [goal]
        cell = goal
        while cell != start:
            parent = parents.get(cell)
            if parent is None:
                return []
            path.append(parent)
            cell = parent
        path.reverse()
        return path

    def path_lookahead_world(self, path_cells: List[Tuple[int, int]]) -> Tuple[float, float]:
        if not path_cells:
            return 0.0, 0.0
        if len(path_cells) == 1:
            return self.global_cell_to_world(path_cells[0][0], path_cells[0][1])

        travelled = 0.0
        previous = path_cells[0]
        for cell in path_cells[1:]:
            px, py = self.global_cell_to_world(previous[0], previous[1])
            cx, cy = self.global_cell_to_world(cell[0], cell[1])
            travelled += math.hypot(cx - px, cy - py)
            if travelled >= self.exploration_path_lookahead_m:
                return cx, cy
            previous = cell

        last = path_cells[-1]
        return self.global_cell_to_world(last[0], last[1])

    def publish_exploration_path(self, path_cells: List[Tuple[int, int]]) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.global_frame

        for cell_x, cell_y in path_cells:
            world_x, world_y = self.global_cell_to_world(cell_x, cell_y)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.global_path_pub.publish(path)

    def collect_frontier_cells(self) -> set[Tuple[int, int]]:
        frontiers: set[Tuple[int, int]] = set()
        for y in range(1, self.global_map_height - 1):
            for x in range(1, self.global_map_width - 1):
                if not self.is_global_unknown(x, y):
                    continue
                if self.count_unknown_neighbors(x, y) < 2:
                    continue
                if self.has_free_neighbor(x, y):
                    frontiers.add((x, y))
        return frontiers

    def cluster_frontiers(
        self,
        frontier_cells: set[Tuple[int, int]],
    ) -> List[List[Tuple[int, int]]]:
        clusters: List[List[Tuple[int, int]]] = []
        unvisited = set(frontier_cells)

        while unvisited:
            seed = unvisited.pop()
            cluster = [seed]
            queue = deque([seed])

            while queue:
                cell_x, cell_y = queue.popleft()
                for neighbor in self.frontier_neighbors(cell_x, cell_y):
                    if neighbor not in unvisited:
                        continue
                    unvisited.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)

            clusters.append(cluster)

        return clusters

    def frontier_neighbors(self, cell_x: int, cell_y: int) -> List[Tuple[int, int]]:
        neighbors: List[Tuple[int, int]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((cell_x + dx, cell_y + dy))
        return neighbors

    def cluster_goal_world(self, cluster: List[Tuple[int, int]]) -> Tuple[float, float]:
        free_boundary: List[Tuple[int, int]] = []
        for cell_x, cell_y in cluster:
            free_boundary.extend(self.free_neighbors(cell_x, cell_y))

        cells = free_boundary if free_boundary else cluster
        mean_x = sum(cell[0] for cell in cells) / max(len(cells), 1)
        mean_y = sum(cell[1] for cell in cells) / max(len(cells), 1)
        return self.global_cell_to_world(int(round(mean_x)), int(round(mean_y)))

    def publish_global_occupancy_grid(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.global_frame
        grid.info.resolution = max(self.global_map_resolution, 1e-3)
        grid.info.width = self.global_map_width
        grid.info.height = self.global_map_height
        grid.info.origin.position.x = self.global_map_origin_x
        grid.info.origin.position.y = self.global_map_origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [self.log_odds_to_occupancy(value) for value in self.global_log_odds]
        self.global_map_pub.publish(grid)

    def log_odds_to_occupancy(self, value: float) -> int:
        if abs(value) <= self.global_unknown_log_odds:
            return -1
        if value < -self.global_unknown_log_odds:
            return 0
        probability = 1.0 - 1.0 / (1.0 + math.exp(value))
        return int(self.clamp(round(probability * 100.0), 1, 100))

    def add_global_log_odds(self, cell_x: int, cell_y: int, delta: float) -> None:
        index = self.global_cell_index(cell_x, cell_y)
        if index is None:
            return
        self.global_log_odds[index] = self.clamp(
            self.global_log_odds[index] + delta,
            self.global_min_log_odds,
            self.global_max_log_odds,
        )

    def world_to_global_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        resolution = max(self.global_map_resolution, 1e-3)
        cell_x = int((x - self.global_map_origin_x) / resolution)
        cell_y = int((y - self.global_map_origin_y) / resolution)
        if cell_x < 0 or cell_x >= self.global_map_width:
            return None
        if cell_y < 0 or cell_y >= self.global_map_height:
            return None
        return cell_x, cell_y

    def global_cell_to_world(self, cell_x: int, cell_y: int) -> Tuple[float, float]:
        resolution = max(self.global_map_resolution, 1e-3)
        return (
            self.global_map_origin_x + (cell_x + 0.5) * resolution,
            self.global_map_origin_y + (cell_y + 0.5) * resolution,
        )

    def global_cell_index(self, cell_x: int, cell_y: int) -> Optional[int]:
        if cell_x < 0 or cell_x >= self.global_map_width:
            return None
        if cell_y < 0 or cell_y >= self.global_map_height:
            return None
        return cell_y * self.global_map_width + cell_x

    def is_global_unknown(self, cell_x: int, cell_y: int) -> bool:
        index = self.global_cell_index(cell_x, cell_y)
        if index is None:
            return False
        return abs(self.global_log_odds[index]) <= self.global_unknown_log_odds

    def is_global_free(self, cell_x: int, cell_y: int) -> bool:
        index = self.global_cell_index(cell_x, cell_y)
        if index is None:
            return False
        return self.global_log_odds[index] < -self.global_unknown_log_odds

    def has_free_neighbor(self, cell_x: int, cell_y: int) -> bool:
        return bool(self.free_neighbors(cell_x, cell_y))

    def free_neighbors(self, cell_x: int, cell_y: int) -> List[Tuple[int, int]]:
        neighbors: List[Tuple[int, int]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if self.is_global_free(cell_x + dx, cell_y + dy):
                    neighbors.append((cell_x + dx, cell_y + dy))
        return neighbors

    def count_unknown_neighbors(self, cell_x: int, cell_y: int) -> int:
        count = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if self.is_global_unknown(cell_x + dx, cell_y + dy):
                    count += 1
        return count

    def count_global_known_cells(self) -> int:
        return sum(
            1
            for value in self.global_log_odds
            if abs(value) > self.global_unknown_log_odds
        )

    def bresenham_cells(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> List[Tuple[int, int]]:
        cells: List[Tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        x = x0
        y = y0

        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x += sx
            if twice_error <= dx:
                error += dx
                y += sy

        return cells

    def publish_metrics(
        self,
        source_counts: Dict[str, int],
        free: List[int],
        distances: List[float],
        detailed_distances: List[float],
        min_clearance: float,
        occupied_cells: int,
        global_known_cells: int,
        frontier_distance: float,
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
            'global_known_cells': int(global_known_cells),
            'frontier_distance_m': float(frontier_distance),
            'exploration_path_length_m': float(self.last_exploration_path_length),
            'exploration_path_cells': int(self.last_exploration_path_cells),
            'smoke_density': self.latest_smoke_density,
            'frame': self.base_frame,
            'global_frame': self.global_frame,
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

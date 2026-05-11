#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, String
import tf2_ros

try:
    from sensor_msgs_py import point_cloud2
except ImportError:  # pragma: no cover - depends on ROS installation.
    point_cloud2 = None


@dataclass(frozen=True)
class ThermalDetection:
    u: float
    v: float
    width: float
    height: float
    area: float


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    z: float
    radar_score: float
    thermal_score: float
    depth_score: float
    heartbeat_score: float
    confidence: float


class FusionNode(Node):
    """Fuse radar geometry with thermal and depth support for human candidates."""

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('radar_frame', 'radar_link')
        self.declare_parameter('radar_cluster_topic', '/radar/human_clusters')
        self.declare_parameter('thermal_topic', '/thermal/human_positions')
        self.declare_parameter('heartbeat_topic', '/human_heartbeat')
        self.declare_parameter('depth_points_topic', '/camera/depth/color/points')
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('metrics_topic', '/human_detection/metrics')
        self.declare_parameter('smoke_density_topic', '/smoke/density')
        self.declare_parameter('tf_timeout_sec', 0.04)

        self.declare_parameter('thermal_hfov_deg', 60.0)
        self.declare_parameter('thermal_vfov_deg', 45.0)
        self.declare_parameter('thermal_match_angle_deg', 16.0)
        self.declare_parameter('thermal_match_vertical_tol', 0.55)
        self.declare_parameter('thermal_allow_mirrored_match', True)

        self.declare_parameter('depth_match_angle_deg', 10.0)
        self.declare_parameter('depth_range_tolerance', 0.85)
        self.declare_parameter('depth_min_range', 0.25)
        self.declare_parameter('depth_max_range', 8.0)
        self.declare_parameter('depth_max_points', 1800)
        self.declare_parameter('heartbeat_match_distance', 0.80)
        self.declare_parameter('heartbeat_min_score', 0.20)
        self.declare_parameter('require_heartbeat', False)
        self.declare_parameter('use_heartbeat_fallback', False)
        self.declare_parameter('heartbeat_fallback_confidence', 0.70)

        self.declare_parameter('radar_weight', 0.54)
        self.declare_parameter('thermal_weight', 0.30)
        self.declare_parameter('depth_weight', 0.16)
        self.declare_parameter('heartbeat_weight', 0.45)
        self.declare_parameter('min_publish_confidence', 0.40)
        self.declare_parameter('max_candidate_range', 8.0)
        self.declare_parameter('radar_human_max_spread', 0.65)
        self.declare_parameter('radar_human_max_points', 9)
        self.declare_parameter('publish_rate', 8.0)
        self.declare_parameter('sensor_timeout', 1.0)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.radar_frame = str(self.get_parameter('radar_frame').value)
        self.radar_cluster_topic = str(self.get_parameter('radar_cluster_topic').value)
        self.thermal_topic = str(self.get_parameter('thermal_topic').value)
        self.heartbeat_topic = str(self.get_parameter('heartbeat_topic').value)
        self.depth_points_topic = str(self.get_parameter('depth_points_topic').value)
        self.humans_topic = str(self.get_parameter('humans_topic').value)
        self.metrics_topic = str(self.get_parameter('metrics_topic').value)
        self.smoke_density_topic = str(self.get_parameter('smoke_density_topic').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.thermal_hfov = math.radians(float(self.get_parameter('thermal_hfov_deg').value))
        self.thermal_vfov = math.radians(float(self.get_parameter('thermal_vfov_deg').value))
        self.thermal_match_angle = math.radians(
            float(self.get_parameter('thermal_match_angle_deg').value)
        )
        self.thermal_match_vertical_tol = float(
            self.get_parameter('thermal_match_vertical_tol').value
        )
        self.thermal_allow_mirrored_match = bool(
            self.get_parameter('thermal_allow_mirrored_match').value
        )

        self.depth_match_angle = math.radians(float(self.get_parameter('depth_match_angle_deg').value))
        self.depth_range_tolerance = float(self.get_parameter('depth_range_tolerance').value)
        self.depth_min_range = float(self.get_parameter('depth_min_range').value)
        self.depth_max_range = float(self.get_parameter('depth_max_range').value)
        self.depth_max_points = int(self.get_parameter('depth_max_points').value)
        self.heartbeat_match_distance = float(
            self.get_parameter('heartbeat_match_distance').value
        )
        self.heartbeat_min_score = float(self.get_parameter('heartbeat_min_score').value)
        self.require_heartbeat = bool(self.get_parameter('require_heartbeat').value)
        self.use_heartbeat_fallback = bool(
            self.get_parameter('use_heartbeat_fallback').value
        )
        self.heartbeat_fallback_confidence = float(
            self.get_parameter('heartbeat_fallback_confidence').value
        )

        self.radar_weight = float(self.get_parameter('radar_weight').value)
        self.thermal_weight = float(self.get_parameter('thermal_weight').value)
        self.depth_weight = float(self.get_parameter('depth_weight').value)
        self.heartbeat_weight = float(self.get_parameter('heartbeat_weight').value)
        self.min_publish_confidence = float(self.get_parameter('min_publish_confidence').value)
        self.max_candidate_range = float(self.get_parameter('max_candidate_range').value)
        self.radar_human_max_spread = float(self.get_parameter('radar_human_max_spread').value)
        self.radar_human_max_points = int(self.get_parameter('radar_human_max_points').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_radar: List[Tuple[float, float, float, float]] = []
        self.latest_thermal: List[ThermalDetection] = []
        self.latest_heartbeat: List[Tuple[float, float, float, float, float]] = []
        self.latest_depth: List[Tuple[float, float]] = []
        self.last_radar_time = None
        self.last_thermal_time = None
        self.last_heartbeat_time = None
        self.last_depth_time = None
        self.smoke_density = 0.0

        self.create_subscription(
            Float32MultiArray,
            self.thermal_topic,
            self.thermal_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.radar_cluster_topic,
            self.radar_cb,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.heartbeat_topic,
            self.heartbeat_cb,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.depth_points_topic,
            self.depth_cb,
            10,
        )
        self.create_subscription(Float32, self.smoke_density_topic, self.smoke_cb, 10)

        self.pub = self.create_publisher(PoseArray, self.humans_topic, 10)
        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)
        period = 1.0 / max(self.publish_rate, 0.5)
        self.create_timer(period, self.publish_fused)

        self.get_logger().info(
            'Human fusion ready (radar=range, thermal=classification, depth=geometry support)'
        )

    def thermal_cb(self, msg: Float32MultiArray) -> None:
        self.latest_thermal = self.parse_thermal(msg.data)
        self.last_thermal_time = self.get_clock().now()

    def radar_cb(self, msg: Float32MultiArray) -> None:
        values = list(msg.data)
        radar = []
        stride = 5 if len(values) >= 5 and len(values) % 5 == 0 else 3
        for i in range(0, len(values) - stride + 1, stride):
            point = self.transform_point(
                values[i],
                values[i + 1],
                values[i + 2],
                self.radar_frame,
            )
            if point is None:
                continue
            x, y, z = point
            if 0.0 < math.hypot(x, y) <= self.max_candidate_range:
                if stride == 5:
                    quality = self.radar_shape_score(values[i + 3], values[i + 4])
                else:
                    quality = 0.7
                radar.append((x, y, z, quality))
        self.latest_radar = radar
        self.last_radar_time = self.get_clock().now()

    def heartbeat_cb(self, msg: Float32MultiArray) -> None:
        self.latest_heartbeat = self.parse_heartbeat(msg.data)
        self.last_heartbeat_time = self.get_clock().now()

    def depth_cb(self, msg: PointCloud2) -> None:
        self.latest_depth = self.extract_depth_support(msg)
        self.last_depth_time = self.get_clock().now()

    def smoke_cb(self, msg: Float32) -> None:
        self.smoke_density = self.clamp(float(msg.data), 0.0, 1.0)

    def publish_fused(self) -> None:
        now = self.get_clock().now()
        radar_fresh = self.is_fresh(self.last_radar_time, now)
        thermal_fresh = self.is_fresh(self.last_thermal_time, now)
        heartbeat_fresh = self.is_fresh(self.last_heartbeat_time, now)
        depth_fresh = self.is_fresh(self.last_depth_time, now)

        candidates = []
        if radar_fresh:
            for x, y, z, radar_quality in self.latest_radar:
                candidates.append(
                    self.score_candidate(
                        x,
                        y,
                        z,
                        radar_quality,
                        thermal_fresh,
                        heartbeat_fresh,
                        depth_fresh,
                    )
                )

        if heartbeat_fresh and self.use_heartbeat_fallback:
            candidates.extend(self.heartbeat_fallback_candidates())

        candidates = [
            c for c in candidates
            if c is not None and c.confidence >= self.min_publish_confidence
        ]
        candidates.sort(key=lambda c: c.confidence, reverse=True)

        humans = PoseArray()
        humans.header.frame_id = self.base_frame
        humans.header.stamp = now.to_msg()

        for candidate in candidates:
            pose = Pose()
            pose.position.x = candidate.x
            pose.position.y = candidate.y
            pose.position.z = candidate.z
            pose.orientation.x = candidate.thermal_score
            pose.orientation.y = candidate.depth_score
            pose.orientation.z = candidate.radar_score
            pose.orientation.w = candidate.confidence
            humans.poses.append(pose)

        self.pub.publish(humans)
        self.publish_metrics(
            candidates,
            radar_fresh,
            thermal_fresh,
            heartbeat_fresh,
            depth_fresh,
        )

    def score_candidate(
        self,
        x: float,
        y: float,
        z: float,
        radar_quality: float,
        thermal_fresh: bool,
        heartbeat_fresh: bool,
        depth_fresh: bool,
    ) -> Optional[Candidate]:
        distance = math.hypot(x, y)
        if distance <= 0.0:
            return None

        range_score = self.clamp(1.0 - distance / max(self.max_candidate_range, 0.1), 0.25, 1.0)
        radar_score = self.clamp(range_score * radar_quality, 0.0, 1.0)
        thermal_score = self.match_thermal(x, y, z) if thermal_fresh else 0.0
        heartbeat_score = self.match_heartbeat(x, y, z) if heartbeat_fresh else 0.0
        if self.require_heartbeat and heartbeat_score < self.heartbeat_min_score:
            return None
        depth_score = self.match_depth(x, y) if depth_fresh else 0.0

        # In dense smoke the radar should carry more of the estimate, while
        # thermal/depth become supporting evidence rather than hard gates.
        smoke = self.smoke_density
        radar_weight = self.radar_weight + 0.14 * smoke
        thermal_weight = max(0.10, self.thermal_weight - 0.12 * smoke)
        depth_weight = max(0.06, self.depth_weight - 0.08 * smoke)
        heartbeat_weight = self.heartbeat_weight if heartbeat_fresh else 0.0
        total = radar_weight + thermal_weight + depth_weight + heartbeat_weight

        confidence = (
            radar_weight * radar_score +
            thermal_weight * thermal_score +
            heartbeat_weight * heartbeat_score +
            depth_weight * depth_score
        ) / max(total, 1e-6)

        if thermal_score <= 0.0 and depth_score <= 0.0:
            confidence *= 0.82
        elif thermal_score > 0.0 and depth_score > 0.0:
            confidence = min(1.0, confidence + 0.08)
        if heartbeat_score >= 0.65:
            confidence = min(1.0, confidence + 0.10)

        return Candidate(
            x=x,
            y=y,
            z=z,
            radar_score=radar_score,
            thermal_score=thermal_score,
            depth_score=depth_score,
            heartbeat_score=heartbeat_score,
            confidence=self.clamp(confidence, 0.0, 1.0),
        )

    def heartbeat_fallback_candidates(self) -> List[Candidate]:
        candidates = []
        for x, y, z, strength, _phase in self.latest_heartbeat:
            confidence = self.clamp(
                self.heartbeat_fallback_confidence,
                0.0,
                1.0,
            )
            candidates.append(
                Candidate(
                    x=x,
                    y=y,
                    z=z,
                    radar_score=0.0,
                    thermal_score=0.0,
                    depth_score=0.0,
                    heartbeat_score=strength,
                    confidence=confidence,
                )
            )
        return candidates

    def radar_shape_score(self, count: float, spread: float) -> float:
        count = max(1.0, float(count))
        spread = max(0.0, float(spread))
        count_score = 1.0
        if count > self.radar_human_max_points:
            count_score = self.clamp(
                1.0 - (count - self.radar_human_max_points) / self.radar_human_max_points,
                0.20,
                1.0,
            )
        spread_score = self.clamp(
            1.0 - spread / max(self.radar_human_max_spread, 1e-6),
            0.15,
            1.0,
        )
        return self.clamp(0.35 + 0.65 * count_score * spread_score, 0.0, 1.0)

    def parse_thermal(self, values: Iterable[float]) -> List[ThermalDetection]:
        values = list(values)
        detections = []
        if len(values) >= 5 and len(values) % 5 == 0:
            stride = 5
        else:
            stride = 2

        for i in range(0, len(values) - stride + 1, stride):
            u = self.clamp(float(values[i]), 0.0, 1.0)
            v = self.clamp(float(values[i + 1]), 0.0, 1.0)
            if stride == 5:
                width = self.clamp(float(values[i + 2]), 0.0, 1.0)
                height = self.clamp(float(values[i + 3]), 0.0, 1.0)
                area = self.clamp(float(values[i + 4]), 0.0, 1.0)
            else:
                width = 0.12
                height = 0.24
                area = width * height
            detections.append(ThermalDetection(u=u, v=v, width=width, height=height, area=area))
        return detections

    def parse_heartbeat(
        self,
        values: Iterable[float],
    ) -> List[Tuple[float, float, float, float, float]]:
        values = list(values)
        beats = []
        stride = 5
        for i in range(0, len(values) - stride + 1, stride):
            x = float(values[i])
            y = float(values[i + 1])
            z = float(values[i + 2])
            strength = self.clamp(float(values[i + 3]), 0.0, 1.0)
            phase = float(values[i + 4])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                beats.append((x, y, z, strength, phase))
        return beats

    def match_thermal(self, x: float, y: float, z: float) -> float:
        if not self.latest_thermal:
            return 0.0

        bearing = math.atan2(y, x)
        elevation = math.atan2(z, max(math.hypot(x, y), 1e-6))
        best = 0.0
        for det in self.latest_thermal:
            thermal_angle = (0.5 - det.u) * self.thermal_hfov
            angle_error = abs(self.normalize_angle(bearing - thermal_angle))
            if self.thermal_allow_mirrored_match:
                angle_error = min(angle_error, abs(self.normalize_angle(bearing + thermal_angle)))

            vertical = abs((0.5 - det.v) * self.thermal_vfov - elevation)
            angle_score = 1.0 - angle_error / max(self.thermal_match_angle, 1e-6)
            vertical_score = 1.0 - vertical / max(self.thermal_match_vertical_tol, 1e-6)
            size_score = self.clamp(math.sqrt(max(det.area, 0.0)) / 0.18, 0.25, 1.0)
            score = self.clamp(angle_score, 0.0, 1.0) * self.clamp(vertical_score, 0.0, 1.0)
            best = max(best, score * size_score)
        return self.clamp(best, 0.0, 1.0)

    def match_heartbeat(self, x: float, y: float, z: float) -> float:
        if not self.latest_heartbeat:
            return 0.0

        best = 0.0
        for beat_x, beat_y, beat_z, strength, _phase in self.latest_heartbeat:
            error = math.sqrt(
                (x - beat_x) * (x - beat_x) +
                (y - beat_y) * (y - beat_y) +
                0.05 * (z - beat_z) * (z - beat_z)
            )
            if error > self.heartbeat_match_distance:
                continue
            spatial_score = 1.0 - error / max(self.heartbeat_match_distance, 1e-6)
            best = max(best, self.clamp(spatial_score, 0.0, 1.0) * strength)
        return self.clamp(best, 0.0, 1.0)

    def match_depth(self, x: float, y: float) -> float:
        if not self.latest_depth:
            return 0.0

        bearing = math.atan2(y, x)
        distance = math.hypot(x, y)
        best = 0.0
        for depth_angle, depth_distance in self.latest_depth:
            angle_error = abs(self.normalize_angle(bearing - depth_angle))
            if angle_error > self.depth_match_angle:
                continue
            range_error = abs(distance - depth_distance)
            angle_score = 1.0 - angle_error / max(self.depth_match_angle, 1e-6)
            range_score = 1.0 - range_error / max(self.depth_range_tolerance, 1e-6)
            best = max(best, self.clamp(angle_score, 0.0, 1.0) * self.clamp(range_score, 0.0, 1.0))
        return self.clamp(best, 0.0, 1.0)

    def extract_depth_support(self, cloud: PointCloud2) -> List[Tuple[float, float]]:
        points = []
        for x, y, z in self.iter_cloud_points(cloud):
            point = self.transform_point(x, y, z, cloud.header.frame_id)
            if point is None:
                continue
            base_x, base_y, _ = point
            distance = math.hypot(base_x, base_y)
            if self.depth_min_range <= distance <= self.depth_max_range:
                points.append((math.atan2(base_y, base_x), distance))
                if len(points) >= self.depth_max_points:
                    break
        return points

    def iter_cloud_points(self, cloud: PointCloud2):
        if point_cloud2 is None:
            return []
        try:
            return point_cloud2.read_points(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
        except TypeError:
            return point_cloud2.read_points(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )

    def transform_point(
        self,
        x: float,
        y: float,
        z: float,
        source_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        if not source_frame or source_frame == self.base_frame:
            return float(x), float(y), float(z)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception:
            return None

        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        rx, ry, rz = self.rotate_vector(q, float(x), float(y), float(z))
        return rx + tx, ry + ty, rz + tz

    @staticmethod
    def rotate_vector(q, x: float, y: float, z: float) -> Tuple[float, float, float]:
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def is_fresh(self, stamp, now) -> bool:
        if stamp is None:
            return False
        return (now - stamp).nanoseconds / 1e9 <= self.sensor_timeout

    def publish_metrics(
        self,
        candidates: List[Candidate],
        radar_fresh: bool,
        thermal_fresh: bool,
        heartbeat_fresh: bool,
        depth_fresh: bool,
    ) -> None:
        metrics = {
            'radar_fresh': radar_fresh,
            'thermal_fresh': thermal_fresh,
            'heartbeat_fresh': heartbeat_fresh,
            'depth_fresh': depth_fresh,
            'radar_candidates': len(self.latest_radar),
            'thermal_detections': len(self.latest_thermal),
            'heartbeat_signals': len(self.latest_heartbeat),
            'depth_support_points': len(self.latest_depth),
            'published_humans': len(candidates),
            'best_confidence': candidates[0].confidence if candidates else 0.0,
            'best_heartbeat_score': candidates[0].heartbeat_score if candidates else 0.0,
            'smoke_density': self.smoke_density,
        }
        msg = String()
        msg.data = json.dumps(metrics, separators=(',', ':'))
        self.metrics_pub.publish(msg)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

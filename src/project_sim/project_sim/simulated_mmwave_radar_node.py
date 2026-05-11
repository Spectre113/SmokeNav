from __future__ import annotations

import math
import random
from typing import List, Sequence

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


class SimulatedMmwaveRadarNode(Node):
    """Synthetic through-wall mmWave radar publisher with velocity field."""

    def __init__(self) -> None:
        super().__init__('simulated_mmwave_radar_node')

        self.declare_parameter('robot_model_name', 'smokenav_robot')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('output_topic', '/radar/points')
        self.declare_parameter('radar_frame', 'radar_link')
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('horizontal_fov_deg', 120.0)
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('radar_offset_x_m', 0.1)
        self.declare_parameter('radar_offset_y_m', 0.0)
        self.declare_parameter('radar_offset_z_m', 0.105)
        self.declare_parameter('tracked_model_prefixes', ['human_'])
        self.declare_parameter('excluded_model_names', ['ground_plane', 'sun'])
        self.declare_parameter('include_stationary_tracked_models', True)
        self.declare_parameter('dynamic_speed_threshold_mps', 0.05)
        self.declare_parameter('points_per_target', 12)
        self.declare_parameter('target_center_z_offset_m', 0.85)
        self.declare_parameter('target_radius_m', 0.18)
        self.declare_parameter('target_height_m', 1.7)
        self.declare_parameter('position_noise_std_m', 0.015)
        self.declare_parameter('velocity_noise_std_mps', 0.02)
        self.declare_parameter('random_seed', 4242)

        self.robot_model_name = str(self.get_parameter('robot_model_name').value)
        self.model_states_topic = str(self.get_parameter('model_states_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.radar_frame = str(self.get_parameter('radar_frame').value)
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.horizontal_fov_rad = math.radians(
            float(self.get_parameter('horizontal_fov_deg').value)
        )
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.radar_offset_x_m = float(self.get_parameter('radar_offset_x_m').value)
        self.radar_offset_y_m = float(self.get_parameter('radar_offset_y_m').value)
        self.radar_offset_z_m = float(self.get_parameter('radar_offset_z_m').value)
        self.tracked_model_prefixes = tuple(
            str(prefix) for prefix in self.get_parameter('tracked_model_prefixes').value
        )
        self.excluded_model_names = {
            str(name).lower() for name in self.get_parameter('excluded_model_names').value
        }
        self.include_stationary_tracked_models = bool(
            self.get_parameter('include_stationary_tracked_models').value
        )
        self.dynamic_speed_threshold_mps = float(
            self.get_parameter('dynamic_speed_threshold_mps').value
        )
        self.points_per_target = max(1, int(self.get_parameter('points_per_target').value))
        self.target_center_z_offset_m = float(
            self.get_parameter('target_center_z_offset_m').value
        )
        self.target_radius_m = float(self.get_parameter('target_radius_m').value)
        self.target_height_m = float(self.get_parameter('target_height_m').value)
        self.position_noise_std_m = float(
            self.get_parameter('position_noise_std_m').value
        )
        self.velocity_noise_std_mps = float(
            self.get_parameter('velocity_noise_std_mps').value
        )
        self._rng = random.Random(int(self.get_parameter('random_seed').value))

        self._latest_model_states: ModelStates | None = None
        self._warned_missing_robot = False

        self._cloud_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self._model_states_callback,
            10,
        )
        self._publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self._timer = self.create_timer(1.0 / self.update_rate_hz, self._publish_cloud)

        self.get_logger().info(
            'Simulated mmWave radar started '
            f'(topic={self.output_topic}, fov={math.degrees(self.horizontal_fov_rad):.0f} deg, '
            f'range={self.min_range_m:.1f}-{self.max_range_m:.1f} m)'
        )

    def _model_states_callback(self, msg: ModelStates) -> None:
        self._latest_model_states = msg

    def _publish_cloud(self) -> None:
        if self._latest_model_states is None:
            return

        msg = self._latest_model_states
        try:
            robot_index = msg.name.index(self.robot_model_name)
        except ValueError:
            if not self._warned_missing_robot:
                self.get_logger().warn(
                    f'Robot model "{self.robot_model_name}" not found in {self.model_states_topic}'
                )
                self._warned_missing_robot = True
            self._publish_points([])
            return

        self._warned_missing_robot = False

        robot_pose = msg.pose[robot_index]
        robot_twist = msg.twist[robot_index]
        robot_yaw = _yaw_from_quaternion(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )
        radar_world_x, radar_world_y = _rotate_xy(
            self.radar_offset_x_m,
            self.radar_offset_y_m,
            robot_yaw,
        )
        radar_world_x += robot_pose.position.x
        radar_world_y += robot_pose.position.y
        radar_world_z = robot_pose.position.z + self.radar_offset_z_m

        points: List[Sequence[float]] = []
        for index, name in enumerate(msg.name):
            if index == robot_index:
                continue
            if name.lower() in self.excluded_model_names:
                continue

            twist = msg.twist[index]
            linear_speed = math.sqrt(
                twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2
            )
            tracked_by_name = name.startswith(self.tracked_model_prefixes)

            if tracked_by_name:
                if not self.include_stationary_tracked_models and (
                    linear_speed < self.dynamic_speed_threshold_mps
                ):
                    continue
            elif linear_speed < self.dynamic_speed_threshold_mps:
                continue

            pose = msg.pose[index]
            target_world_x = pose.position.x
            target_world_y = pose.position.y
            target_world_z = pose.position.z + self.target_center_z_offset_m

            dx = target_world_x - radar_world_x
            dy = target_world_y - radar_world_y
            dz = target_world_z - radar_world_z

            rel_x, rel_y = _rotate_xy(dx, dy, -robot_yaw)
            ground_range = math.hypot(rel_x, rel_y)
            if ground_range < self.min_range_m or ground_range > self.max_range_m:
                continue

            angle = math.atan2(rel_y, rel_x)
            if abs(angle) > self.horizontal_fov_rad * 0.5:
                continue

            range_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
            line_of_sight_x = dx / max(range_3d, 1e-6)
            line_of_sight_y = dy / max(range_3d, 1e-6)
            line_of_sight_z = dz / max(range_3d, 1e-6)
            radial_velocity = (
                (twist.linear.x - robot_twist.linear.x) * line_of_sight_x
                + (twist.linear.y - robot_twist.linear.y) * line_of_sight_y
                + (twist.linear.z - robot_twist.linear.z) * line_of_sight_z
            )

            points.extend(
                self._make_target_cluster(
                    center_x=rel_x,
                    center_y=rel_y,
                    center_z=dz,
                    radial_velocity=radial_velocity,
                    base_speed=linear_speed,
                )
            )

        self._publish_points(points)

    def _make_target_cluster(
        self,
        center_x: float,
        center_y: float,
        center_z: float,
        radial_velocity: float,
        base_speed: float,
    ) -> List[Sequence[float]]:
        points: List[Sequence[float]] = []
        vertical_half_extent = max(0.05, self.target_height_m * 0.25)

        for i in range(self.points_per_target):
            phase = (2.0 * math.pi * i) / self.points_per_target
            lateral = self.target_radius_m * math.sin(phase)
            depth = 0.04 * math.cos(phase)
            vertical = vertical_half_extent * math.sin(phase * 0.5)

            x = center_x + depth + self._rng.gauss(0.0, self.position_noise_std_m)
            y = center_y + lateral + self._rng.gauss(0.0, self.position_noise_std_m)
            z = center_z + vertical + self._rng.gauss(0.0, self.position_noise_std_m * 0.5)
            velocity = radial_velocity + self._rng.gauss(
                0.0,
                self.velocity_noise_std_mps + 0.02 * base_speed,
            )
            points.append((float(x), float(y), float(z), float(velocity)))

        return points

    def _publish_points(self, points: List[Sequence[float]]) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.radar_frame
        cloud = point_cloud2.create_cloud(header, self._cloud_fields, points)
        self._publisher.publish(cloud)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedMmwaveRadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

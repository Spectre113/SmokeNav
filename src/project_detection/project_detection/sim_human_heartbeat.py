import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
import tf2_ros


@dataclass(frozen=True)
class Heartbeat:
    x: float
    y: float
    z: float
    strength: float
    phase: float


class SimHumanHeartbeat(Node):
    """Simulation-only biosignal emitted by Gazebo human models.

    The red cylinder still participates in normal range sensing, but this topic
    gives the detector a "living target" signature so boxes do not become humans.
    """

    def __init__(self) -> None:
        super().__init__("sim_human_heartbeat")

        self.declare_parameter("model_name_prefix", "human_")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("heartbeat_topic", "/human_heartbeat")
        self.declare_parameter("heartbeat_height", 0.85)
        self.declare_parameter("frequency_hz", 1.25)
        self.declare_parameter("max_signal_range", 10.0)
        self.declare_parameter("tf_timeout_sec", 0.04)

        self.model_name_prefix = str(self.get_parameter("model_name_prefix").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)
        self.heartbeat_height = float(self.get_parameter("heartbeat_height").value)
        self.frequency_hz = float(self.get_parameter("frequency_hz").value)
        self.max_signal_range = float(self.get_parameter("max_signal_range").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Float32MultiArray, self.heartbeat_topic, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.model_cb, 10)

        self.get_logger().info(
            f"Simulation heartbeat enabled for models '{self.model_name_prefix}*'"
        )

    def model_cb(self, msg: ModelStates) -> None:
        beats = []
        for index, name in enumerate(msg.name):
            if not name.startswith(self.model_name_prefix):
                continue

            pose = msg.pose[index]
            transformed = self.transform_point(
                pose.position.x,
                pose.position.y,
                pose.position.z + self.heartbeat_height,
                self.world_frame,
            )
            if transformed is None:
                continue

            x, y, z = transformed
            distance = math.hypot(x, y)
            if distance > self.max_signal_range:
                continue

            phase = self.current_phase()
            range_score = self.clamp(
                1.0 - distance / max(self.max_signal_range, 1e-6),
                0.35,
                1.0,
            )
            pulse = 0.65 + 0.35 * math.sin(phase)
            strength = self.clamp(range_score * pulse, 0.0, 1.0)
            beats.append(Heartbeat(x=x, y=y, z=z, strength=strength, phase=phase))

        out = Float32MultiArray()
        data = []
        for beat in beats:
            data.extend([beat.x, beat.y, beat.z, beat.strength, beat.phase])
        out.data = data
        self.pub.publish(out)

    def current_phase(self) -> float:
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        return (2.0 * math.pi * self.frequency_hz * seconds) % (2.0 * math.pi)

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

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main() -> None:
    rclpy.init()
    node = SimHumanHeartbeat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

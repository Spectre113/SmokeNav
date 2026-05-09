from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class Detection:
    name: str
    pose: PoseStamped


class GazeboHumanDetector(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_human_detector")

        self.declare_parameter("model_name_prefix", "human_")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("publish_markers", True)

        self._prefix = (
            self.get_parameter("model_name_prefix").get_parameter_value().string_value
        )
        self._world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        self._publish_markers = (
            self.get_parameter("publish_markers").get_parameter_value().bool_value
        )

        self._sub = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_model_states, 10
        )
        self._pose_pub = self.create_publisher(PoseStamped, "/human_pose", 10)
        self._marker_pub = self.create_publisher(Marker, "/human_pose_marker", 10)

    def _select_first_human(self, msg: ModelStates) -> Optional[Detection]:
        for i, name in enumerate(msg.name):
            if name.startswith(self._prefix):
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self._world_frame
                pose_msg.pose = msg.pose[i]
                return Detection(name=name, pose=pose_msg)
        return None

    def _on_model_states(self, msg: ModelStates) -> None:
        det = self._select_first_human(msg)
        if det is None:
            return

        self._pose_pub.publish(det.pose)

        if self._publish_markers:
            m = Marker()
            m.header = det.pose.header
            m.ns = "human_pose"
            m.id = 0
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose = det.pose.pose
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.7
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.8
            m.lifetime.sec = 0
            self._marker_pub.publish(m)


def main() -> None:
    rclpy.init()
    node = GazeboHumanDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


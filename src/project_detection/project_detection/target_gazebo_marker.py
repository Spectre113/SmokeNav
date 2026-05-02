from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32MultiArray


class TargetGazeboMarker(Node):
    def __init__(self) -> None:
        super().__init__("target_gazebo_marker")

        self.declare_parameter("target_pose_topic", "/human_localization/pose")
        self.declare_parameter("target_info_topic", "/target_info")
        self.declare_parameter("marker_name", "goal_target_marker")
        self.declare_parameter("marker_height", 0.12)
        self.declare_parameter("marker_radius", 0.16)
        self.declare_parameter("consume_on_reach", True)
        self.declare_parameter("consume_distance", 0.75)
        self.declare_parameter("target_entity_name", "human_0")

        self._target_topic = str(self.get_parameter("target_pose_topic").value)
        self._target_info_topic = str(self.get_parameter("target_info_topic").value)
        self._marker_name = str(self.get_parameter("marker_name").value)
        self._marker_height = float(self.get_parameter("marker_height").value)
        self._marker_radius = float(self.get_parameter("marker_radius").value)
        self._consume_on_reach = bool(self.get_parameter("consume_on_reach").value)
        self._consume_distance = float(self.get_parameter("consume_distance").value)
        self._target_entity_name = str(self.get_parameter("target_entity_name").value)

        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")
        self._set_cli_primary = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self._set_cli_fallback = self.create_client(SetEntityState, "/set_entity_state")

        self._spawned = False
        self._spawn_in_flight = False
        self._consume_in_flight = False
        self._consumed_once = False
        self._warned_no_set_service = False
        self._last_pose: Optional[Pose] = None

        self.create_subscription(PoseStamped, self._target_topic, self._pose_cb, 10)
        self.create_subscription(Float32MultiArray, self._target_info_topic, self._target_info_cb, 10)
        self.get_logger().info(
            f"Target marker enabled. Listening {self._target_topic}, model={self._marker_name}"
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._consumed_once:
            return

        self._last_pose = msg.pose

        if not self._spawned:
            self._ensure_spawned(msg.pose)
            return

        self._update_marker(msg.pose)

    def _target_info_cb(self, msg: Float32MultiArray) -> None:
        if self._consumed_once or self._consume_in_flight or not self._consume_on_reach:
            return

        if len(msg.data) < 4:
            return

        detected = float(msg.data[0]) > 0.5
        distance = float(msg.data[2])
        confidence = float(msg.data[3])

        # If close enough, consume target even when adapter already hid it (detected=0).
        has_valid_target_signal = detected or confidence > 0.2
        if has_valid_target_signal and 0.0 < distance <= self._consume_distance:
            self._consume_target_once(distance)

    def _ensure_spawned(self, pose: Pose) -> None:
        if self._spawn_in_flight:
            return

        if not self._spawn_cli.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Waiting for /spawn_entity service...", throttle_duration_sec=2.0)
            return

        req = SpawnEntity.Request()
        req.name = self._marker_name
        req.xml = self._build_marker_sdf()
        req.initial_pose = self._pose_with_marker_height(pose)
        req.reference_frame = "world"

        self._spawn_in_flight = True
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future) -> None:
        self._spawn_in_flight = False

        if self._consumed_once:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to spawn marker: {exc}")
            return

        status = response.status_message.lower()
        if response.success or "already exists" in status:
            self._spawned = True
            self.get_logger().info("Goal marker spawned in Gazebo.")
            if self._last_pose is not None:
                self._update_marker(self._last_pose)
            return

        self.get_logger().warn(f"Failed to spawn marker: {response.status_message}")

    def _pick_set_service(self):
        if self._set_cli_primary.wait_for_service(timeout_sec=0.0):
            return self._set_cli_primary
        if self._set_cli_fallback.wait_for_service(timeout_sec=0.0):
            return self._set_cli_fallback
        return None

    def _update_marker(self, pose: Pose) -> None:
        set_cli = self._pick_set_service()
        if set_cli is None:
            if not self._warned_no_set_service:
                self.get_logger().warn(
                    "No SetEntityState service found (/gazebo/set_entity_state or /set_entity_state). "
                    "Marker will stay at the first detected target."
                )
                self._warned_no_set_service = True
            return

        req = SetEntityState.Request()
        state = EntityState()
        state.name = self._marker_name
        state.pose = self._pose_with_marker_height(pose)
        state.reference_frame = "world"
        req.state = state
        set_cli.call_async(req)

    def _consume_target_once(self, distance: float) -> None:
        if not self._delete_cli.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Waiting for /delete_entity service...", throttle_duration_sec=2.0)
            return

        self._consume_in_flight = True
        self._consumed_once = True
        self._spawned = False
        self.get_logger().info(
            f"Target reached (dist={distance:.2f} <= {self._consume_distance:.2f}). "
            "Removing target from Gazebo."
        )

        self._delete_entity(self._target_entity_name)
        self._delete_entity(self._marker_name)
        self._consume_in_flight = False

    def _delete_entity(self, name: str) -> None:
        req = DeleteEntity.Request()
        req.name = name
        future = self._delete_cli.call_async(req)
        future.add_done_callback(lambda f, n=name: self._on_delete_done(f, n))

    def _on_delete_done(self, future, name: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"DeleteEntity failed for {name}: {exc}")
            return

        if response.success:
            self.get_logger().info(f"Deleted Gazebo entity: {name}")
        else:
            self.get_logger().warn(f"DeleteEntity for {name}: {response.status_message}")

    def _pose_with_marker_height(self, pose: Pose) -> Pose:
        out = Pose()
        out.position.x = float(pose.position.x)
        out.position.y = float(pose.position.y)
        out.position.z = self._marker_height
        out.orientation.w = 1.0
        return out

    def _build_marker_sdf(self) -> str:
        radius = max(0.03, self._marker_radius)
        return f"""
<sdf version='1.7'>
  <model name='{self._marker_name}'>
    <static>false</static>
    <link name='marker_link'>
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <self_collide>false</self_collide>
      <visual name='marker_visual'>
        <geometry>
          <sphere>
            <radius>{radius:.3f}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.1 1.0 0.1 0.95</ambient>
          <diffuse>0.1 1.0 0.1 0.95</diffuse>
          <emissive>0.05 0.9 0.05 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def main() -> None:
    rclpy.init()
    node = TargetGazeboMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

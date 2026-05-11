from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState


class HumanBreathingNode(Node):
    """Apply micro-oscillation to a human model so mmWave can detect motion."""

    def __init__(self) -> None:
        super().__init__('human_breathing_node')

        self.declare_parameter('target_entity_name', 'human_0')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('breathing_frequency_hz', 1.0)
        self.declare_parameter('breathing_amplitude_m', 0.0015)
        self.declare_parameter('oscillation_axis', 'x')
        self.declare_parameter('update_rate_hz', 30.0)

        self.target_entity_name = str(self.get_parameter('target_entity_name').value)
        self.model_states_topic = str(self.get_parameter('model_states_topic').value)
        self.frequency_hz = float(self.get_parameter('breathing_frequency_hz').value)
        self.amplitude_m = float(self.get_parameter('breathing_amplitude_m').value)
        self.axis = str(self.get_parameter('oscillation_axis').value).lower()
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)

        self._set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self._latest_model_states: ModelStates | None = None
        self._base_pose = None
        self._start_time = self.get_clock().now()
        self._warned_no_service = False

        self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self._model_states_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / self.update_rate_hz, self._update_breathing)

        self.get_logger().info(
            'Human breathing node started '
            f'(target={self.target_entity_name}, f={self.frequency_hz:.2f} Hz, '
            f'amp={self.amplitude_m * 1000.0:.1f} mm, axis={self.axis})'
        )

    def _model_states_callback(self, msg: ModelStates) -> None:
        self._latest_model_states = msg
        if self._base_pose is not None:
            return

        try:
            index = msg.name.index(self.target_entity_name)
        except ValueError:
            return

        pose = msg.pose[index]
        self._base_pose = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

    def _update_breathing(self) -> None:
        if self._base_pose is None:
            return

        if not self._set_cli.wait_for_service(timeout_sec=0.0):
            if not self._warned_no_service:
                self.get_logger().warn('Waiting for /gazebo/set_entity_state service...')
                self._warned_no_service = True
            return

        self._warned_no_service = False

        elapsed = (
            self.get_clock().now().nanoseconds - self._start_time.nanoseconds
        ) / 1e9
        omega = 2.0 * math.pi * self.frequency_hz
        offset = self.amplitude_m * math.sin(omega * elapsed)
        velocity = self.amplitude_m * omega * math.cos(omega * elapsed)

        base_x, base_y, base_z, qx, qy, qz, qw = self._base_pose
        state = EntityState()
        state.name = self.target_entity_name
        state.reference_frame = 'world'
        state.pose.position.x = base_x
        state.pose.position.y = base_y
        state.pose.position.z = base_z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        if self.axis == 'y':
            state.pose.position.y += offset
            state.twist.linear.y = velocity
        elif self.axis == 'z':
            state.pose.position.z += offset
            state.twist.linear.z = velocity
        else:
            state.pose.position.x += offset
            state.twist.linear.x = velocity

        req = SetEntityState.Request()
        req.state = state
        self._set_cli.call_async(req)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HumanBreathingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

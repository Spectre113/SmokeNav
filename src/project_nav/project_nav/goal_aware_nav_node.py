import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from rclpy.duration import Duration


class GoalAwareNavNode(Node):
    def __init__(self) -> None:
        super().__init__('goal_aware_nav_node')

        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('perception_timeout', 1.0)

        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.6)

        self.declare_parameter('front_safe_distance', 0.8)
        self.declare_parameter('front_blocked_distance', 0.45)
        self.declare_parameter('side_safe_distance', 0.6)
        self.declare_parameter('wall_caution_distance', 0.7)
        self.declare_parameter('wall_critical_distance', 0.45)
        self.declare_parameter('wall_stop_distance', 0.28)
        self.declare_parameter('wall_linear_cap', 0.08)
        self.declare_parameter('side_danger_weight', 1.0)

        self.declare_parameter('goal_angle_gain', 1.0)
        self.declare_parameter('goal_distance_gain', 0.2)
        self.declare_parameter('avoid_turn_gain', 0.8)

        self.declare_parameter('target_stop_distance', 0.7)
        self.declare_parameter('target_confidence_threshold', 0.4)
        self.declare_parameter('prefer_left', True)
        self.declare_parameter('commit_time_sec', 1.2)
        self.declare_parameter('commit_side_margin', 0.2)
        self.declare_parameter('front_clear_cycles_required', 3)

        self.declare_parameter('obs_weight', 1.0)
        self.declare_parameter('target_weight', 0.8)
        self.declare_parameter('commit_weight', 0.4)
        self.declare_parameter('side_score_cap', 3.5)

        self.declare_parameter('free_topic', '/free_sectors')
        self.declare_parameter('distance_topic', '/sector_distances')
        self.declare_parameter('target_topic', '/target_info')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.perception_timeout = float(self.get_parameter('perception_timeout').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.front_safe_distance = float(self.get_parameter('front_safe_distance').value)
        self.front_blocked_distance = float(self.get_parameter('front_blocked_distance').value)
        self.side_safe_distance = float(self.get_parameter('side_safe_distance').value)
        self.wall_caution_distance = float(self.get_parameter('wall_caution_distance').value)
        self.wall_critical_distance = float(self.get_parameter('wall_critical_distance').value)
        self.wall_stop_distance = float(self.get_parameter('wall_stop_distance').value)
        self.wall_linear_cap = float(self.get_parameter('wall_linear_cap').value)
        self.side_danger_weight = float(self.get_parameter('side_danger_weight').value)

        self.goal_angle_gain = float(self.get_parameter('goal_angle_gain').value)
        self.goal_distance_gain = float(self.get_parameter('goal_distance_gain').value)
        self.avoid_turn_gain = float(self.get_parameter('avoid_turn_gain').value)

        self.target_stop_distance = float(self.get_parameter('target_stop_distance').value)
        self.target_confidence_threshold = float(self.get_parameter('target_confidence_threshold').value)
        self.prefer_left = bool(self.get_parameter('prefer_left').value)

        self.commit_time_sec = float(self.get_parameter('commit_time_sec').value)
        self.commit_side_margin = float(self.get_parameter('commit_side_margin').value)
        self.front_clear_cycles_required = int(self.get_parameter('front_clear_cycles_required').value)

        self.obs_weight = float(self.get_parameter('obs_weight').value)
        self.target_weight = float(self.get_parameter('target_weight').value)
        self.commit_weight = float(self.get_parameter('commit_weight').value)
        self.side_score_cap = float(self.get_parameter('side_score_cap').value)

        self.turn_commit_dir = 0
        self.turn_commit_until = None
        self.front_clear_counter = 0

        self.free_topic = str(self.get_parameter('free_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.free_sub = self.create_subscription(
            Int32MultiArray,
            self.free_topic,
            self.free_callback,
            10
        )
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            self.distance_topic,
            self.distance_callback,
            10
        )
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            self.target_topic,
            self.target_callback,
            10
        )

        self.left_free = False
        self.center_free = False
        self.right_free = False

        self.left_distance: Optional[float] = None
        self.center_distance: Optional[float] = None
        self.right_distance: Optional[float] = None

        self.target_detected = False
        self.target_angle = 0.0
        self.target_distance = 0.0
        self.target_confidence = 0.0

        self.last_free_time = None
        self.last_distance_time = None
        self.last_target_time = None

        self.last_decision: Optional[str] = None

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Goal-aware navigation node started (v2 smooth fusion)')

    def free_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 3:
            self.get_logger().warn('Invalid /free_sectors message')
            return

        self.left_free = bool(msg.data[0])
        self.center_free = bool(msg.data[1])
        self.right_free = bool(msg.data[2])
        self.last_free_time = self.get_clock().now()

    def distance_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 3:
            self.get_logger().warn('Invalid /sector_distances message')
            return

        self.left_distance = self.sanitize_distance(msg.data[0])
        self.center_distance = self.sanitize_distance(msg.data[1])
        self.right_distance = self.sanitize_distance(msg.data[2])
        self.last_distance_time = self.get_clock().now()

    def target_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn('Invalid /target_info message')
            return

        self.target_detected = bool(msg.data[0] > 0.5)
        self.target_angle = float(msg.data[1])
        self.target_distance = self.sanitize_distance(msg.data[2])
        self.target_confidence = float(msg.data[3])
        self.last_target_time = self.get_clock().now()

    def sanitize_distance(self, value: float) -> float:
        if math.isnan(value) or math.isinf(value):
            return 0.0
        return float(value)

    def control_loop(self) -> None:
        cmd, decision = self.compute_command()
        self.cmd_pub.publish(cmd)

        if decision != self.last_decision:
            self.get_logger().info(
                f'Decision: {decision} | '
                f'obs=({self.format_value(self.left_distance)}, '
                f'{self.format_value(self.center_distance)}, '
                f'{self.format_value(self.right_distance)}) | '
                f'target=(det={self.target_detected}, '
                f'ang={self.target_angle:.2f}, '
                f'dist={self.target_distance:.2f}, '
                f'conf={self.target_confidence:.2f}) | '
                f'cmd=(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f})'
            )
            self.last_decision = decision

    def compute_command(self) -> tuple[Twist, str]:
        if not self.has_fresh_data():
            return self.build_stop_cmd(), 'STOP_TIMEOUT'

        if self.left_distance is None or self.center_distance is None or self.right_distance is None:
            return self.build_stop_cmd(), 'STOP_NO_OBSTACLE_INFO'

        if not self.target_detected or self.target_confidence < self.target_confidence_threshold:
            return self.build_stop_cmd(), 'STOP_NO_TARGET'

        if 0.0 < self.target_distance <= self.target_stop_distance:
            return self.build_stop_cmd(), 'STOP_TARGET_REACHED'

        left_score = self.left_distance
        center_score = self.center_distance
        right_score = self.right_distance

        goal_linear = self.compute_goal_linear(self.target_distance)
        goal_angular = self.clamp(
            self.goal_angle_gain * self.target_angle,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        danger_alpha = self.compute_danger_alpha(center_score, left_score, right_score)

        self.update_turn_commit(left_score, center_score, right_score, danger_alpha)

        avoid_angular = self.compute_avoid_angular(left_score, right_score)
        avoid_angular = self.apply_turn_commit(avoid_angular)

        linear_x = (1.0 - danger_alpha) * goal_linear

        if center_score <= self.front_blocked_distance:
            linear_x = 0.0
        elif danger_alpha > 0.8:
            side_clearance = max(left_score, right_score)
            speed_boost = 0.1 * (side_clearance / max(self.side_safe_distance, 1e-6))
            linear_x = max(linear_x, min(0.15, speed_boost))

        if goal_linear > 0.0 and danger_alpha < 1.0:
            linear_x = max(self.min_linear_speed * (1.0 - danger_alpha), linear_x)

        if danger_alpha > 0.7:
            angular_z = avoid_angular
        else:
            angular_z = (1.0 - danger_alpha) * goal_angular + danger_alpha * avoid_angular

        min_side = min(left_score, right_score)
        if min_side < self.wall_caution_distance:
            if self.wall_caution_distance > self.wall_critical_distance:
                wall_alpha = self.clamp(
                    (self.wall_caution_distance - min_side)
                    / (self.wall_caution_distance - self.wall_critical_distance),
                    0.0,
                    1.0,
                )
            else:
                wall_alpha = 1.0

            if left_score < right_score:
                wall_away_angular = -self.max_angular_speed
            else:
                wall_away_angular = self.max_angular_speed

            angular_z = (1.0 - wall_alpha) * angular_z + wall_alpha * wall_away_angular
            linear_x = min(linear_x, self.wall_linear_cap)

        if min_side <= self.wall_stop_distance:
            linear_x = 0.0

        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        if center_score <= self.front_blocked_distance * 0.7 and left_score <= self.side_safe_distance and right_score <= self.side_safe_distance:
            return self.build_stop_cmd(), 'STOP_TRAPPED'

        if danger_alpha >= 0.95 and abs(angular_z) < 1e-3:
            angular_z = self.max_angular_speed if self.prefer_left else -self.max_angular_speed

        if abs(linear_x) < 0.02 and abs(angular_z) < 0.1:
            angular_z = self.max_angular_speed if self.prefer_left else -self.max_angular_speed

        cmd = self.build_cmd(linear_x, angular_z)

        if danger_alpha < 0.1:
            return cmd, 'GO_TO_TARGET_SMOOTH'
        if danger_alpha < 0.6:
            return cmd, 'GO_TO_TARGET_WITH_AVOID'
        return cmd, 'AVOID_WITH_TARGET_BIAS'
    
    def update_turn_commit(self, left_score: float, center_score: float, right_score: float, danger_alpha: float) -> None:
        now = self.get_clock().now()

        if center_score > self.front_safe_distance:
            self.front_clear_counter += 1
        else:
            self.front_clear_counter = 0

        if self.turn_commit_dir != 0:
            commit_expired = False
            if self.turn_commit_until is not None:
                commit_expired = now >= self.turn_commit_until

            committed_side_blocked = (
                self.turn_commit_dir > 0 and left_score < self.side_safe_distance
            ) or (
                self.turn_commit_dir < 0 and right_score < self.side_safe_distance
            )

            front_stably_clear = self.front_clear_counter >= self.front_clear_cycles_required

            if commit_expired or committed_side_blocked or front_stably_clear:
                self.turn_commit_dir = 0
                self.turn_commit_until = None

        if self.turn_commit_dir == 0:
            if danger_alpha < 0.6:
                return

            side_diff = left_score - right_score

            if side_diff > self.commit_side_margin and left_score > self.side_safe_distance:
                self.turn_commit_dir = 1
                self.turn_commit_until = now + Duration(seconds=self.commit_time_sec)

            elif side_diff < -self.commit_side_margin and right_score > self.side_safe_distance:
                self.turn_commit_dir = -1
                self.turn_commit_until = now + Duration(seconds=self.commit_time_sec)

    def apply_turn_commit(self, avoid_angular: float) -> float:
        if self.turn_commit_dir == 0:
            return avoid_angular

        min_commit_turn = 0.2 * self.max_angular_speed

        if self.turn_commit_dir > 0:
            return max(avoid_angular, min_commit_turn)

        return min(avoid_angular, -min_commit_turn)

    def compute_goal_linear(self, target_distance: float) -> float:
        goal_linear = self.goal_distance_gain * target_distance
        goal_linear = min(goal_linear, self.max_linear_speed)
        goal_linear = max(goal_linear, self.min_linear_speed)
        return goal_linear

    def compute_danger_alpha(self, center: float, left: float, right: float) -> float:
        front_alpha = 0.0

        if center <= self.front_blocked_distance:
            front_alpha = 1.0
        elif center < self.front_safe_distance:
            span = self.front_safe_distance - self.front_blocked_distance
            front_alpha = (self.front_safe_distance - center) / max(span, 1e-6)

        side_alpha = 0.0
        min_side = min(left, right)

        if min_side < self.side_safe_distance:
            side_alpha = (self.side_safe_distance - min_side) / max(self.side_safe_distance, 1e-6)

        alpha = max(front_alpha, self.side_danger_weight * side_alpha)
        return self.clamp(alpha, 0.0, 1.0)

    def compute_avoid_angular(self, left_score: float, right_score: float) -> float:
        left_clear = self.clamp(left_score / max(self.side_score_cap, 1e-6), 0.0, 1.0)
        right_clear = self.clamp(right_score / max(self.side_score_cap, 1e-6), 0.0, 1.0)

        left_clear = left_clear ** 2
        right_clear = right_clear ** 2

        target_left = max(0.0, self.target_angle)
        target_right = max(0.0, -self.target_angle)

        target_left = self.clamp(target_left / self.max_angular_speed, 0.0, 1.0)
        target_right = self.clamp(target_right / self.max_angular_speed, 0.0, 1.0)

        commit_left = 1.0 if self.turn_commit_dir > 0 else 0.0
        commit_right = 1.0 if self.turn_commit_dir < 0 else 0.0

        danger_left = 1.0 - left_clear
        danger_right = 1.0 - right_clear

        score_left = (
            self.obs_weight * left_clear +
            self.target_weight * target_left +
            self.commit_weight * commit_left -
            0.5 * danger_left
        )

        score_right = (
            self.obs_weight * right_clear +
            self.target_weight * target_right +
            self.commit_weight * commit_right -
            0.5 * danger_right
        )

        if left_score < 0.5:
            score_left *= 0.3

        if right_score < 0.5:
            score_right *= 0.3

        score_diff = score_left - score_right
        score_diff = self.clamp(score_diff, -1.0, 1.0)

        max_avoid = self.max_angular_speed * self.avoid_turn_gain
        avoid = math.tanh(score_diff * 2.0) * max_avoid

        if abs(score_diff) < 0.05:
            avoid = 0.15 if self.prefer_left else -0.15

        return avoid

    def has_fresh_data(self) -> bool:
        now = self.get_clock().now()

        if self.last_free_time is None or self.last_distance_time is None or self.last_target_time is None:
            return False

        dt_free = (now - self.last_free_time).nanoseconds / 1e9
        dt_dist = (now - self.last_distance_time).nanoseconds / 1e9
        dt_target = (now - self.last_target_time).nanoseconds / 1e9

        return (
            dt_free <= self.perception_timeout and
            dt_dist <= self.perception_timeout and
            dt_target <= self.perception_timeout
        )

    def build_stop_cmd(self) -> Twist:
        return self.build_cmd(0.0, 0.0)

    def build_cmd(self, linear_x: float, angular_z: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        return cmd

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def format_value(self, value: Optional[float]) -> str:
        if value is None:
            return 'None'
        return f'{value:.2f}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalAwareNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().warn(f'Shutting down due to exception: {e}')
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

import json
import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray, String
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
        self.declare_parameter('centerline_gain', 0.25)
        self.declare_parameter('max_centerline_angular', 0.25)
        self.declare_parameter('passage_mode_enabled', True)
        self.declare_parameter('passage_front_clear_distance', 0.75)
        self.declare_parameter('passage_min_side_distance', 0.18)
        self.declare_parameter('passage_danger_alpha_cap', 0.35)
        self.declare_parameter('passage_linear_speed', 0.10)
        self.declare_parameter('passage_centerline_gain', 0.45)
        self.declare_parameter('passage_max_centerline_angular', 0.35)
        self.declare_parameter('passage_target_angular_weight', 0.45)
        self.declare_parameter('avoid_target_angular_weight', 0.30)

        self.declare_parameter('goal_angle_gain', 1.0)
        self.declare_parameter('goal_distance_gain', 0.2)
        self.declare_parameter('avoid_turn_gain', 0.8)
        self.declare_parameter('align_angle_threshold', 0.75)
        self.declare_parameter('align_linear_scale', 0.25)

        self.declare_parameter('target_stop_distance', 0.7)
        self.declare_parameter('target_confidence_threshold', 0.4)
        self.declare_parameter('require_target', False)
        self.declare_parameter('use_target_memory', True)
        self.declare_parameter('target_memory_timeout', 2.0)
        self.declare_parameter('target_hint_stop_distance', 0.8)
        self.declare_parameter('target_reached_hold_sec', 0.8)
        self.declare_parameter('target_reacquire_distance', 1.1)
        self.declare_parameter('clear_target_after_reached', True)
        self.declare_parameter('search_linear_speed', 0.12)
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
        self.declare_parameter('detailed_distance_topic', '/sector_distances_detailed')
        self.declare_parameter('exploration_hint_topic', '/exploration_hint')
        self.declare_parameter('sensor_metrics_topic', '/sensor_fusion_metrics')
        self.declare_parameter('nav_metrics_topic', '/navigation_metrics')
        self.declare_parameter('smoke_density_topic', '/smoke/density')
        self.declare_parameter('target_topic', '/target_info')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detailed_sector_outer_angle_deg', 90.0)
        self.declare_parameter('exploration_turn_gain', 0.8)
        self.declare_parameter('exploration_center_bias', 0.35)
        self.declare_parameter('use_global_exploration', True)
        self.declare_parameter('exploration_hint_timeout', 2.0)
        self.declare_parameter('metrics_publish_rate', 1.0)

        self.declare_parameter('stuck_window_sec', 2.0)
        self.declare_parameter('stuck_min_progress_m', 0.05)
        self.declare_parameter('stuck_cmd_linear_threshold', 0.06)
        self.declare_parameter('recovery_reverse_speed', -0.06)
        self.declare_parameter('recovery_turn_speed', 0.8)
        self.declare_parameter('recovery_duration_sec', 1.4)

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
        self.centerline_gain = float(self.get_parameter('centerline_gain').value)
        self.max_centerline_angular = float(self.get_parameter('max_centerline_angular').value)
        self.passage_mode_enabled = bool(
            self.get_parameter('passage_mode_enabled').value
        )
        self.passage_front_clear_distance = float(
            self.get_parameter('passage_front_clear_distance').value
        )
        self.passage_min_side_distance = float(
            self.get_parameter('passage_min_side_distance').value
        )
        self.passage_danger_alpha_cap = float(
            self.get_parameter('passage_danger_alpha_cap').value
        )
        self.passage_linear_speed = float(
            self.get_parameter('passage_linear_speed').value
        )
        self.passage_centerline_gain = float(
            self.get_parameter('passage_centerline_gain').value
        )
        self.passage_max_centerline_angular = float(
            self.get_parameter('passage_max_centerline_angular').value
        )
        self.passage_target_angular_weight = float(
            self.get_parameter('passage_target_angular_weight').value
        )
        self.avoid_target_angular_weight = float(
            self.get_parameter('avoid_target_angular_weight').value
        )

        self.goal_angle_gain = float(self.get_parameter('goal_angle_gain').value)
        self.goal_distance_gain = float(self.get_parameter('goal_distance_gain').value)
        self.avoid_turn_gain = float(self.get_parameter('avoid_turn_gain').value)
        self.align_angle_threshold = float(self.get_parameter('align_angle_threshold').value)
        self.align_linear_scale = float(self.get_parameter('align_linear_scale').value)

        self.target_stop_distance = float(self.get_parameter('target_stop_distance').value)
        self.target_confidence_threshold = float(
            self.get_parameter('target_confidence_threshold').value
        )
        self.require_target = bool(self.get_parameter('require_target').value)
        self.use_target_memory = bool(self.get_parameter('use_target_memory').value)
        self.target_memory_timeout = float(self.get_parameter('target_memory_timeout').value)
        self.target_hint_stop_distance = float(
            self.get_parameter('target_hint_stop_distance').value
        )
        self.target_reached_hold_sec = float(
            self.get_parameter('target_reached_hold_sec').value
        )
        self.target_reacquire_distance = float(
            self.get_parameter('target_reacquire_distance').value
        )
        self.clear_target_after_reached = bool(
            self.get_parameter('clear_target_after_reached').value
        )
        self.search_linear_speed = float(self.get_parameter('search_linear_speed').value)
        self.prefer_left = bool(self.get_parameter('prefer_left').value)

        self.commit_time_sec = float(self.get_parameter('commit_time_sec').value)
        self.commit_side_margin = float(self.get_parameter('commit_side_margin').value)
        self.front_clear_cycles_required = int(
            self.get_parameter('front_clear_cycles_required').value
        )

        self.obs_weight = float(self.get_parameter('obs_weight').value)
        self.target_weight = float(self.get_parameter('target_weight').value)
        self.commit_weight = float(self.get_parameter('commit_weight').value)
        self.side_score_cap = float(self.get_parameter('side_score_cap').value)

        self.turn_commit_dir = 0
        self.turn_commit_until = None
        self.front_clear_counter = 0

        self.free_topic = str(self.get_parameter('free_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)
        self.detailed_distance_topic = str(
            self.get_parameter('detailed_distance_topic').value
        )
        self.exploration_hint_topic = str(self.get_parameter('exploration_hint_topic').value)
        self.sensor_metrics_topic = str(self.get_parameter('sensor_metrics_topic').value)
        self.nav_metrics_topic = str(self.get_parameter('nav_metrics_topic').value)
        self.smoke_density_topic = str(self.get_parameter('smoke_density_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.detailed_sector_outer_angle_deg = float(
            self.get_parameter('detailed_sector_outer_angle_deg').value
        )
        self.exploration_turn_gain = float(self.get_parameter('exploration_turn_gain').value)
        self.exploration_center_bias = float(self.get_parameter('exploration_center_bias').value)
        self.use_global_exploration = bool(
            self.get_parameter('use_global_exploration').value
        )
        self.exploration_hint_timeout = float(
            self.get_parameter('exploration_hint_timeout').value
        )
        self.metrics_publish_rate = float(self.get_parameter('metrics_publish_rate').value)

        self.stuck_window_sec = float(self.get_parameter('stuck_window_sec').value)
        self.stuck_min_progress_m = float(self.get_parameter('stuck_min_progress_m').value)
        self.stuck_cmd_linear_threshold = float(
            self.get_parameter('stuck_cmd_linear_threshold').value
        )
        self.recovery_reverse_speed = float(self.get_parameter('recovery_reverse_speed').value)
        self.recovery_turn_speed = float(self.get_parameter('recovery_turn_speed').value)
        self.recovery_duration_sec = float(self.get_parameter('recovery_duration_sec').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.metrics_pub = self.create_publisher(String, self.nav_metrics_topic, 10)

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
        self.detailed_distance_sub = self.create_subscription(
            Float32MultiArray,
            self.detailed_distance_topic,
            self.detailed_distance_callback,
            10
        )
        self.exploration_hint_sub = self.create_subscription(
            Float32MultiArray,
            self.exploration_hint_topic,
            self.exploration_hint_callback,
            10
        )
        self.sensor_metrics_sub = self.create_subscription(
            String,
            self.sensor_metrics_topic,
            self.sensor_metrics_callback,
            10
        )
        self.smoke_density_sub = self.create_subscription(
            Float32,
            self.smoke_density_topic,
            self.smoke_density_callback,
            10
        )
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            self.target_topic,
            self.target_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.left_free = False
        self.center_free = False
        self.right_free = False

        self.left_distance: Optional[float] = None
        self.center_distance: Optional[float] = None
        self.right_distance: Optional[float] = None
        self.detailed_distances: list[float] = []
        self.last_detailed_distance_time = None
        self.exploration_hint_valid = False
        self.exploration_hint_angle = 0.0
        self.exploration_hint_distance = 0.0
        self.exploration_hint_score = 0.0
        self.last_exploration_hint_time = None

        self.target_detected = False
        self.target_angle = 0.0
        self.target_distance = 0.0
        self.target_confidence = 0.0
        self.last_valid_target_time = None
        self.last_valid_target_angle = 0.0
        self.last_valid_target_distance = 0.0
        self.last_valid_target_confidence = 0.0
        self.target_reached_latched = False
        self.target_reached_until = None

        self.last_free_time = None
        self.last_distance_time = None
        self.last_target_time = None
        self.odom_history = deque(maxlen=400)
        self.last_odom_pose: Optional[tuple[float, float]] = None

        self.recovery_active = False
        self.recovery_until = None
        self.recovery_turn_sign = 1.0

        self.last_decision: Optional[str] = None
        self.last_metrics_time = None
        self.path_length_m = 0.0
        self.stuck_events = 0
        self.target_reached_events = 0
        self.collision_risk_events = 0
        self.collision_risk_active = False
        self.min_clearance_seen = float('inf')
        self.latest_sensor_min_clearance: Optional[float] = None
        self.latest_smoke_density: Optional[float] = None
        self.target_session_start = None
        self.last_time_to_target_sec: Optional[float] = None

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            'Goal-aware navigation node started (target optional, fused obstacles)'
        )

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

    def detailed_distance_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn('Invalid /sector_distances_detailed message')
            return

        self.detailed_distances = [self.sanitize_distance(value) for value in msg.data]
        self.last_detailed_distance_time = self.get_clock().now()

    def exploration_hint_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn('Invalid /exploration_hint message')
            return

        self.exploration_hint_valid = bool(msg.data[0] > 0.5)
        self.exploration_hint_angle = float(msg.data[1])
        self.exploration_hint_distance = self.sanitize_distance(msg.data[2])
        self.exploration_hint_score = float(msg.data[3])
        self.last_exploration_hint_time = self.get_clock().now()

    def sensor_metrics_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        min_clearance = data.get('min_clearance_m')
        if isinstance(min_clearance, (int, float)) and math.isfinite(min_clearance):
            self.latest_sensor_min_clearance = float(min_clearance)

        smoke_density = data.get('smoke_density')
        if isinstance(smoke_density, (int, float)) and math.isfinite(smoke_density):
            self.latest_smoke_density = float(smoke_density)

    def smoke_density_callback(self, msg: Float32) -> None:
        self.latest_smoke_density = float(msg.data)

    def target_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn('Invalid /target_info message')
            return

        self.target_detected = bool(msg.data[0] > 0.5)
        self.target_angle = float(msg.data[1])
        self.target_distance = self.sanitize_distance(msg.data[2])
        self.target_confidence = float(msg.data[3])
        self.last_target_time = self.get_clock().now()

        if self.target_reached_latched:
            if self.should_reacquire_target_after_reached():
                self.clear_target_reached_latch()
            else:
                return

        if self.is_current_target_valid():
            if self.last_valid_target_time is None:
                self.target_session_start = self.get_clock().now()
            self.last_valid_target_time = self.last_target_time
            self.last_valid_target_angle = self.target_angle
            self.last_valid_target_distance = self.target_distance
            self.last_valid_target_confidence = self.target_confidence

    def odom_callback(self, msg: Odometry) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.odom_history.append((stamp_sec, x, y))

        if self.last_odom_pose is not None:
            dx = x - self.last_odom_pose[0]
            dy = y - self.last_odom_pose[1]
            step = math.hypot(dx, dy)
            if step < 1.0:
                self.path_length_m += step
        self.last_odom_pose = (x, y)

        # Keep about 3x window history only.
        keep_sec = max(3.0 * self.stuck_window_sec, 3.0)
        while (
            len(self.odom_history) >= 2 and
            (stamp_sec - self.odom_history[0][0]) > keep_sec
        ):
            self.odom_history.popleft()

    def sanitize_distance(self, value: float) -> float:
        if math.isnan(value) or math.isinf(value):
            return 0.0
        return float(value)

    def control_loop(self) -> None:
        cmd, decision = self.compute_command()
        self.cmd_pub.publish(cmd)
        self.update_runtime_metrics(decision)
        self.publish_runtime_metrics(decision, cmd)

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
        if self.recovery_active:
            now = self.get_clock().now()
            if self.recovery_until is not None and now < self.recovery_until:
                return self.build_cmd(
                    self.recovery_reverse_speed,
                    self.recovery_turn_sign * self.recovery_turn_speed
                ), 'RECOVERY_STUCK'
            self.recovery_active = False
            self.recovery_until = None

        if not self.has_fresh_obstacle_data():
            return self.build_stop_cmd(), 'STOP_TIMEOUT'

        if (
            self.left_distance is None or
            self.center_distance is None or
            self.right_distance is None
        ):
            return self.build_stop_cmd(), 'STOP_NO_OBSTACLE_INFO'

        nav_target = self.get_navigation_target()
        has_target = nav_target is not None

        if self.require_target and not has_target:
            return self.build_stop_cmd(), 'STOP_NO_TARGET'

        if self.should_stop_for_target_hint():
            return self.build_stop_cmd(), 'STOP_TARGET_REACHED'

        left_score = self.left_distance
        center_score = self.center_distance
        right_score = self.right_distance

        if has_target:
            target_angle, target_distance, target_mode = nav_target
            goal_linear = self.compute_goal_linear(target_distance)
            goal_angular = self.clamp(
                self.goal_angle_gain * target_angle,
                -self.max_angular_speed,
                self.max_angular_speed
            )
        else:
            target_angle = self.get_exploration_angle()
            target_mode = 'explore'
            goal_linear = self.search_linear_speed
            goal_angular = self.clamp(
                self.exploration_turn_gain * target_angle,
                -self.max_angular_speed,
                self.max_angular_speed
            )
        global_explore = (not has_target) and self.has_fresh_exploration_hint()

        danger_alpha = self.compute_danger_alpha(center_score, left_score, right_score)
        passage_mode = self.is_passage_mode(center_score, left_score, right_score)

        if passage_mode:
            danger_alpha = min(danger_alpha, self.passage_danger_alpha_cap)

        self.update_turn_commit(left_score, center_score, right_score, danger_alpha)

        avoid_angular = self.compute_avoid_angular(left_score, right_score, target_angle)
        if not has_target and danger_alpha < 0.2:
            avoid_angular = 0.0
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

        if passage_mode:
            linear_x = max(linear_x, self.passage_linear_speed)

        if (
            not passage_mode and
            has_target and
            abs(target_angle) > self.align_angle_threshold and
            danger_alpha < 0.5
        ):
            linear_x = min(linear_x, self.min_linear_speed * self.align_linear_scale)

        if (
            not passage_mode and
            global_explore and
            abs(target_angle) > self.align_angle_threshold and
            danger_alpha < 0.5
        ):
            linear_x = min(linear_x, self.min_linear_speed * 0.8)

        if passage_mode and has_target:
            angular_z = self.passage_target_angular_weight * goal_angular
        elif passage_mode and global_explore and abs(target_angle) > 0.5:
            angular_z = 0.85 * goal_angular
        elif passage_mode:
            angular_z = 0.0
        elif danger_alpha > 0.7 and has_target:
            angular_z = (
                self.avoid_target_angular_weight * goal_angular +
                (1.0 - self.avoid_target_angular_weight) * avoid_angular
            )
        elif danger_alpha > 0.7:
            angular_z = avoid_angular
        elif has_target:
            angular_z = (
                (1.0 - danger_alpha) * goal_angular +
                danger_alpha * avoid_angular
            )
        else:
            angular_z = (
                (1.0 - danger_alpha) * goal_angular +
                danger_alpha * avoid_angular
            )

        # Corridor centering to avoid wall-hugging in room transitions.
        if center_score > self.front_blocked_distance:
            if passage_mode:
                centerline_angular = self.compute_passage_centerline_angular(
                    left_score,
                    right_score
                )
            else:
                centerline_angular = self.compute_centerline_angular(
                    left_score,
                    right_score
                )
            angular_z += centerline_angular

        min_side = min(left_score, right_score)
        if min_side < self.wall_caution_distance and not passage_mode:
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

        if min_side <= self.wall_stop_distance and not passage_mode:
            linear_x = 0.0

        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        if (
            center_score <= self.front_blocked_distance * 0.7 and
            left_score <= self.side_safe_distance and
            right_score <= self.side_safe_distance
        ):
            return self.build_stop_cmd(), 'STOP_TRAPPED'

        if danger_alpha >= 0.95 and abs(angular_z) < 1e-3:
            angular_z = self.max_angular_speed if self.prefer_left else -self.max_angular_speed

        if abs(linear_x) < 0.02 and abs(angular_z) < 0.1:
            angular_z = self.max_angular_speed if self.prefer_left else -self.max_angular_speed

        if self.should_trigger_recovery(
            linear_x,
            left_score,
            center_score,
            right_score,
            passage_mode,
        ):
            self.start_recovery(left_score, right_score)
            return self.build_cmd(
                self.recovery_reverse_speed,
                self.recovery_turn_sign * self.recovery_turn_speed
            ), 'RECOVERY_STUCK'

        cmd = self.build_cmd(linear_x, angular_z)

        if not has_target:
            if passage_mode:
                return cmd, 'EXPLORE_THROUGH_PASSAGE'
            if danger_alpha < 0.1:
                return cmd, 'EXPLORE_CLEAR'
            if danger_alpha < 0.6:
                return cmd, 'EXPLORE_WITH_AVOID'
            return cmd, 'AVOID_NO_TARGET'

        if target_mode == 'memory':
            if danger_alpha < 0.1:
                return cmd, 'GO_TO_LAST_TARGET'
            if passage_mode:
                return cmd, 'GO_TO_LAST_TARGET_THROUGH_PASSAGE'
            return cmd, 'GO_TO_LAST_TARGET_WITH_AVOID'

        if danger_alpha < 0.1:
            return cmd, 'GO_TO_TARGET_SMOOTH'
        if passage_mode:
            return cmd, 'GO_TO_TARGET_THROUGH_PASSAGE'
        if danger_alpha < 0.6:
            return cmd, 'GO_TO_TARGET_WITH_AVOID'
        return cmd, 'AVOID_WITH_TARGET_BIAS'

    def update_turn_commit(
        self,
        left_score: float,
        center_score: float,
        right_score: float,
        danger_alpha: float,
    ) -> None:
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
            side_alpha = (
                (self.side_safe_distance - min_side) /
                max(self.side_safe_distance, 1e-6)
            )

        alpha = max(front_alpha, self.side_danger_weight * side_alpha)
        return self.clamp(alpha, 0.0, 1.0)

    def compute_avoid_angular(
        self,
        left_score: float,
        right_score: float,
        target_angle: float,
    ) -> float:
        left_clear = self.clamp(left_score / max(self.side_score_cap, 1e-6), 0.0, 1.0)
        right_clear = self.clamp(right_score / max(self.side_score_cap, 1e-6), 0.0, 1.0)

        left_clear = left_clear ** 2
        right_clear = right_clear ** 2

        target_left = max(0.0, target_angle)
        target_right = max(0.0, -target_angle)

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

    def get_exploration_angle(self) -> float:
        if self.has_fresh_exploration_hint():
            return self.clamp(
                self.exploration_hint_angle,
                -self.max_angular_speed * 2.5,
                self.max_angular_speed * 2.5,
            )

        if self.has_fresh_detailed_obstacle_data() and self.detailed_distances:
            return self.best_detailed_sector_angle()

        if (
            self.left_distance is None or
            self.center_distance is None or
            self.right_distance is None
        ):
            return 0.0

        if self.center_distance > self.front_safe_distance:
            return 0.0

        return 0.7 if self.left_distance >= self.right_distance else -0.7

    def has_fresh_exploration_hint(self) -> bool:
        if not self.use_global_exploration:
            return False
        if not self.exploration_hint_valid or self.last_exploration_hint_time is None:
            return False
        dt_hint = (
            self.get_clock().now() - self.last_exploration_hint_time
        ).nanoseconds / 1e9
        return dt_hint <= self.exploration_hint_timeout

    def best_detailed_sector_angle(self) -> float:
        distances = self.detailed_distances
        if not distances:
            return 0.0

        count = len(distances)
        side_outer = math.radians(self.detailed_sector_outer_angle_deg)
        best_index = count // 2
        best_score = -float('inf')

        for index, distance in enumerate(distances):
            angle = self.detailed_sector_angle(index, count, side_outer)
            if distance <= self.front_blocked_distance:
                score = -10.0 + distance
            else:
                turn_penalty = self.exploration_center_bias * abs(angle) / max(side_outer, 1e-6)
                score = min(distance, self.side_score_cap) - turn_penalty

            if score > best_score:
                best_score = score
                best_index = index

        return self.detailed_sector_angle(best_index, count, side_outer)

    def detailed_sector_angle(self, index: int, count: int, side_outer: float) -> float:
        if count <= 1:
            return 0.0
        ratio = index / float(count - 1)
        return -side_outer + ratio * 2.0 * side_outer

    def compute_centerline_angular(self, left_score: float, right_score: float) -> float:
        denom = max(left_score + right_score, 1e-6)
        imbalance = (left_score - right_score) / denom
        correction = self.centerline_gain * imbalance
        return self.clamp(
            correction,
            -self.max_centerline_angular,
            self.max_centerline_angular,
        )

    def compute_passage_centerline_angular(
        self,
        left_score: float,
        right_score: float,
    ) -> float:
        denom = max(left_score + right_score, 1e-6)
        imbalance = (left_score - right_score) / denom
        correction = self.passage_centerline_gain * imbalance
        return self.clamp(
            correction,
            -self.passage_max_centerline_angular,
            self.passage_max_centerline_angular,
        )

    def is_passage_mode(
        self,
        center_score: float,
        left_score: float,
        right_score: float,
    ) -> bool:
        if not self.passage_mode_enabled:
            return False

        min_side = min(left_score, right_score)
        close_to_side = min_side < self.wall_caution_distance

        return (
            center_score >= self.passage_front_clear_distance and
            min_side >= self.passage_min_side_distance and
            close_to_side
        )

    def should_trigger_recovery(
        self,
        commanded_linear: float,
        left_score: float,
        center_score: float,
        right_score: float,
        passage_mode: bool,
    ) -> bool:
        if self.recovery_active:
            return False
        if passage_mode:
            return False
        if abs(commanded_linear) < self.stuck_cmd_linear_threshold:
            return False
        if len(self.odom_history) < 2:
            return False

        obstacle_constrained = (
            center_score <= self.front_safe_distance or
            min(left_score, right_score) <= self.wall_caution_distance or
            not self.center_free or
            not self.left_free or
            not self.right_free
        )
        if not obstacle_constrained:
            return False

        progress = self.get_progress_over_window(self.stuck_window_sec)
        return progress < self.stuck_min_progress_m

    def get_progress_over_window(self, window_sec: float) -> float:
        if len(self.odom_history) < 2:
            return float('inf')

        t_now, x_now, y_now = self.odom_history[-1]
        t_cut = t_now - window_sec

        x_ref = self.odom_history[0][1]
        y_ref = self.odom_history[0][2]
        for t, x, y in self.odom_history:
            if t >= t_cut:
                x_ref, y_ref = x, y
                break

        return math.hypot(x_now - x_ref, y_now - y_ref)

    def start_recovery(self, left_score: float, right_score: float) -> None:
        self.recovery_turn_sign = -1.0 if left_score < right_score else 1.0
        self.recovery_active = True
        self.stuck_events += 1
        self.recovery_until = (
            self.get_clock().now() +
            Duration(seconds=self.recovery_duration_sec)
        )
        turn_dir = 'right' if self.recovery_turn_sign < 0.0 else 'left'
        self.get_logger().warn(
            'Stuck detected: entering recovery for '
            f'{self.recovery_duration_sec:.2f}s, turn={turn_dir}'
        )

    def has_fresh_data(self) -> bool:
        return self.has_fresh_obstacle_data()

    def has_fresh_obstacle_data(self) -> bool:
        now = self.get_clock().now()

        if self.last_free_time is None or self.last_distance_time is None:
            return False

        dt_free = (now - self.last_free_time).nanoseconds / 1e9
        dt_dist = (now - self.last_distance_time).nanoseconds / 1e9

        return (
            dt_free <= self.perception_timeout and
            dt_dist <= self.perception_timeout
        )

    def has_fresh_detailed_obstacle_data(self) -> bool:
        if self.last_detailed_distance_time is None:
            return False
        dt_detailed = (
            self.get_clock().now() - self.last_detailed_distance_time
        ).nanoseconds / 1e9
        return dt_detailed <= self.perception_timeout

    def has_fresh_target_data(self, timeout: float) -> bool:
        if self.last_target_time is None:
            return False
        dt_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        return dt_target <= timeout

    def is_current_target_valid(self) -> bool:
        return (
            self.target_detected and
            self.target_confidence >= self.target_confidence_threshold and
            self.target_distance > 0.0
        )

    def get_navigation_target(self) -> Optional[tuple[float, float, str]]:
        if self.target_reached_latched:
            return None

        if self.has_fresh_target_data(self.perception_timeout) and self.is_current_target_valid():
            return self.target_angle, self.target_distance, 'current'

        if not self.use_target_memory or self.last_valid_target_time is None:
            return None

        dt_memory = (self.get_clock().now() - self.last_valid_target_time).nanoseconds / 1e9
        if dt_memory > self.target_memory_timeout:
            return None

        return self.last_valid_target_angle, self.last_valid_target_distance, 'memory'

    def should_stop_for_target_hint(self) -> bool:
        now = self.get_clock().now()
        if self.target_reached_latched:
            return self.target_reached_until is not None and now < self.target_reached_until

        if self.has_fresh_target_data(self.target_memory_timeout):
            current_target_reached = (
                self.target_detected and
                self.target_confidence >= self.target_confidence_threshold and
                0.0 < self.target_distance <= self.target_stop_distance
            )
            hidden_target_reached = (
                not self.target_detected and
                self.target_confidence >= self.target_confidence_threshold and
                0.0 < self.target_distance <= self.target_hint_stop_distance
            )
            if current_target_reached or hidden_target_reached:
                self.mark_target_reached()
                return True

        if self.last_valid_target_time is None:
            return False

        dt_memory = (self.get_clock().now() - self.last_valid_target_time).nanoseconds / 1e9
        memory_target_reached = (
            dt_memory <= self.target_memory_timeout and
            0.0 < self.last_valid_target_distance <= self.target_stop_distance
        )
        if memory_target_reached:
            self.mark_target_reached()
        return memory_target_reached

    def mark_target_reached(self) -> None:
        if self.target_reached_latched:
            return

        self.target_reached_latched = True
        self.target_reached_until = (
            self.get_clock().now() +
            Duration(seconds=max(0.0, self.target_reached_hold_sec))
        )

        if self.clear_target_after_reached:
            self.last_valid_target_time = None
            self.last_valid_target_angle = 0.0
            self.last_valid_target_distance = 0.0
            self.last_valid_target_confidence = 0.0
            self.target_session_start = None

    def should_reacquire_target_after_reached(self) -> bool:
        return (
            self.target_detected and
            self.target_confidence >= self.target_confidence_threshold and
            self.target_distance >= self.target_reacquire_distance
        )

    def clear_target_reached_latch(self) -> None:
        self.target_reached_latched = False
        self.target_reached_until = None

    def update_runtime_metrics(self, decision: str) -> None:
        clearances = [
            value for value in (
                self.left_distance,
                self.center_distance,
                self.right_distance,
                self.latest_sensor_min_clearance,
            )
            if value is not None and value > 0.0 and math.isfinite(value)
        ]
        if clearances:
            current_min = min(clearances)
            self.min_clearance_seen = min(self.min_clearance_seen, current_min)
        else:
            current_min = None

        risk_now = bool(
            current_min is not None and
            (
                current_min <= self.wall_stop_distance or
                (
                    self.center_distance is not None and
                    self.center_distance <= self.front_blocked_distance * 0.8
                )
            )
        )
        if risk_now and not self.collision_risk_active:
            self.collision_risk_events += 1
        self.collision_risk_active = risk_now

        if decision == 'STOP_TARGET_REACHED' and self.last_decision != 'STOP_TARGET_REACHED':
            self.target_reached_events += 1
            if self.target_session_start is not None:
                elapsed = (self.get_clock().now() - self.target_session_start).nanoseconds / 1e9
                self.last_time_to_target_sec = elapsed
            self.target_session_start = None
            self.last_valid_target_time = None

    def publish_runtime_metrics(self, decision: str, cmd: Twist) -> None:
        now = self.get_clock().now()
        if self.last_metrics_time is not None:
            dt = (now - self.last_metrics_time).nanoseconds / 1e9
            if dt < 1.0 / max(self.metrics_publish_rate, 1e-6):
                return
        self.last_metrics_time = now

        min_clearance = None
        if math.isfinite(self.min_clearance_seen):
            min_clearance = self.min_clearance_seen

        metrics = {
            'decision': decision,
            'path_length_m': self.path_length_m,
            'stuck_events': self.stuck_events,
            'target_reached_events': self.target_reached_events,
            'last_time_to_target_sec': self.last_time_to_target_sec,
            'collision_risk_events': self.collision_risk_events,
            'min_clearance_seen_m': min_clearance,
            'sensor_min_clearance_m': self.latest_sensor_min_clearance,
            'smoke_density': self.latest_smoke_density,
            'target_detected': self.target_detected,
            'target_confidence': self.target_confidence,
            'exploration_hint_valid': self.has_fresh_exploration_hint(),
            'exploration_hint_angle': self.exploration_hint_angle,
            'exploration_hint_distance': self.exploration_hint_distance,
            'cmd_linear_x': cmd.linear.x,
            'cmd_angular_z': cmd.angular.z,
        }

        msg = String()
        msg.data = json.dumps(metrics, separators=(',', ':'))
        self.metrics_pub.publish(msg)

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

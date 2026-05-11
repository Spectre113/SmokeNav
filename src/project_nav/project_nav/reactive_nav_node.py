import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist


class ReactiveNavNode(Node):
    def __init__(self) -> None:
        super().__init__('reactive_nav_node')

        # Speeds
        self.declare_parameter('forward_speed_fast', 0.25)
        self.declare_parameter('forward_speed_slow', 0.12)
        self.declare_parameter('forward_speed_turn', 0.08)

        self.declare_parameter('turn_speed_in_place', 0.60)
        self.declare_parameter('turn_speed_moving', 0.35)

        # Timing
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('perception_timeout', 1.0)

        # Distances
        self.declare_parameter('front_safe_distance', 0.8)
        self.declare_parameter('front_clear_distance', 1.5)
        self.declare_parameter('front_turn_distance', 0.5)
        self.declare_parameter('side_safe_distance', 0.6)

        # Decision policy
        self.declare_parameter('turn_margin', 0.15)
        self.declare_parameter('prefer_left', True)

        # Topics
        self.declare_parameter('free_topic', '/free_sectors')
        self.declare_parameter('distance_topic', '/sector_distances')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.forward_speed_fast = float(self.get_parameter('forward_speed_fast').value)
        self.forward_speed_slow = float(self.get_parameter('forward_speed_slow').value)
        self.forward_speed_turn = float(self.get_parameter('forward_speed_turn').value)

        self.turn_speed_in_place = float(self.get_parameter('turn_speed_in_place').value)
        self.turn_speed_moving = float(self.get_parameter('turn_speed_moving').value)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.perception_timeout = float(self.get_parameter('perception_timeout').value)

        self.front_safe_distance = float(self.get_parameter('front_safe_distance').value)
        self.front_clear_distance = float(self.get_parameter('front_clear_distance').value)
        self.front_turn_distance = float(self.get_parameter('front_turn_distance').value)
        self.side_safe_distance = float(self.get_parameter('side_safe_distance').value)

        self.turn_margin = float(self.get_parameter('turn_margin').value)
        self.prefer_left = bool(self.get_parameter('prefer_left').value)

        self.free_topic = str(self.get_parameter('free_topic').value)
        self.distance_topic = str(self.get_parameter('distance_topic').value)
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

        self.left_free = False
        self.center_free = False
        self.right_free = False

        self.left_distance: Optional[float] = None
        self.center_distance: Optional[float] = None
        self.right_distance: Optional[float] = None

        self.last_free_time = None
        self.last_distance_time = None

        self.last_decision: Optional[str] = None

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Reactive navigation node started (smooth local control)')

    def free_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 3:
            self.get_logger().warn('Invalid /free_sectors message. Expected [left, center, right]')
            return

        self.left_free = bool(msg.data[0])
        self.center_free = bool(msg.data[1])
        self.right_free = bool(msg.data[2])
        self.last_free_time = self.get_clock().now()

    def distance_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 3:
            self.get_logger().warn(
                'Invalid /sector_distances message. Expected [left, center, right]'
            )
            return

        self.left_distance = self.sanitize_distance(msg.data[0])
        self.center_distance = self.sanitize_distance(msg.data[1])
        self.right_distance = self.sanitize_distance(msg.data[2])
        self.last_distance_time = self.get_clock().now()

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
                f'free=({self.left_free}, {self.center_free}, {self.right_free}) | '
                f'dist=({self.format_distance(self.left_distance)}, '
                f'{self.format_distance(self.center_distance)}, '
                f'{self.format_distance(self.right_distance)}) | '
                f'cmd=(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f})'
            )
            self.last_decision = decision

    def compute_command(self) -> tuple[Twist, str]:
        if not self.has_fresh_data():
            return self.build_stop_cmd(), 'STOP_TIMEOUT'

        if (
            self.left_distance is None or
            self.center_distance is None or
            self.right_distance is None
        ):
            return self.build_stop_cmd(), 'STOP_NO_DISTANCES'

        left_score = self.left_distance
        center_score = self.center_distance
        right_score = self.right_distance

        if center_score > self.front_clear_distance:
            return self.build_forward_cmd(self.forward_speed_fast), 'GO_FORWARD_FAST'

        if center_score > self.front_safe_distance:
            if left_score > right_score + self.turn_margin:
                return self.build_arc_cmd(
                    linear_speed=self.forward_speed_slow,
                    angular_speed=self.turn_speed_moving
                ), 'FORWARD_LEFT_BIAS'

            if right_score > left_score + self.turn_margin:
                return self.build_arc_cmd(
                    linear_speed=self.forward_speed_slow,
                    angular_speed=-self.turn_speed_moving
                ), 'FORWARD_RIGHT_BIAS'

            return self.build_forward_cmd(self.forward_speed_slow), 'GO_FORWARD_SLOW'

        if center_score > self.front_turn_distance:
            if (
                left_score > right_score + self.turn_margin and
                left_score > self.side_safe_distance
            ):
                return self.build_arc_cmd(
                    linear_speed=self.forward_speed_turn,
                    angular_speed=self.turn_speed_moving
                ), 'FORWARD_LEFT'

            if (
                right_score > left_score + self.turn_margin and
                right_score > self.side_safe_distance
            ):
                return self.build_arc_cmd(
                    linear_speed=self.forward_speed_turn,
                    angular_speed=-self.turn_speed_moving
                ), 'FORWARD_RIGHT'

        # 4. Вперед нельзя, выбираем поворот на месте
        left_good = left_score > self.side_safe_distance
        right_good = right_score > self.side_safe_distance

        if left_good and not right_good:
            return self.build_turn_cmd(left=True), 'TURN_LEFT_IN_PLACE'

        if right_good and not left_good:
            return self.build_turn_cmd(left=False), 'TURN_RIGHT_IN_PLACE'

        if left_good and right_good:
            if left_score > right_score + self.turn_margin:
                return self.build_turn_cmd(left=True), 'TURN_LEFT_BETTER'
            if right_score > left_score + self.turn_margin:
                return self.build_turn_cmd(left=False), 'TURN_RIGHT_BETTER'
            return self.build_turn_cmd(left=self.prefer_left), 'TURN_DEFAULT'

        # 5. Все плохо
        return self.build_stop_cmd(), 'STOP_BLOCKED'

    def has_fresh_data(self) -> bool:
        now = self.get_clock().now()

        if self.last_free_time is None or self.last_distance_time is None:
            return False

        dt_free = (now - self.last_free_time).nanoseconds / 1e9
        dt_dist = (now - self.last_distance_time).nanoseconds / 1e9

        return dt_free <= self.perception_timeout and dt_dist <= self.perception_timeout

    def build_stop_cmd(self) -> Twist:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    def build_forward_cmd(self, speed: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        return cmd

    def build_turn_cmd(self, left: bool) -> Twist:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.turn_speed_in_place if left else -self.turn_speed_in_place
        return cmd

    def build_arc_cmd(self, linear_speed: float, angular_speed: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        return cmd

    def format_distance(self, value: Optional[float]) -> str:
        if value is None:
            return 'None'
        return f'{value:.2f}'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveNavNode()

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

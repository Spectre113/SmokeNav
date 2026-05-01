import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class FakeTargetPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('fake_target_publisher_node')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('scenario_period_sec', 3.0)
        self.declare_parameter('mode', 'cycle')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.scenario_period_sec = float(self.get_parameter('scenario_period_sec').value)
        self.mode = str(self.get_parameter('mode').value)

        self.target_pub = self.create_publisher(Float32MultiArray, '/target_info', 10)

        self.scenarios = [
            'target_center_far',
            'target_left_far',
            'target_right_far',
            'target_center_near',
            'target_left_near',
            'target_right_near',
            'target_lost',
        ]
        self.scenario_index = 0
        self.last_switch_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(f'Fake target publisher node started in mode: {self.mode}')

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_switch_time).nanoseconds / 1e9

        if self.mode == 'cycle' and dt >= self.scenario_period_sec:
            self.scenario_index = (self.scenario_index + 1) % len(self.scenarios)
            self.last_switch_time = now

        scenario = self.scenarios[self.scenario_index]
        msg = self.build_target_message(scenario)
        self.target_pub.publish(msg)

        self.get_logger().info(f'Published fake target scenario: {scenario} -> {list(msg.data)}')

    def build_target_message(self, scenario: str) -> Float32MultiArray:
        msg = Float32MultiArray()

        detected = 1.0
        angle = 0.0
        distance = 2.0
        confidence = 0.9

        if scenario == 'target_center_far':
            angle = 0.0
            distance = 2.5

        elif scenario == 'target_left_far':
            angle = 0.6
            distance = 2.5

        elif scenario == 'target_right_far':
            angle = -0.6
            distance = 2.5

        elif scenario == 'target_center_near':
            angle = 0.0
            distance = 0.8

        elif scenario == 'target_left_near':
            angle = 0.5
            distance = 0.8

        elif scenario == 'target_right_near':
            angle = -0.5
            distance = 0.8

        elif scenario == 'target_lost':
            detected = 0.0
            angle = 0.0
            distance = 0.0
            confidence = 0.0

        msg.data = [detected, angle, distance, confidence]
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeTargetPublisherNode()

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
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class FakeScanPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('fake_scan_publisher_node')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('scenario_period_sec', 3.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 3.5)
        self.declare_parameter('num_readings', 181)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.scenario_period_sec = float(self.get_parameter('scenario_period_sec').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.num_readings = int(self.get_parameter('num_readings').value)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.scenarios = [
            'all_free',
            'front_blocked',
            'front_left_narrow',
            'front_right_narrow',
            'left_blocked',
            'right_blocked',
            'all_blocked',
        ]
        self.scenario_index = 0
        self.last_switch_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('Fake scan publisher node started')

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_switch_time).nanoseconds / 1e9

        if dt >= self.scenario_period_sec:
            self.scenario_index = (self.scenario_index + 1) % len(self.scenarios)
            self.last_switch_time = now

        scenario = self.scenarios[self.scenario_index]
        msg = self.build_scan_message(scenario)
        self.scan_pub.publish(msg)

        self.get_logger().info(f'Published fake scan scenario: {scenario}')

    def build_scan_message(self, scenario: str) -> LaserScan:
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = -math.pi / 2.0
        msg.angle_max = math.pi / 2.0
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (self.num_readings - 1)

        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.publish_rate

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = [self.range_max] * self.num_readings

        # Сектора по индексам для 181 луча (-90..+90)
        center_start = 75
        center_end = 105

        right_start = 15
        right_end = 55

        left_start = 125
        left_end = 165

        # Доп. узкие зоны для несимметричных сценариев
        center_left_slice_start = 75
        center_left_slice_end = 90

        center_right_slice_start = 90
        center_right_slice_end = 105

        # Дистанции
        hard_block = 0.30
        narrow_block = 0.65
        semi_open = 1.00

        if scenario == 'front_blocked':
            for i in range(center_start, center_end):
                ranges[i] = hard_block

        elif scenario == 'left_blocked':
            for i in range(left_start, left_end):
                ranges[i] = hard_block

        elif scenario == 'right_blocked':
            for i in range(right_start, right_end):
                ranges[i] = hard_block

        elif scenario == 'all_blocked':
            ranges = [hard_block] * self.num_readings

        elif scenario == 'front_left_narrow':
            # Спереди проход есть, но слева уже и хуже, справа лучше
            for i in range(center_start, center_end):
                ranges[i] = semi_open
            for i in range(left_start, left_end):
                ranges[i] = narrow_block
            for i in range(center_left_slice_start, center_left_slice_end):
                ranges[i] = narrow_block

        elif scenario == 'front_right_narrow':
            # Спереди проход есть, но справа уже и хуже, слева лучше
            for i in range(center_start, center_end):
                ranges[i] = semi_open
            for i in range(right_start, right_end):
                ranges[i] = narrow_block
            for i in range(center_right_slice_start, center_right_slice_end):
                ranges[i] = narrow_block

        elif scenario == 'all_free':
            pass

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings

        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeScanPublisherNode()

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
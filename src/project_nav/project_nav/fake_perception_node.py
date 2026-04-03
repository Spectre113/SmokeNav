import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray


class FakePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('fake_perception_node')

        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('mode', 'cycle')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.mode = str(self.get_parameter('mode').value)

        self.pub = self.create_publisher(Int32MultiArray, '/free_sectors', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.scenarios = [
            [1, 1, 1],  # all free
            [1, 0, 0],  # left only
            [0, 0, 1],  # right only
            [1, 0, 1],  # left and right
            [0, 0, 0],  # blocked
        ]
        self.index = 0

        self.get_logger().info(f'Fake perception node started in mode: {self.mode}')

    def timer_callback(self) -> None:
        scenario = self.get_next_scenario()

        msg = Int32MultiArray()
        msg.data = scenario
        self.pub.publish(msg)

        self.get_logger().info(f'Published scenario: {scenario}')

    def get_next_scenario(self) -> list[int]:
        if self.mode == 'fixed':
            return [1, 1, 1]

        scenario = self.scenarios[self.index]
        self.index = (self.index + 1) % len(self.scenarios)
        return scenario


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
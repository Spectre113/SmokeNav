import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray
import tf2_ros

from .utils import transform_pose


class HumanPoseAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('human_pose_adapter_node')

        self.declare_parameter('hide_target_when_close', True)
        self.declare_parameter('hide_distance', 0.75)
        self.declare_parameter('reappear_distance', 1.0)

        self.latest_pose = None
        self.latest_detected = False
        self.latest_conf = 0.0
        self.target_hidden_close = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.hide_target_when_close = bool(self.get_parameter('hide_target_when_close').value)
        self.hide_distance = float(self.get_parameter('hide_distance').value)
        self.reappear_distance = float(self.get_parameter('reappear_distance').value)

        self.create_subscription(PoseStamped, '/human_pose', self.pose_cb, 10)
        self.create_subscription(Bool, '/human_localization/detected', self.detected_cb, 10)
        self.create_subscription(Float32, '/human_localization/confidence', self.conf_cb, 10)
        self.create_subscription(PoseStamped, '/human_localization/pose', self.loc_pose_cb, 10)

        self.humans_pub = self.create_publisher(PoseArray, '/humans', 10)
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_info', 10)

    def pose_cb(self, msg: PoseStamped) -> None:
        arr = PoseArray()
        arr.header = msg.header
        arr.poses = [msg.pose]
        self.humans_pub.publish(arr)

    def detected_cb(self, msg: Bool) -> None:
        self.latest_detected = bool(msg.data)
        self.publish_target()

    def conf_cb(self, msg: Float32) -> None:
        self.latest_conf = float(msg.data)
        self.publish_target()

    def loc_pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.publish_target()

    def publish_target(self) -> None:
        out = Float32MultiArray()
        detected = 0.0

        angle = 0.0
        distance = 0.0
        if self.latest_pose is not None:
            pose_in_base = transform_pose(self.tf_buffer, self.latest_pose, 'base_link')
            if pose_in_base is not None:
                x = float(pose_in_base.pose.position.x)
                y = float(pose_in_base.pose.position.y)
                distance = (x * x + y * y) ** 0.5
                angle = float(__import__('math').atan2(y, x))

                if self.hide_target_when_close:
                    if not self.target_hidden_close and distance <= self.hide_distance:
                        self.target_hidden_close = True
                        self.get_logger().info(
                            f'Hide target: close enough (dist={distance:.2f} <= {self.hide_distance:.2f})'
                        )
                    elif self.target_hidden_close and distance >= self.reappear_distance:
                        self.target_hidden_close = False
                        self.get_logger().info(
                            f'Restore target: moved away (dist={distance:.2f} >= {self.reappear_distance:.2f})'
                        )

                target_visible = self.latest_detected and (not self.target_hidden_close)
                detected = 1.0 if target_visible else 0.0

        out.data = [detected, angle, distance, self.latest_conf]
        self.target_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanPoseAdapterNode()
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

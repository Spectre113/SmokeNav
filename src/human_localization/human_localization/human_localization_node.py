import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Float32
import tf2_ros

from .tracker import Tracker
from .utils import transform_pose


class HumanLocalizationNode(Node):
    def __init__(self):
        super().__init__('human_localization')

        self.sub = self.create_subscription(
            PoseArray,
            '/humans',
            self.callback,
            10
        )

        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/human_localization/pose',
            10
        )
        self.pub_detected = self.create_publisher(
            Bool,
            '/human_localization/detected',
            10
        )
        self.pub_conf = self.create_publisher(
            Float32,
            '/human_localization/confidence',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tracker = Tracker()

        self.get_logger().info('Localization node started')

    def callback(self, msg: PoseArray):
        detections = []
        for pose in msg.poses:
            conf = float(pose.orientation.w) if pose.orientation.w > 0.0 else 0.3
            detections.append((pose.position.x, pose.position.y, conf))

        tracks = self.tracker.update(detections, msg.header.stamp)
        confirmed = [t for t in tracks if t.confirmed]

        if not confirmed:
            self.pub_detected.publish(Bool(data=False))
            return

        best = max(confirmed, key=lambda t: t.confidence)

        self.pub_detected.publish(Bool(data=True))
        self.pub_conf.publish(Float32(data=best.confidence))

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = msg.header.frame_id
        pose.pose.position.x = best.x
        pose.pose.position.y = best.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        pose_map = transform_pose(self.tf_buffer, pose, 'map')
        if pose_map is None:
            return

        self.pub_pose.publish(pose_map)


def main(args=None):
    rclpy.init(args=args)
    node = HumanLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

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

        self.declare_parameter('match_distance', 3.0)  # Mahalanobis gate, not meters.
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('process_noise', 1.0)
        self.declare_parameter('measurement_noise', 0.25)
        self.declare_parameter('initial_position_variance', 1.0)
        self.declare_parameter('initial_velocity_variance', 4.0)
        self.declare_parameter('tracking_frame', 'base_link')
        self.declare_parameter('target_frame', 'map')

        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.sub = self.create_subscription(PoseArray, '/humans', self.callback, 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/human_localization/pose', 10)
        self.pub_detected = self.create_publisher(Bool, '/human_localization/detected', 10)
        self.pub_conf = self.create_publisher(Float32, '/human_localization/confidence', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tracker = Tracker(
            match_distance=self.get_parameter('match_distance').value,
            min_hits=self.get_parameter('min_hits').value,
            max_misses=self.get_parameter('max_misses').value,
            process_noise=self.get_parameter('process_noise').value,
            measurement_noise=self.get_parameter('measurement_noise').value,
            initial_position_variance=self.get_parameter('initial_position_variance').value,
            initial_velocity_variance=self.get_parameter('initial_velocity_variance').value,
        )

        self.get_logger().info('Kalman human localization node started')

    def callback(self, msg: PoseArray):
        detections = [(p.position.x, p.position.y) for p in msg.poses]
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
        pose.header.frame_id = msg.header.frame_id or self.tracking_frame
        pose.pose.position.x = best.px
        pose.pose.position.y = best.py
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        pose_map = transform_pose(self.tf_buffer, pose, self.target_frame)
        if pose_map is None:
            return

        self.pub_pose.publish(pose_map)


def main(args=None):
    rclpy.init(args=args)
    node = HumanLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

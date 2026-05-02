import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose


def transform_pose(tf_buffer, pose_stamped, target_frame='map'):
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            pose_stamped.header.frame_id,
            rclpy.time.Time()
        )

        transformed_pose = PoseStamped()
        transformed_pose.header.stamp = pose_stamped.header.stamp
        transformed_pose.header.frame_id = target_frame
        transformed_pose.pose = do_transform_pose(pose_stamped.pose, transform)

        return transformed_pose

    except Exception as e:
        print(f"Transform failed: {e}")
        return None
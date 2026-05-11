from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detector = Node(
        package="project_detection",
        executable="gazebo_human_detector",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"model_name_prefix": "human_"},
            {"world_frame": "map"},
            {"publish_markers": True},
        ],
    )

    target_marker = Node(
        package="project_detection",
        executable="target_gazebo_marker",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"target_pose_topic": "/human_localization/pose"},
            {"target_info_topic": "/target_info"},
            {"marker_name": "goal_target_marker"},
            {"marker_height": 0.12},
            {"marker_radius": 0.16},
            {"consume_on_reach": True},
            {"consume_distance": 0.75},
            {"target_entity_name": "human_0"},
        ],
    )

    return LaunchDescription([detector, target_marker])

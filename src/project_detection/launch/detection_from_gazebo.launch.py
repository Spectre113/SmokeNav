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

    return LaunchDescription([detector])


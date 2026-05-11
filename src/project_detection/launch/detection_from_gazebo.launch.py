from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    human_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('human_detector'), 'launch', 'human_detection_launch.launch.py']
            )
        ),
        launch_arguments={
            'require_heartbeat': 'true',
            'heartbeat_topic': '/human_heartbeat',
        }.items(),
    )

    heartbeat = Node(
        package="project_detection",
        executable="sim_human_heartbeat",
        name="sim_human_heartbeat",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"model_name_prefix": "human_"},
            {"heartbeat_topic": "/human_heartbeat"},
            {"base_frame": "base_link"},
            {"world_frame": "map"},
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

    return LaunchDescription([heartbeat, human_detection, target_marker])

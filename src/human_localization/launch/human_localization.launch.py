from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='human_localization',
            executable='human_localization',
            name='human_localization',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('human_localization'),
        'config',
        'human_localization.yaml',
    )

    return LaunchDescription([
        Node(
            package='human_localization',
            executable='human_localization',
            name='human_localization',
            output='screen',
            parameters=[config],
        )
    ])

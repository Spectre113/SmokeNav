from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    reactive_nav = Node(
        package='project_nav',
        executable='reactive_nav_node',
        name='reactive_nav_node',
        output='screen',
        parameters=[{
            'forward_speed': 0.2,
            'turn_speed': 0.6,
            'control_rate': 10.0,
            'perception_timeout': 1.0,
            'prefer_left': True,
        }]
    )

    fake_perception = Node(
        package='project_nav',
        executable='fake_perception_node',
        name='fake_perception_node',
        output='screen',
        parameters=[{
            'publish_rate': 5.0,
            'mode': 'cycle',
        }]
    )

    return LaunchDescription([
        fake_perception,
        reactive_nav,
    ])
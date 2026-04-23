from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic to use for navigation.",
    )

    sector_analyzer = Node(
        package='project_nav',
        executable='sector_analyzer_node',
        name='sector_analyzer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': LaunchConfiguration("scan_topic"),
            'output_topic': '/free_sectors',
            'front_half_angle_deg': 20.0,
            'side_outer_angle_deg': 90.0,
            'front_safe_distance': 0.8,
            'side_safe_distance': 0.6,
            'use_inf_as_free': True,
            'publish_rate': 10.0,
        }]
    )

    reactive_nav = Node(
        package='project_nav',
        executable='reactive_nav_node',
        name='reactive_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'forward_speed': 0.2,
            'turn_speed': 0.6,
            'control_rate': 10.0,
            'perception_timeout': 1.0,
            'prefer_left': True,
        }]
    )

    return LaunchDescription([
        scan_topic_arg,
        sector_analyzer,
        reactive_nav,
    ])
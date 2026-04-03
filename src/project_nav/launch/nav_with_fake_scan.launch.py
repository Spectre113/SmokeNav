from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fake_scan = Node(
        package='project_nav',
        executable='fake_scan_publisher_node',
        name='fake_scan_publisher_node',
        output='screen',
        parameters=[{
            'publish_rate': 5.0,
            'scenario_period_sec': 3.0,
            'range_min': 0.12,
            'range_max': 3.5,
            'num_readings': 181,
        }]
    )

    sector_analyzer = Node(
        package='project_nav',
        executable='sector_analyzer_node',
        name='sector_analyzer_node',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
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
            'forward_speed_fast': 0.25,
            'forward_speed_slow': 0.12,
            'forward_speed_turn': 0.08,
            'turn_speed_in_place': 0.60,
            'turn_speed_moving': 0.35,
            'control_rate': 10.0,
            'perception_timeout': 1.0,
            'front_safe_distance': 0.8,
            'front_clear_distance': 1.5,
            'front_turn_distance': 0.5,
            'side_safe_distance': 0.6,
            'turn_margin': 0.15,
            'prefer_left': True,
            'free_topic': '/free_sectors',
            'distance_topic': '/sector_distances',
            'cmd_topic': '/cmd_vel',
        }]
    )

    return LaunchDescription([
        fake_scan,
        sector_analyzer,
        reactive_nav,
    ])
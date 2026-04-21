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
            'distance_topic': '/sector_distances',
            'front_half_angle_deg': 20.0,
            'side_outer_angle_deg': 90.0,
            'front_safe_distance': 0.8,
            'side_safe_distance': 0.6,
            'use_inf_as_free': True,
            'publish_rate': 10.0,
        }]
    )

    fake_target = Node(
        package='project_nav',
        executable='fake_target_publisher_node',
        name='fake_target_publisher_node',
        output='screen',
        parameters=[{
            'publish_rate': 5.0,
            'scenario_period_sec': 3.0,
            'mode': 'cycle',
        }]
    )

    goal_nav = Node(
        package='project_nav',
        executable='goal_aware_nav_node',
        name='goal_aware_nav_node',
        output='screen',
        parameters=[{
            'control_rate': 10.0,
            'perception_timeout': 1.0,
            'max_linear_speed': 0.25,
            'min_linear_speed': 0.08,
            'max_angular_speed': 0.6,
            'front_safe_distance': 0.8,
            'front_blocked_distance': 0.45,
            'side_safe_distance': 0.6,
            'goal_angle_gain': 1.0,
            'goal_distance_gain': 0.2,
            'avoid_turn_gain': 0.8,
            'target_stop_distance': 0.7,
            'target_confidence_threshold': 0.4,
            'commit_time_sec': 1.2,
            'commit_side_margin': 0.2,
            'front_clear_cycles_required': 3,
            'obs_weight': 1.0,
            'target_weight': 0.8,
            'commit_weight': 0.4,
            'side_score_cap': 3.5,
            'prefer_left': True,
            'free_topic': '/free_sectors',
            'distance_topic': '/sector_distances',
            'target_topic': '/target_info',
            'cmd_topic': '/cmd_vel',
        }]
    )

    return LaunchDescription([
        fake_scan,
        sector_analyzer,
        fake_target,
        goal_nav,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic to use for navigation.",
    )
    radar_topic_arg = DeclareLaunchArgument(
        "radar_topic",
        default_value="/radar/points",
        description="PointCloud2 radar topic to use for navigation if available.",
    )
    depth_points_topic_arg = DeclareLaunchArgument(
        "depth_points_topic",
        default_value="/camera/depth/color/points",
        description="RGB-D PointCloud2 topic to use for navigation if available.",
    )
    ultrasonic_topic_arg = DeclareLaunchArgument(
        "ultrasonic_topic",
        default_value="/ultrasonic/front",
        description="Range topic to use as a short-range front safety sensor.",
    )
    require_target_arg = DeclareLaunchArgument(
        "require_target",
        default_value="false",
        description="If true, stop when no target is available.",
    )

    sector_analyzer = Node(
        package='project_nav',
        executable='sector_analyzer_node',
        name='sector_analyzer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': LaunchConfiguration("scan_topic"),
            'radar_topic': LaunchConfiguration("radar_topic"),
            'depth_points_topic': LaunchConfiguration("depth_points_topic"),
            'ultrasonic_topic': LaunchConfiguration("ultrasonic_topic"),
            'output_topic': '/free_sectors',
            'distance_topic': '/sector_distances',
            'enable_lidar': True,
            'enable_radar': True,
            'enable_depth_camera': True,
            'enable_ultrasonic': True,
            'front_half_angle_deg': 20.0,
            'side_outer_angle_deg': 90.0,
            'front_safe_distance': 0.8,
            'side_safe_distance': 0.6,
            'use_inf_as_free': True,
            'publish_rate': 10.0,
            'sensor_timeout': 1.0,
        }]
    )

    goal_nav = Node(
        package='project_nav',
        executable='goal_aware_nav_node',
        name='goal_aware_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_rate': 10.0,
            'perception_timeout': 1.0,
            'require_target': ParameterValue(
                LaunchConfiguration("require_target"),
                value_type=bool,
            ),
            'use_target_memory': True,
            'target_memory_timeout': 2.0,
            'target_hint_stop_distance': 0.8,
            'search_linear_speed': 0.12,
            'passage_mode_enabled': True,
            'passage_front_clear_distance': 0.75,
            'passage_min_side_distance': 0.18,
            'passage_danger_alpha_cap': 0.35,
            'passage_linear_speed': 0.10,
            'prefer_left': True,
        }]
    )

    return LaunchDescription([
        scan_topic_arg,
        radar_topic_arg,
        depth_points_topic_arg,
        ultrasonic_topic_arg,
        require_target_arg,
        sector_analyzer,
        goal_nav,
    ])

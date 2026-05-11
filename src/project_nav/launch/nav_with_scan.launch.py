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
            'smoke_affects_radar': False,
            'smoke_affects_depth': True,
            'smoke_affects_ultrasonic': False,
            'base_frame': 'base_link',
            'enable_tf_transform': True,
            'allow_tf_fallback': False,
            'front_half_angle_deg': 20.0,
            'side_outer_angle_deg': 90.0,
            'num_detailed_sectors': 9,
            'front_safe_distance': 0.65,
            'side_safe_distance': 0.25,
            'detailed_safe_distance': 0.35,
            'fusion_percentile': 35.0,
            'source_percentile': 25.0,
            'radar_min_support': 2,
            'depth_min_support': 4,
            'use_inf_as_free': True,
            'publish_rate': 10.0,
            'sensor_timeout': 1.0,
            'publish_costmap': True,
            'costmap_resolution': 0.05,
            'costmap_width_m': 5.0,
            'costmap_height_m': 5.0,
            'costmap_origin_x': -0.8,
            'costmap_origin_y': -2.5,
            'costmap_inflation_radius': 0.16,
            'publish_global_map': True,
            'global_frame': 'map',
            'global_map_topic': '/global_map',
            'global_path_topic': '/exploration_path',
            'exploration_hint_topic': '/exploration_hint',
            'global_map_resolution': 0.10,
            'global_map_width_m': 30.0,
            'global_map_height_m': 30.0,
            'global_map_origin_x': -15.0,
            'global_map_origin_y': -15.0,
            'max_free_ray_range': 4.5,
            'frontier_min_distance': 0.9,
            'frontier_max_distance': 12.0,
            'frontier_max_abs_angle_deg': 170.0,
            'frontier_min_cluster_size': 4,
            'frontier_cluster_weight': 0.18,
            'frontier_cluster_score_cap': 4.0,
            'frontier_distance_weight': 0.90,
            'frontier_heading_weight': 0.35,
            'frontier_current_bonus': 3.0,
            'frontier_reached_distance': 0.7,
            'frontier_keep_radius': 1.2,
            'exploration_path_lookahead_m': 0.85,
            'frontier_min_path_distance': 1.2,
            'frontier_max_path_distance': 18.0,
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
            'target_reached_hold_sec': 0.8,
            'target_reacquire_distance': 1.1,
            'clear_target_after_reached': True,
            'search_linear_speed': 0.12,
            'front_safe_distance': 0.65,
            'front_blocked_distance': 0.40,
            'side_safe_distance': 0.35,
            'wall_caution_distance': 0.55,
            'wall_critical_distance': 0.32,
            'wall_stop_distance': 0.22,
            'passage_mode_enabled': True,
            'passage_front_clear_distance': 0.60,
            'passage_min_side_distance': 0.20,
            'passage_danger_alpha_cap': 0.25,
            'passage_linear_speed': 0.14,
            'passage_target_angular_weight': 0.45,
            'avoid_target_angular_weight': 0.30,
            'detailed_distance_topic': '/sector_distances_detailed',
            'exploration_hint_topic': '/exploration_hint',
            'use_global_exploration': True,
            'exploration_hint_timeout': 2.0,
            'sensor_metrics_topic': '/sensor_fusion_metrics',
            'nav_metrics_topic': '/navigation_metrics',
            'smoke_density_topic': '/smoke/density',
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

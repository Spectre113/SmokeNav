from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    density_arg = DeclareLaunchArgument(
        'density',
        default_value='0.0',
        description='Smoke density in [0..1].',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('project_sim'), 'launch', 'sim_bringup.launch.py'])
        )
    )

    smoke = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('project_smoke'), 'launch', 'smoke_filter.launch.py'])
        ),
        launch_arguments={'density': LaunchConfiguration('density')}.items(),
    )

    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('project_detection'), 'launch', 'detection_from_gazebo.launch.py'])
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('human_localization'), 'launch', 'human_localization.launch.py'])
        )
    )

    adapter = Node(
        package='human_localization',
        executable='human_pose_adapter',
        name='human_pose_adapter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_humans_from_pose': False,
        }],
    )

    sector_analyzer = Node(
        package='project_nav',
        executable='sector_analyzer_node',
        name='sector_analyzer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan_smoked',
            'radar_topic': '/radar/points',
            'depth_points_topic': '/camera/depth/color/points',
            'ultrasonic_topic': '/ultrasonic/front',
            'enable_lidar': True,
            'enable_radar': True,
            'enable_depth_camera': True,
            'enable_ultrasonic': True,
            'base_frame': 'base_link',
            'enable_tf_transform': True,
            'allow_tf_fallback': False,
            'num_detailed_sectors': 9,
            'front_safe_distance': 0.65,
            'side_safe_distance': 0.25,
            'detailed_safe_distance': 0.35,
            'fusion_percentile': 35.0,
            'source_percentile': 25.0,
            'radar_min_support': 2,
            'depth_min_support': 4,
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
            'sensor_timeout': 1.0,
        }],
    )

    goal_nav = Node(
        package='project_nav',
        executable='goal_aware_nav_node',
        name='goal_aware_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'require_target': False,
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
            'detailed_distance_topic': '/sector_distances_detailed',
            'exploration_hint_topic': '/exploration_hint',
            'use_global_exploration': True,
            'exploration_hint_timeout': 2.0,
            'sensor_metrics_topic': '/sensor_fusion_metrics',
            'nav_metrics_topic': '/navigation_metrics',
            'smoke_density_topic': '/smoke/density',
        }],
    )

    return LaunchDescription([
        density_arg,
        sim,
        smoke,
        detection,
        localization,
        adapter,
        sector_analyzer,
        goal_nav,
    ])

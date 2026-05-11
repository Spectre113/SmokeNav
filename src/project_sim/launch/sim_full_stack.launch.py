from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('project_sim'), 'launch', 'sim_bringup.launch.py'])
        )
    )

    smoke = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('project_smoke'), 'launch', 'smoke_filter.launch.py'])
        )
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
            'sensor_metrics_topic': '/sensor_fusion_metrics',
            'nav_metrics_topic': '/navigation_metrics',
            'smoke_density_topic': '/smoke/density',
        }],
    )

    return LaunchDescription([sim, smoke, detection, localization, adapter, sector_analyzer, goal_nav])

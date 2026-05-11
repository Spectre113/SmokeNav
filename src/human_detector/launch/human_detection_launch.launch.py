from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'require_heartbeat',
            default_value='false',
            description='If true, fusion publishes humans only near a fresh biosignal.',
        ),
        DeclareLaunchArgument(
            'heartbeat_topic',
            default_value='/human_heartbeat',
            description='Simulation biosignal topic used to reject non-human objects.',
        ),

        # Node 1: Thermal detection
        Node(
            package='human_detector',
            executable='thermal_detection_node',
            name='thermal_detection_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Node 2: Radar detection
        Node(
            package='human_detector',
            executable='radar_detection_node',
            name='radar_detection_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # Node 3: Fusion
        Node(
            package='human_detector',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',
                'radar_frame': 'radar_link',
                'radar_cluster_topic': '/radar/human_clusters',
                'thermal_topic': '/thermal/human_positions',
                'heartbeat_topic': LaunchConfiguration('heartbeat_topic'),
                'require_heartbeat': ParameterValue(
                    LaunchConfiguration('require_heartbeat'),
                    value_type=bool,
                ),
                'heartbeat_match_distance': 0.90,
                'heartbeat_min_score': 0.08,
                'heartbeat_weight': 0.55,
                'use_heartbeat_fallback': ParameterValue(
                    LaunchConfiguration('require_heartbeat'),
                    value_type=bool,
                ),
                'heartbeat_fallback_confidence': 0.85,
                'depth_points_topic': '/camera/depth/color/points',
                'smoke_density_topic': '/smoke/density',
                'humans_topic': '/humans',
                'min_publish_confidence': 0.40,
            }],
        ),
    ])

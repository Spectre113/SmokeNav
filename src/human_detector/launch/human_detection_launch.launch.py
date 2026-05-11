from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            parameters=[{'use_sim_time': True}],
        ),
    ])
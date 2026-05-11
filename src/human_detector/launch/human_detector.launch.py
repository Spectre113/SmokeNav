from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    radar_node = Node(
        package='human_detector',
        executable='radar_detection_node',
        name='radar_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/radar/pointcloud', '/radar/points')],
    )

    thermal_node = Node(
        package='human_detector',
        executable='thermal_detection_node',
        name='thermal_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    fusion_node = Node(
        package='human_detector',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([radar_node, thermal_node, fusion_node])
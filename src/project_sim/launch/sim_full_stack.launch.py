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
        parameters=[{'use_sim_time': True, 'scan_topic': '/scan_smoked'}],
    )

    goal_nav = Node(
        package='project_nav',
        executable='goal_aware_nav_node',
        name='goal_aware_nav_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([sim, smoke, detection, localization, adapter, sector_analyzer, goal_nav])

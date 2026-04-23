from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("project_sim"),
                    "launch",
                    "sim_with_nav.launch.py",
                ]
            )
        )
    )

    detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("project_detection"),
                    "launch",
                    "detection_from_gazebo.launch.py",
                ]
            )
        )
    )

    return LaunchDescription([sim_and_nav, detection])


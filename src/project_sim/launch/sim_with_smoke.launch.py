from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("project_sim"), "launch", "sim_bringup.launch.py"]
            )
        )
    )

    smoke = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("project_smoke"), "launch", "smoke_filter.launch.py"]
            )
        )
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("project_nav"), "launch", "nav_with_scan.launch.py"]
            )
        ),
        launch_arguments={"scan_topic": "/scan_smoked"}.items(),
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

    return LaunchDescription([sim, smoke, nav, detection])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    density_arg = DeclareLaunchArgument(
        "density",
        default_value="0.0",
        description="Smoke density in [0..1] forwarded to the smoke filter.",
    )

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
        ),
        launch_arguments={"density": LaunchConfiguration("density")}.items(),
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

    return LaunchDescription([density_arg, sim, smoke, nav, detection])


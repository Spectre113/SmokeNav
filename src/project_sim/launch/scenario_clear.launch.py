from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("project_sim"), "launch", "sim_with_smoke.launch.py"]
            )
        ),
        launch_arguments={"density": "0.0"}.items(),
    )

    metrics = Node(
        package="project_eval",
        executable="metrics_logger",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"output_csv": str(Path("logs/metrics_clear.csv").resolve())},
        ],
    )

    return LaunchDescription([sim, metrics])


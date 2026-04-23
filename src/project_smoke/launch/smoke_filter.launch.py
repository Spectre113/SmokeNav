from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    density_arg = DeclareLaunchArgument(
        "density",
        default_value="0.0",
        description="Smoke density in [0..1].",
    )

    smoke = Node(
        package="project_smoke",
        executable="scan_smoke_filter",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"density": LaunchConfiguration("density")},
            {"input_topic": "/scan"},
            {"output_topic": "/scan_smoked"},
        ],
    )

    return LaunchDescription([density_arg, smoke])


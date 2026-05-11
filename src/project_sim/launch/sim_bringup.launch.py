from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [FindPackageShare("project_sim"), "worlds", "custom-flat.world"]
    )
    default_xacro = PathJoinSubstitution(
        [FindPackageShare("project_sim"), "urdf", "robot.urdf.xacro"]
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Path to the Gazebo world file (.world).",
    )
    xacro_arg = DeclareLaunchArgument(
        "xacro_file",
        default_value=default_xacro,
        description="Path to the robot URDF Xacro file.",
    )
    enable_rendering_sensors_arg = DeclareLaunchArgument(
        "enable_rendering_sensors",
        default_value="false",
        description="Enable Gazebo rendering sensors such as depth and thermal cameras.",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="smokenav_robot",
        description="Gazebo model name for the robot.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true.",
    )
    enable_human_breathing_arg = DeclareLaunchArgument(
        "enable_human_breathing",
        default_value="true",
        description="Enable micro-oscillation for human_0 so mmWave can detect breathing motion.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": Command(
                    [
                        FindExecutable(name="xacro"),
                        " ",
                        LaunchConfiguration("xacro_file"),
                        " ",
                        "enable_rendering_sensors:=",
                        LaunchConfiguration("enable_rendering_sensors"),
                    ]
                )
            },
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            LaunchConfiguration("robot_name"),
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    simulated_mmwave_radar = Node(
        package="project_sim",
        executable="simulated_mmwave_radar_node",
        name="simulated_mmwave_radar_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_model_name": LaunchConfiguration("robot_name")},
        ],
    )

    human_breathing = Node(
        package="project_sim",
        executable="human_breathing_node",
        name="human_breathing_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_human_breathing")),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            xacro_arg,
            enable_rendering_sensors_arg,
            robot_name_arg,
            use_sim_time_arg,
            enable_human_breathing_arg,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            map_to_odom,
            simulated_mmwave_radar,
            human_breathing,
        ]
    )

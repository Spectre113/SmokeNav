from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    # If ~/.ros/log is not writable (common after sudo runs), gzserver can crash.
    # Use a workspace-local log dir by default so Gazebo + gazebo_ros always start.
    workspace_log_dir = str((Path.cwd() / ".roslog").resolve())
    workspace_gazebo_log_dir = str((Path.cwd() / ".gazebo_log").resolve())

    default_world = PathJoinSubstitution(
        [FindPackageShare("project_sim"), "worlds", "custom-flat.world"]
    )
    default_xacro = PathJoinSubstitution(
        [FindPackageShare("project_sim"), "urdf", "robot.urdf.xacro"]
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo client (gzclient) if true.",
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": Command(
                    [FindExecutable(name="xacro"), " ", LaunchConfiguration("xacro_file")]
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

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_LOG_DIR", value=workspace_log_dir),
            SetEnvironmentVariable(name="GAZEBO_LOG_PATH", value=workspace_gazebo_log_dir),
            ExecuteProcess(cmd=["mkdir", "-p", workspace_log_dir, workspace_gazebo_log_dir]),
            gui_arg,
            world_arg,
            xacro_arg,
            robot_name_arg,
            use_sim_time_arg,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            map_to_odom,
        ]
    )


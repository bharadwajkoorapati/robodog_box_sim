import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dof_two")

    urdf_file = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            "pkg_2dof.urdf",
        ]
    )

    world_file = PathJoinSubstitution(
        [
            pkg_share,
            "worlds",
            "empty_us_world.sdf",
        ]
    )

    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    gazebo_launch_file = PathJoinSubstitution(
        [
            ros_gz_sim_share,
            "launch",
            "gz_sim.launch.py",
        ]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # FindPackageShare cannot be passed to os.path.dirname().
    # Resolve the package share path using a normal Python string.
    from ament_index_python.packages import get_package_share_directory

    package_share_directory = get_package_share_directory("dof_two")
    install_parent_dir = os.path.dirname(package_share_directory)

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            install_parent_dir,
            ":",
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ],
    )

    robot_description = {
        "robot_description": Command(
            [
                "cat ",
                urdf_file,
            ]
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": ["-r ", world_file],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dof_two",
        output="screen",
        arguments=[
            "-name",
            "dof_two",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.05",
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start joint_state_publisher_gui.",
            ),
            set_gz_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            joint_state_publisher_gui,
        ]
    )
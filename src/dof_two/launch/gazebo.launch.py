import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    controllers_file = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "controllers.yaml",
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
    gz_partition = LaunchConfiguration("gz_partition")

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

    set_gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value=gz_partition,
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    urdf_file,
                    " controller_config:=",
                    controllers_file,
                ]
            ),
            value_type=str,
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
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    keyboard_and_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32",
        ],
        name="dof_two_gz_bridge",
        output="screen",
    )

    keyboard_joint_controller = Node(
        package="dof_two",
        executable="keyboard_joint_controller",
        name="keyboard_joint_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
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
            DeclareLaunchArgument(
                "gz_partition",
                default_value="dof_two",
                description="Gazebo Transport partition for this simulation.",
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner],
                )
            ),
            set_gz_partition,
            set_gz_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            keyboard_and_clock_bridge,
            keyboard_joint_controller,
            joint_state_publisher_gui,
        ]
    )
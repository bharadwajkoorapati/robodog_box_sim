import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ultrasonic_box_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'ultrasonic_box_bot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_us_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_bot_rviz_config.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ultrasonic_box_bot', '-z', '0.05'],
        output='screen'
    )

    # NOTE: the world name below MUST match <world name="..."> in empty_us.sdf
    # and the model name MUST match the -name given to `create` above.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/us_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty_us_world/model/ultrasonic_box_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty_us_world/model/ultrasonic_box_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        rviz2,
    ])

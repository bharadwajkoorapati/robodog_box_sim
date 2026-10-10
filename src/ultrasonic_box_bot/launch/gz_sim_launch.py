import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# --- GPU Rendering (NVIDIA Offload) ---
set_nv_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
set_glx_vendor = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
set_no_sw     = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0')


def generate_launch_description():
    pkg_share     = get_package_share_directory('ultrasonic_box_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    xacro_file       = os.path.join(pkg_share, 'urdf',   'ultrasonic_box_bot.urdf.xacro')
    world_file       = os.path.join(pkg_share, 'worlds', 'empty_us_world.sdf')
    rviz_config      = os.path.join(pkg_share, 'rviz',   'view_bot_rviz_config.rviz')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # -----------------------------------------------------------------------
    # 1. Robot State Publisher
    # -----------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # -----------------------------------------------------------------------
    # 2. Gazebo Harmonic Simulation
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # -----------------------------------------------------------------------
    # 3. Spawn Robot into Gazebo
    # -----------------------------------------------------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name',  'ultrasonic_box_bot',
                   '-z',     '0.01'],
        output='screen'
    )

    # -----------------------------------------------------------------------
    # 4. ROS-GZ Bridge
    # -----------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/us_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/us_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
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

    # -----------------------------------------------------------------------
    # 5. Target Cube Publisher (ground-truth static pose)
    # -----------------------------------------------------------------------
    target_cube_publisher = Node(
        package='ultrasonic_box_bot',
        executable='target_cube_publisher',
        name='target_cube_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -----------------------------------------------------------------------
    # 6. SLAM Toolbox – Online Async Mapping
    #    Publishes: /map (nav_msgs/msg/OccupancyGrid)
    #               /map_metadata
    #               TF: map -> odom (indirectly through slam)
    # -----------------------------------------------------------------------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True},
        ],
    )

    # -----------------------------------------------------------------------
    # 7. Nav2 Bringup – Delayed 5s to let SLAM initialize first
    #    Provides: AMCL, costmaps, planner, controller, bt_navigator
    # -----------------------------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file':  nav2_params_file,
            'autostart':    'true',
        }.items()
    )

    nav2_bringup_delayed = TimerAction(
        period=5.0,
        actions=[nav2_bringup]
    )

    # -----------------------------------------------------------------------
    # 8. RViz2
    # -----------------------------------------------------------------------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # -----------------------------------------------------------------------
    # Launch Ordering
    # -----------------------------------------------------------------------
    return LaunchDescription([
        set_nv_offload,
        set_glx_vendor,
        set_no_sw,
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        target_cube_publisher,
        slam_toolbox,
        nav2_bringup_delayed,
        rviz2,
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition

# --- GPU Rendering (NVIDIA Offload) ---
set_nv_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
set_glx_vendor = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
set_no_sw = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0')


def _is_true(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _validate_args(context, localization, use_autonav, map_file):
    """Fail fast on contradictory or incomplete launch arguments.

    Combination errors are much cheaper to catch here than as a half-started
    simulation several seconds later.
    """
    localized = _is_true(context.perform_substitution(localization))
    autonav = _is_true(context.perform_substitution(use_autonav))

    if localized and autonav:
        raise RuntimeError(
            "localization:=true and use_autonav:=true are mutually exclusive: "
            "both modes expect to drive /cmd_vel and the localizing mode needs "
            "Nav2. Run one at a time."
        )

    if localized:
        path = context.perform_substitution(map_file)
        if not path:
            raise RuntimeError(
                "localization:=true requires map:=<path to map yaml>. "
                "Create one first with: ros2 launch ultrasonic_box_bot save_map_launch.py"
            )
        if not os.path.isfile(path):
            raise RuntimeError(
                f"map:='{path}' does not exist. Save a map while SLAM is running, "
                "then pass the resulting .yaml file."
            )

    return []


def _nav2_navigation(pkg_nav2_bringup, nav2_params_file, use_autonav):
    """Nav2 navigation servers (planner/controller/BT), never the SLAM owner."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
        condition=UnlessCondition(use_autonav),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('ultrasonic_box_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_autonav = LaunchConfiguration('use_autonav')
    localization = LaunchConfiguration('localization')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2_goal_client = LaunchConfiguration('use_nav2_goal_client')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')

    declare_use_autonav = DeclareLaunchArgument(
        'use_autonav',
        default_value='false',
        description='Run the scripted wall-following FSM instead of Nav2. '
                    'Only one node may own /cmd_vel.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Default false: always build the map live with SLAM and '
                    'navigate on it (map -> odom is produced by SLAM). '
                    'localization:=true with map:=<file.yaml> is only for loading '
                    'a map saved explicitly with save_map_launch.py; this launch '
                    'never writes a map by itself.',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Only used by the optional localization:=true path.',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 (disable for headless runs).',
    )
    declare_goal_client = DeclareLaunchArgument(
        'use_nav2_goal_client',
        default_value='true',
        description='Forward /target_cube/pose to the Nav2 NavigateToPose action.',
    )
    declare_cmd_vel_timeout = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.5',
        description='Seconds of command silence before the watchdog commands a stop.',
    )
    declare_target_x = DeclareLaunchArgument(
        'target_x',
        default_value='4.0',
        description='Target x (m) published on /target_cube/pose; the goal client '
                    'drives there. Override to steer the mission from the command line.',
    )
    declare_target_y = DeclareLaunchArgument(
        'target_y',
        default_value='-4.0',
        description='Target y (m) published on /target_cube/pose.',
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'ultrasonic_box_bot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'slam_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_bot_rviz_config.rviz')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    validate_args = OpaqueFunction(
        function=_validate_args,
        args=[localization, use_autonav, map_file],
    )

    # -----------------------------------------------------------------------
    # 1. Robot State Publisher  (t = 0s)
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
    # 2. Gazebo Harmonic Simulation  (t = 0s)
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # -----------------------------------------------------------------------
    # 3. Spawn Robot into Gazebo  (t = 0s, ros_gz_sim create blocks until done)
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
    # 4. ROS-GZ Bridge  (t = 0s)
    #
    # /cmd_vel_safe (not /cmd_vel) is what reaches the DiffDrive plugin: the
    # watchdog forwards every command and injects a zero Twist when the stream
    # stops, because DiffDrive otherwise keeps applying the last command.
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
            '/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/slam_world/model/ultrasonic_box_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/slam_world/model/ultrasonic_box_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # -----------------------------------------------------------------------
    # 5. Command watchdog: /cmd_vel -> /cmd_vel_safe  (t = 0s)
    # -----------------------------------------------------------------------
    cmd_vel_watchdog = Node(
        package='ultrasonic_box_bot',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_safe',
            'timeout_sec': ParameterValue(cmd_vel_timeout, value_type=float),
            'zero_publish_rate_hz': 10.0,
        }],
    )

    # -----------------------------------------------------------------------
    # 6. Target Cube Publisher  (t = 0s)
    # -----------------------------------------------------------------------
    target_cube_publisher = Node(
        package='ultrasonic_box_bot',
        executable='target_cube_publisher',
        name='target_cube_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_x': ParameterValue(target_x, value_type=float),
            'target_y': ParameterValue(target_y, value_type=float),
        }]
    )

    # -----------------------------------------------------------------------
    # 6b. Map padder: /map -> /map_padded
    #
    # The global costmap's StaticLayer sizes the costmap from the map it
    # receives.  Feeding it raw SLAM output means the costmap grows (and is
    # reallocated) every time SLAM maps another room, and a goal in an
    # unexplored area is rejected with "outside bounds".  map_padder republishes
    # the map inside a fixed 12 x 12 m arena window, so bounds are stable and
    # goals anywhere in the arena are accepted while still being planned
    # against the live map.
    # -----------------------------------------------------------------------
    map_padder = Node(
        package='ultrasonic_box_bot',
        executable='map_padder',
        name='map_padder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/map',
            'output_topic': '/map_padded',
            'min_x': -6.0,
            'max_x': 6.0,
            'min_y': -6.0,
            'max_y': 6.0,
        }],
    )

    # -----------------------------------------------------------------------
    # 7. Readiness gate
    #    No node may consume /scan, /odom or TF before the bridge carries real
    #    data.  The gate exits as soon as those are live, so SLAM (or AMCL) is
    #    configured from an event instead of a guessed sleep.
    # -----------------------------------------------------------------------
    sensor_ready = Node(
        package='ultrasonic_box_bot',
        executable='sensor_ready',
        name='sensor_ready',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'timeout_sec': 90.0,
            'scan_topic': '/scan',
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )

    # -----------------------------------------------------------------------
    # 8. SLAM Toolbox lifecycle node (mapping mode only)
    # -----------------------------------------------------------------------
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True, 'use_lifecycle_manager': False},
        ],
        condition=UnlessCondition(localization),
    )

    # -----------------------------------------------------------------------
    # 9. Saved-map localization (localization mode only)
    #    map_server publishes the saved map with transient-local durability and
    #    AMCL becomes the single owner of map -> odom.  Never run SLAM and AMCL
    #    at the same time: two publishers of map -> odom fight silently.
    # -----------------------------------------------------------------------
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
        condition=IfCondition(localization),
    )

    # Bounded wait for /map + map -> base_footprint before Nav2's costmaps come
    # up.  AMCL publishes map -> odom as soon as it has an initial pose (the
    # parameters provide one for the default spawn).
    navigation_ready = Node(
        package='ultrasonic_box_bot',
        executable='navigation_ready',
        name='navigation_ready',
        output='screen',
        condition=IfCondition(localization),
        parameters=[{'use_sim_time': True, 'timeout_sec': 120.0}],
    )

    start_slam_or_localization = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_ready,
            on_exit=[
                LogInfo(
                    msg='[LAUNCH] Sensors ready -> configuring SLAM Toolbox',
                    condition=UnlessCondition(localization),
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                    condition=UnlessCondition(localization),
                ),
                LogInfo(
                    msg='[LAUNCH] Sensors ready -> starting saved-map localization',
                    condition=IfCondition(localization),
                ),
                localization_bringup,
                navigation_ready,
            ],
        )
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LAUNCH] Activating SLAM Toolbox...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # -----------------------------------------------------------------------
    # 10. Nav2 – mapping mode: start once SLAM is active (map -> odom exists);
    #     localization mode: start once the saved map and map -> odom are live.
    # -----------------------------------------------------------------------
    start_nav2_when_slam_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg='[LAUNCH] SLAM active -> starting Nav2 in 3s'),
                TimerAction(
                    period=3.0,
                    actions=[
                        _nav2_navigation(pkg_nav2_bringup, nav2_params_file, use_autonav)
                    ],
                ),
            ],
        )
    )

    start_nav2_when_localized = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_ready,
            on_exit=[
                LogInfo(msg='[LAUNCH] Localization ready -> starting Nav2 in 1s'),
                TimerAction(
                    period=1.0,
                    actions=[
                        _nav2_navigation(pkg_nav2_bringup, nav2_params_file, use_autonav)
                    ],
                ),
            ],
        )
    )

    # -----------------------------------------------------------------------
    # 11. Target -> Nav2 goal bridge (Nav2 mode only; the FSM does not use it)
    # -----------------------------------------------------------------------
    goal_client_enabled = PythonExpression([
        "'true' if ('", use_nav2_goal_client, "'.lower() in ('true', '1') and '",
        use_autonav, "'.lower() not in ('true', '1')) else 'false'",
    ])

    target_nav2_client = Node(
        package='ultrasonic_box_bot',
        executable='target_nav2_client',
        name='target_nav2_client',
        output='screen',
        condition=IfCondition(goal_client_enabled),
        parameters=[{
            'use_sim_time': True,
            'target_topic': '/target_cube/pose',
            'action_name': '/navigate_to_pose',
            'global_frame': 'map',
            'min_goal_distance': 0.25,
            'min_goal_yaw': 0.25,
            'start_delay_sec': 5.0,
            'retry_period_sec': 1.0,
            'cancel_on_new_target': False,
        }],
    )

    # -----------------------------------------------------------------------
    # 12. RViz2
    # -----------------------------------------------------------------------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    autonavigator = Node(
        package='ultrasonic_box_bot',
        executable='autonavigator_node',
        name='autonavigator_node',
        output='screen',
        condition=IfCondition(use_autonav),
        parameters=[{
            'use_sim_time': True,
            'cmd_vel_topic': '/cmd_vel',
            'data_timeout_sec': 0.5,
            'emergency_stop_distance': 0.22,
            'target_frame': 'odom',
        }],
    )

    # -----------------------------------------------------------------------
    # Launch Ordering
    # -----------------------------------------------------------------------
    return LaunchDescription([
        declare_use_autonav,
        declare_localization,
        declare_map,
        declare_use_rviz,
        declare_goal_client,
        declare_cmd_vel_timeout,
        declare_target_x,
        declare_target_y,
        validate_args,
        set_nv_offload, set_glx_vendor, set_no_sw,
        robot_state_publisher, gz_sim, spawn_entity, bridge,
        cmd_vel_watchdog,
        target_cube_publisher,
        map_padder,
        sensor_ready,
        slam_toolbox_node, activate_slam_toolbox, start_slam_or_localization,
        start_nav2_when_slam_active,
        start_nav2_when_localized,
        target_nav2_client,
        autonavigator,
        rviz2,
    ])

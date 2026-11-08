import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
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


def _is_true(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _validate_args(context, localization, use_autonav, map_file):
    """Fail fast on contradictory or incomplete launch arguments."""
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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Find the python node script path (either in source or in share/lib)
    pkg_src_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    mujoco_script_candidate = os.path.join(pkg_src_dir, 'scripts', 'mujoco_simulator_node.py')
    if not os.path.isfile(mujoco_script_candidate):
        mujoco_script_candidate = os.path.join(pkg_share, 'scripts', 'mujoco_simulator_node.py')

    # Resolve python interpreter with mujoco installed (.venv preferred)
    venv_candidates = [
        os.path.join(os.environ['VIRTUAL_ENV'], 'bin', 'python3') if 'VIRTUAL_ENV' in os.environ else None,
        os.path.expanduser('~/ros2/ros2_ws/src/ultrasonic_box_bot/.venv/bin/python3'),
        os.path.join(pkg_src_dir, '.venv', 'bin', 'python3'),
        os.path.abspath(os.path.join(pkg_src_dir, '..', '..', '.venv', 'bin', 'python3')),
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'ultrasonic_box_bot', '.venv', 'bin', 'python3')),
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', 'src', 'ultrasonic_box_bot', '.venv', 'bin', 'python3')),
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '.venv', 'bin', 'python3')),
    ]
    python_exec = sys.executable
    for cand in venv_candidates:
        if cand and os.path.isfile(cand):
            python_exec = cand
            break

    use_autonav = LaunchConfiguration('use_autonav')
    localization = LaunchConfiguration('localization')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    real_time_factor = LaunchConfiguration('real_time_factor')
    use_nav2_goal_client = LaunchConfiguration('use_nav2_goal_client')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    mujoco_model = LaunchConfiguration('mujoco_model')

    default_model_path = os.path.join(pkg_share, 'worlds', 'mujoco', 'slam_world.xml')
    if not os.path.isfile(default_model_path):
        default_model_path = os.path.join(pkg_src_dir, 'worlds', 'mujoco', 'slam_world.xml')

    declare_use_autonav = DeclareLaunchArgument(
        'use_autonav',
        default_value='false',
        description='Run the scripted wall-following FSM instead of Nav2.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Default false: live SLAM mapping; true: AMCL saved-map localization.',
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
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run MuJoCo physics headless without native GLFW viewer window.',
    )
    declare_real_time_factor = DeclareLaunchArgument(
        'real_time_factor',
        default_value='1.0',
        description='Target real-time factor (1.0 = real-time, 1.5 = 1.5x faster).',
    )
    declare_mujoco_model = DeclareLaunchArgument(
        'mujoco_model',
        default_value=default_model_path,
        description='Absolute path to MuJoCo MJCF XML world model.',
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
        description='Target x (m) published on /target_cube/pose.',
    )
    declare_target_y = DeclareLaunchArgument(
        'target_y',
        default_value='-4.0',
        description='Target y (m) published on /target_cube/pose.',
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'ultrasonic_box_bot.urdf.xacro')
    if not os.path.isfile(xacro_file):
        xacro_file = os.path.join(pkg_src_dir, 'urdf', 'ultrasonic_box_bot.urdf.xacro')

    rviz_config = os.path.join(pkg_share, 'rviz', 'view_bot_rviz_config.rviz')
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(pkg_src_dir, 'rviz', 'view_bot_rviz_config.rviz')

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    if not os.path.isfile(slam_params_file):
        slam_params_file = os.path.join(pkg_src_dir, 'config', 'slam_toolbox_params.yaml')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    if not os.path.isfile(nav2_params_file):
        nav2_params_file = os.path.join(pkg_src_dir, 'config', 'nav2_params.yaml')

    validate_args = OpaqueFunction(
        function=_validate_args,
        args=[localization, use_autonav, map_file],
    )

    # 1. Robot State Publisher
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

    mujoco_simulator_node = Node(
        executable=python_exec,
        arguments=[mujoco_script_candidate],
        name='mujoco_simulator_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'model_path': mujoco_model,
            'headless': ParameterValue(headless, value_type=bool),
            'real_time_factor': ParameterValue(real_time_factor, value_type=float),
            'cmd_vel_topic': '/cmd_vel_safe',
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'cmd_vel_timeout': ParameterValue(cmd_vel_timeout, value_type=float),
        }]
    )

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
                    msg='[MUJOCO LAUNCH] Sensors ready -> configuring SLAM Toolbox',
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
                    msg='[MUJOCO LAUNCH] Sensors ready -> starting saved-map localization',
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
                LogInfo(msg='[MUJOCO LAUNCH] Activating SLAM Toolbox...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    start_nav2_when_slam_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg='[MUJOCO LAUNCH] SLAM active -> starting Nav2 in 3s'),
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
                LogInfo(msg='[MUJOCO LAUNCH] Localization ready -> starting Nav2 in 1s'),
                TimerAction(
                    period=1.0,
                    actions=[
                        _nav2_navigation(pkg_nav2_bringup, nav2_params_file, use_autonav)
                    ],
                ),
            ],
        )
    )

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

    return LaunchDescription([
        declare_use_autonav,
        declare_localization,
        declare_map,
        declare_use_rviz,
        declare_headless,
        declare_real_time_factor,
        declare_mujoco_model,
        declare_goal_client,
        declare_cmd_vel_timeout,
        declare_target_x,
        declare_target_y,
        validate_args,
        robot_state_publisher,
        mujoco_simulator_node,
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


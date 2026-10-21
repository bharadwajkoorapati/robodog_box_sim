"""Persist the live SLAM map so a later run can localize instead of re-map.

Run this while the mapping launch is up and SLAM is publishing /map:

    ros2 launch ultrasonic_box_bot save_map_launch.py map_name:=$HOME/maze_map

Writes <map_name>.yaml and <map_name>.pgm.  The map is read from the transient
local /map topic, so it does not matter whether the robot is moving; sim time
is enabled because the map server resolves /map stamps against /clock.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=os.path.join(os.path.expanduser('~'), 'maze_map'),
        description='Absolute path prefix for the saved map (.yaml and .pgm).',
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        arguments=['-f', map_name],
        parameters=[{
            'use_sim_time': True,
            'save_map_timeout': 10.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }],
    )

    return LaunchDescription([
        declare_map_name,
        LogInfo(msg=['[map_saver] saving /map as ', map_name, '.yaml + .pgm']),
        map_saver,
    ])

# Copyright 2026 Riccardo Enrico
# SPDX-License-Identifier: BSD-3-Clause

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tracker = LaunchConfiguration('tracker')
    return LaunchDescription([
        DeclareLaunchArgument('tracker', default_value='drone1',
                              description='vrpn_mocap rigid-body name'),
        Node(
            package='vrpn_px4_bridge',
            executable='vrpn_px4_bridge_node',
            name='vrpn_px4_bridge',
            output='screen',
            parameters=[{'tracker': tracker}],
        ),
    ])

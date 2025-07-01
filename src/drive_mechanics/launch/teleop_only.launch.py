#!/usr/bin/env python3
"""
Minimal PC launch file for teleop only
Runs: Teleop keyboard control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable teleop control'
    )

    # Teleop Twist Keyboard - the main node we need
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in separate terminal
        remappings=[
            # This will send commands over network to Pi Zero
            ('cmd_vel', 'cmd_vel'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )

    return LaunchDescription([
        # Launch arguments
        enable_teleop_arg,

        # Control nodes
        teleop_keyboard,
    ])

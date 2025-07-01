#!/usr/bin/env python3
"""
Launch file for Pi Zero - Hardware interface only
Runs: Motor control node, diagnostics
Network: Publishes to topics that PC can subscribe to
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the config file path
    pkg_share = FindPackageShare(
        package='drive_mechanics').find('drive_mechanics')
    config_file_path = os.path.join(pkg_share, 'config', 'motor_config.yaml')

    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot (for multi-robot setups)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )

    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic publishing'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # Drive mechanics node - MAIN HARDWARE INTERFACE
    drive_mechanics_node = Node(
        package='drive_mechanics',
        executable='drive_node',
        name='drive_mechanics_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            config_file_path,
            {
                'use_sim_time': False,  # Pi Zero uses real time
                'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
                'robot_name': LaunchConfiguration('robot_name'),
                'enable_debug': False,  # Keep lightweight
            }
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        remappings=[
            # These topics will be available across the network
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
            ('diagnostics', 'diagnostics'),
            ('motor_status', 'motor_status'),
        ]
    )

    # Optional: Robot state broadcaster (lightweight TF publishing)
    robot_state_broadcaster = Node(
        package='drive_mechanics',
        executable='robot_state_broadcaster',
        name='robot_state_broadcaster',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'robot_name': LaunchConfiguration('robot_name'),
                'publish_frequency': 10.0,  # Lower frequency to save bandwidth
            }
        ],
        # Only include if you want basic TF on Pi Zero
        condition=LaunchConfiguration('enable_diagnostics')
    )

    return LaunchDescription([
        # Set environment for network discovery
        # Make sure both use same domain
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Launch arguments
        robot_name_arg,
        namespace_arg,
        enable_diagnostics_arg,
        log_level_arg,

        # Hardware nodes (Pi Zero)
        drive_mechanics_node,
        # robot_state_broadcaster,  # Uncomment if needed
    ])

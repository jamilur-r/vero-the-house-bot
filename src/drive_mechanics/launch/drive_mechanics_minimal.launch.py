#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='drive_mechanics').find('drive_mechanics')
    
    # Path to config file
    config_file_path = os.path.join(pkg_share, 'config', 'motor_config.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Drive mechanics node (minimal for nav2)
    drive_mechanics_node = Node(
        package='drive_mechanics',
        executable='drive_node',
        name='drive_mechanics_node',
        output='screen',
        parameters=[
            config_file_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_diagnostics': True,
                'enable_debug': False,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # Nodes
        drive_mechanics_node,
    ])

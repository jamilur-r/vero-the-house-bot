#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic publishing'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug output'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # Drive mechanics node
    drive_mechanics_node = Node(
        package='drive_mechanics',
        executable='drive_node',
        name='drive_mechanics_node',
        output='screen',
        parameters=[
            config_file_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
                'enable_debug': LaunchConfiguration('enable_debug'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/diagnostics', '/diagnostics'),
        ]
    )
    
    # Robot state publisher (optional - for TF tree visualization)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': get_robot_description()}
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_sim_time'), "' == 'false'"]))
    )
    
    # Joint state publisher (for robot model)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_sim_time'), "' == 'false'"]))
    )
    
    # Teleop keyboard node (optional - for testing)
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in separate terminal
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_debug'))
    )

    return LaunchDescription([
        # Environment variables
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Launch arguments
        use_sim_time_arg,
        enable_diagnostics_arg,
        enable_debug_arg,
        log_level_arg,
        
        # Nodes
        drive_mechanics_node,
        robot_state_publisher,
        joint_state_publisher,
        teleop_keyboard,
    ])


def get_robot_description():
    """Generate a simple URDF for the robot"""
    return '''<?xml version="1.0"?>
<robot name="drive_mechanics_robot">
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.12 0.10 0.05"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.12 0.10 0.05"/>
            </geometry>
        </collision>
    </link>
    
    <!-- Base footprint -->
    <link name="base_footprint"/>
    
    <joint name="base_footprint_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 0.0375" rpy="0 0 0"/>
    </joint>
    
    <!-- Left wheel -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0375" length="0.02"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
    </link>
    
    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="0 0.0445 0" rpy="1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
    
    <!-- Right wheel -->
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0375" length="0.02"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
    </link>
    
    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.0445 0" rpy="1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>'''

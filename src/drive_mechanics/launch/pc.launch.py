#!/usr/bin/env python3
"""
Launch file for PC - Navigation, simulation, and teleop
Runs: Nav2, RViz, teleop, robot simulation
Network: Subscribes to Pi Zero topics, publishes commands
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the config file path
    pkg_share = FindPackageShare(
        package='drive_mechanics').find('drive_mechanics')
    config_file_path = os.path.join(pkg_share, 'config', 'motor_config.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Launch nav2 stack'
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable teleop control'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )

    # Robot State Publisher (for visualization and TF tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': get_robot_description()
            }
        ]
    )

    # Joint State Publisher (for robot visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(pkg_share, 'rviz', 'drive_mechanics.rviz')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # Teleop Twist Keyboard
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
            ('cmd_vel', 'cmd_vel'),  # This will go over network to Pi Zero
        ],
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )

    # Alternative: Teleop Twist Joy (for gamepad)
    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joy',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ],
        # Uncomment condition below to enable joy control instead of keyboard
        # condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )

    # Robot Monitor (diagnostics visualization)
    robot_monitor = Node(
        package='rqt_robot_monitor',
        executable='rqt_robot_monitor',
        name='robot_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # Network Bridge Node (if needed for different ROS2 distributions)
    # network_bridge = Node(
    #     package='drive_mechanics',
    #     executable='network_bridge',
    #     name='network_bridge',
    #     output='screen',
    # )

    return LaunchDescription([
        # Global parameters
        SetParameter(name='use_sim_time',
                     value=LaunchConfiguration('use_sim_time')),

        # Launch arguments
        use_sim_time_arg,
        enable_nav2_arg,
        enable_rviz_arg,
        enable_teleop_arg,
        robot_name_arg,
        namespace_arg,

        # Visualization and control nodes (PC)
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        teleop_keyboard,
        # teleop_joy,  # Uncomment for gamepad control
        robot_monitor,
        # network_bridge,  # Uncomment if needed
    ])


def get_robot_description():
    """Generate robot URDF for visualization"""
    return '''<?xml version="1.0"?>
<robot name="drive_mechanics_robot">
    <!-- Base footprint -->
    <link name="base_footprint"/>
    
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.12 0.10 0.05"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.12 0.10 0.05"/>
            </geometry>
        </collision>
    </link>
    
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
                <color rgba="0.1 0.1 0.1 1"/>
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
                <color rgba="0.1 0.1 0.1 1"/>
            </material>
        </visual>
    </link>
    
    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.0445 0" rpy="1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
    
    <!-- Lidar mount (optional) -->
    <link name="lidar_link">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.03"/>
            </geometry>
            <material name="red">
                <color rgba="0.8 0.1 0.1 1"/>
            </material>
        </visual>
    </link>
    
    <joint name="lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar_link"/>
        <origin xyz="0.04 0 0.04" rpy="0 0 0"/>
    </joint>
</robot>'''

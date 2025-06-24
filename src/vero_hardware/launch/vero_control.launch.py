from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('vero_hardware'),
            'urdf',
            'vero.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': robot_description_content}

    diff_drive_controller_config = PathJoinSubstitution([
        FindPackageShare('vero_hardware'),
        'config',
        'diff_drive_controller.yaml'
    ])

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, diff_drive_controller_config],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        ),
    ])

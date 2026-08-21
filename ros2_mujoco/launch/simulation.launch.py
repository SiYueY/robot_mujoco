from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', description='Path to ros2_mujoco hardware.yaml'),
        DeclareLaunchArgument('robot_description', description='URDF containing ros2_control hardware_config parameter'),
        DeclareLaunchArgument('controller_params', default_value='', description='Optional controller-manager YAML'),
        Node(
            package='controller_manager', executable='ros2_control_node', output='screen',
            parameters=[
                {'robot_description': LaunchConfiguration('robot_description'),
                 'use_sim_time': True},
                LaunchConfiguration('controller_params'),
            ],
        ),
    ])

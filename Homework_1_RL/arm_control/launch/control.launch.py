from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'control_params_file',
            default_value='arm_controller.yaml',
            description='YAML file with the controller configuration.',
        )
    )

    # Initialize Arguments
    use_sim = LaunchConfiguration('use_sim')
    control_params_file_name = LaunchConfiguration('control_params_file')
    control_params_file = PathJoinSubstitution([
        get_package_share_directory('arm_control'),
        'config',
        control_params_file_name
    ])

    # Node to spawn the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # Node to spawn the position controller
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # Assemble launch description
    nodes = [
        # Nodes to launch controller spawners
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)


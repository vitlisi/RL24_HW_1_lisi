from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_delay',
            default_value='5.0',
            description='Delay in seconds before starting the control nodes to ensure Gazebo is ready'
        )
    )

    # Initialize Arguments
    control_delay = LaunchConfiguration('control_delay')

    # Gazebo World Launch
    gazebo_world_launch_file = PathJoinSubstitution([
        FindPackageShare('arm_gazebo'),
        'launch',
        'arm_world.launch.py'
    ])
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_world_launch_file)
    )

    # Arm Control Launch with delay
    arm_control_launch_file = PathJoinSubstitution([
        FindPackageShare('arm_control'),
        'launch',
        'control.launch.py'
    ])
    controllers = TimerAction(
        period=control_delay,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_control_launch_file)
        )]
    )

    # Assemble launch description
    nodes = [
        # Declare arguments
        *declared_arguments,
        # Gazebo World
        gazebo_world,
        # Arm Controllers (with delay)
        controllers,
    ]

    return LaunchDescription(nodes)


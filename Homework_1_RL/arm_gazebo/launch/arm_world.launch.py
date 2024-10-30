from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r -v 1 empty.sdf',
            description='Arguments for Ignition Gazebo (gz_sim)'
        )
    )

    # Initialize Arguments
    gz_args = LaunchConfiguration('gz_args')

    # Define the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('arm_gazebo'),
        'urdf',
        'arm.urdf.xacro'
    )
    
    # Process the xacro file to generate URDF
    complete_robot_description = xacro.process_file(xacro_file).toxml()

    # Ignition Gazebo launch file location
    ignition_launch_file = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    # Include Ignition Gazebo launch description
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ignition_launch_file),
        launch_arguments={'gz_args': gz_args}.items()
    )
    
    # Node to spawn the robot in Ignition Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'arm', '-allow_renaming', 'true'],
    )

    # Node to bridge camera data
    bridge_camera = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        '--ros-args',
        '-r', '/camera:=/videocamera',
    ],
    output='screen'
    )

    # Node to publish the robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': complete_robot_description}],
    )

    # Assemble launch description
    nodes = [
        # Declare arguments
        *declared_arguments,
        # Nodes to launch Ignition Gazebo itself
        gazebo_ignition,
        # Node to publish the robot description
        robot_state_publisher_node,
        # Node to spawn the robot in Ignition Gazebo
        gz_spawn_entity,
        # Node to bridge camera data
        bridge_camera,
    ]

    return LaunchDescription(declared_arguments + nodes)


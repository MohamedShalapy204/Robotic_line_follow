import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package info
    pkg_name = 'line_follower_core'
    pkg_path = get_package_share_directory(pkg_name)

    # Path to xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Path to world file
    world_file = os.path.join(pkg_path, 'worlds', 'line_track.world')

    # Path to Gazebo ROS launch file
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'line_follower_robot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])

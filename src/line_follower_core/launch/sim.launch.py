import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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

    # Path to GUI folder (assuming it is in the source directory for easier development)
    # But usually it is installed in the share directory.
    gui_path = os.path.join(pkg_path, 'gui')

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

    # Core Logic Nodes
    line_sensor_node = Node(
        package=pkg_name,
        executable='line_sensor_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    encoder_odometry_node = Node(
        package=pkg_name,
        executable='encoder_odometry_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # line_controller_node is disabled so the robot doesn't auto-move
    # line_controller_node = Node(
    #     package=pkg_name,
    #     executable='line_controller_node.py',
    #     output='screen',
    #     parameters=[{'use_sim_time': True, 'kp': 1.0, 'base_speed': 0.1}]
    # )

    motor_driver_node = Node(
        package=pkg_name,
        executable='motor_driver_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Python HTTP Server and Browser Opener
    start_gui_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=gui_path,
        output='screen'
    )

    open_browser = ExecuteProcess(
        cmd=['xdg-open', 'http://localhost:8000'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        line_sensor_node,
        encoder_odometry_node,
        # line_controller_node, # Disabled
        motor_driver_node,
        rosbridge_node,
        start_gui_server,
        open_browser
    ])

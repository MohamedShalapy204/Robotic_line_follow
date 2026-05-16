import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Package info
    pkg_name = 'line_follower_core'
    pkg_path = get_package_share_directory(pkg_name)

    # --- UNIVERSAL CORE NODES (HARDWARE MODE) ---
    line_sensor_node = Node(
        package=pkg_name,
        executable='line_sensor_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'hardware_mode': True
        }]
    )

    encoder_odometry_node = Node(
        package=pkg_name,
        executable='encoder_odometry_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'hardware_mode': True
        }]
    )

    line_controller_node = Node(
        package=pkg_name,
        executable='line_controller_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'kp': 1.0, 
            'base_speed': 0.1
        }]
    )

    motor_driver_node = Node(
        package=pkg_name,
        executable='motor_driver_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'hardware_mode': True
        }]
    )

    # --- MICRO-ROS AGENT ---
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888']
    )

    # --- WEB SERVER & ROSBRIDGE ---
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    gui_path = os.path.join(pkg_path, 'gui')
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
        micro_ros_agent_node,
        line_sensor_node,
        encoder_odometry_node,
        line_controller_node,
        motor_driver_node,
        rosbridge_node,
        rosapi_node,
        start_gui_server,
        open_browser
    ])

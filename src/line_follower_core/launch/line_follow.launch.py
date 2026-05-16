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
            'use_sim_time': False
        }]
    )

    encoder_odometry_node = Node(
        package=pkg_name,
        executable='encoder_odometry_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    line_controller_node = Node(
        package=pkg_name,
        executable='line_controller_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'kp': 20.0, 
            'ki': 0.0,
            'kd': 0.1,
            'base_speed': 0.7
        }]
    )

    motor_driver_node = Node(
        package=pkg_name,
        executable='motor_driver_node.py',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        line_sensor_node,
        encoder_odometry_node,
        line_controller_node,
        motor_driver_node
    ])

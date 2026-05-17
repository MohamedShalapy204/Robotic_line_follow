import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- ROSBRIDGE & ROSAPI FOR EXTERNAL COMMUNICATION ---
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

    return LaunchDescription([
        rosbridge_node,
        rosapi_node
    ])

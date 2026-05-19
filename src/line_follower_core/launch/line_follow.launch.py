import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    encoder_odometry_node = Node(
        package='line_follower_core',
        executable='encoder_odometry_node.py',
        output='screen',
        parameters=[
            {'wheel_radius': 0.0325},
            {'wheel_separation': 0.135},
            {'ticks_per_rev': 20.0}
        ]
    )

    return LaunchDescription([
        encoder_odometry_node
    ])


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np

class BagPlotter(Node):
    def __init__(self):
        super().__init__('bag_plotter')
        self.odom_x = []
        self.odom_y = []
        self.errors = []
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/line_error', self.error_callback, 10)

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def error_callback(self, msg):
        self.errors.append(msg.data)

def main():
    rclpy.init()
    node = BagPlotter()
    print("==================================================")
    print("Listening to /odom and /line_error...")
    print("In another terminal, play your bag file:")
    print("    ros2 bag play rosbags/lap_recording_XXX")
    print("Press Ctrl+C when playback is finished to view plots.")
    print("==================================================")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nGenerating plots...")

    # Plot
    plt.figure(figsize=(12, 5))
    
    # Trajectory Plot
    plt.subplot(1, 2, 1)
    if node.odom_x and node.odom_y:
        plt.plot(node.odom_x, node.odom_y, '-o', markersize=2, label='Estimated Trajectory')
        plt.plot(node.odom_x[0], node.odom_y[0], 'go', label='Start')
        plt.plot(node.odom_x[-1], node.odom_y[-1], 'ro', label='End')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Odometry Trajectory (Forward Kinematics)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # Line Error Plot
    plt.subplot(1, 2, 2)
    if node.errors:
        rms_error = np.sqrt(np.mean(np.square(node.errors)))
        plt.plot(node.errors, label=f'Error (RMS: {rms_error:.3f})')
        plt.axhline(0, color='r', linestyle='--', alpha=0.5)
    plt.xlabel('Time Step / Msg Index')
    plt.ylabel('Lateral Error')
    plt.title('Line Tracking Error Over Time')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('kinematics_results.png')
    print("Saved plot to kinematics_results.png")
    plt.show()

if __name__ == '__main__':
    main()

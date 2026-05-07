#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")
        
        # Parameters (matching URDF)
        self.wheel_radius = 0.033
        self.wheel_separation = 0.17
        
        # Subscription to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.listener_callback,
            10
        )
        
        self.get_logger().info("Motor Driver Node (Phase 3) started.")
        self.get_logger().info(f"Kinematics: Radius={self.wheel_radius}, Separation={self.wheel_separation}")

    def listener_callback(self, msg):
        # Differential Drive Inverse Kinematics
        # v = (r/2) * (w_r + w_l)
        # omega = (r/L) * (w_r - w_l)
        
        v = msg.linear.x
        omega = msg.angular.z
        
        # Solve for wheel angular velocities (rad/s)
        w_r = (2 * v + omega * self.wheel_separation) / (2 * self.wheel_radius)
        w_l = (2 * v - omega * self.wheel_separation) / (2 * self.wheel_radius)
        
        # In simulation, the Gazebo plugin handles the actual movement from cmd_vel.
        # This node acts as a monitor/pre-processor for hardware parity.
        # We log the "target" wheel speeds that would be sent to motors.
        self.get_logger().info(f"Target Wheel Speeds: L={w_l:.2f} rad/s, R={w_r:.2f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

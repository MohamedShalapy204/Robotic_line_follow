#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")
        
        # Parameters
        self.declare_parameter("hardware_mode", False)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("pwm_gain", 20.0) # Gain to convert rad/s to PWM
        
        self.hardware_mode = self.get_parameter("hardware_mode").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.pwm_gain = self.get_parameter("pwm_gain").value
        
        # Subscribers
        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10
        )
        
        # Publishers (Hardware Only)
        if self.hardware_mode:
            self.get_logger().info("Motor Driver: HARDWARE MODE (Publishing to /motor/*/pwm)")
            self.pub_l = self.create_publisher(Int32, "motor/left/pwm", 10)
            self.pub_r = self.create_publisher(Int32, "motor/right/pwm", 10)
        else:
            self.get_logger().info("Motor Driver: SIMULATION MODE (Monitoring cmd_vel)")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Inverse Kinematics (rad/s)
        w_r = (2 * v + omega * self.wheel_separation) / (2 * self.wheel_radius)
        w_l = (2 * v - omega * self.wheel_separation) / (2 * self.wheel_radius)
        
        if self.hardware_mode:
            # Map rad/s to PWM (-255 to 255)
            pwm_l = int(w_l * self.pwm_gain)
            pwm_r = int(w_r * self.pwm_gain)
            
            # Constrain to 8-bit
            pwm_l = max(-255, min(255, pwm_l))
            pwm_r = max(-255, min(255, pwm_r))
            
            msg_l = Int32()
            msg_l.data = pwm_l
            msg_r = Int32()
            msg_r.data = pwm_r
            
            self.pub_l.publish(msg_l)
            self.pub_r.publish(msg_r)
        else:
            # Simulation: Gazebo plugin handles cmd_vel directly.
            # We just log for monitoring.
            # self.get_logger().info(f"Monitor: L={w_l:.2f} rad/s, R={w_r:.2f} rad/s")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

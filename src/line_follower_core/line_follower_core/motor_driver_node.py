#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")
        
        # Parameters
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_separation", 0.11)
        self.declare_parameter("pwm_gain", 20.0) # Gain to convert rad/s to PWM
        
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
        
        # Publishers
        self.pub_l = self.create_publisher(Int32, "motor/left/pwm", 10)
        self.pub_r = self.create_publisher(Int32, "motor/right/pwm", 10)
        
        self.get_logger().info("Motor Driver: Started (Controlling Real Motors)")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # Simple Proportional Mapping: 1.0 (input) = 255 (PWM)
        # We mix linear and angular velocity based on wheel separation
        # This treats 'v' and 'omega' as normalized control factors
        l_raw = v - (omega * self.wheel_separation / 2.0)
        r_raw = v + (omega * self.wheel_separation / 2.0)
        
        # Direct conversion to 8-bit PWM (0-1 -> 0-255)
        pwm_l = int(l_raw * 255)
        pwm_r = int(r_raw * 255)
        
        # Clamp to hardware limits (-255 to 255)
        pwm_l = max(-255, min(255, pwm_l))
        pwm_r = max(-255, min(255, pwm_r))
        
        # Publish for Hardware
        msg_l = Int32()
        msg_l.data = pwm_l
        msg_r = Int32()
        msg_r.data = pwm_r
        
        self.pub_l.publish(msg_l)
        self.pub_r.publish(msg_r)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

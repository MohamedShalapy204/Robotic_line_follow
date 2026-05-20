#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

class LineControllerNode(Node):
    def __init__(self):
        super().__init__("line_controller_node")
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Float32,
            "line_error",
            self.error_callback,
            10
        )
        
        
        # Publishers
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        
        # PID Parameters (Initial Guesses)
        self.declare_parameter("kp", 1.2)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("base_speed", 0.3)
        
        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value
        self.base_speed = self.get_parameter("base_speed").value
        
        # PID State
        self.prev_error = 0.0
        self.integral = 0.0
        self.kickstart_count = 0 
        
        self.get_logger().info("Line Controller Node started.")



    def error_callback(self, msg):
            
        error = msg.data
        
        # PID Logic
        self.integral += error
        derivative = error - self.prev_error
        
        angular_z = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        twist = Twist()

        twist.linear.x = self.base_speed
            
        twist.angular.z = angular_z
        
        self.publisher_.publish(twist)
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = LineControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

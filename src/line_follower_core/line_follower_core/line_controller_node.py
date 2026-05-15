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
        
        self.mission_sub = self.create_subscription(
            String,
            "mission_control",
            self.mission_callback,
            10
        )
        
        self.tuning_sub = self.create_subscription(
            String,
            "tuning_params",
            self.tuning_callback,
            10
        )
        
        self.is_active = False
        
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
        self.kickstart_enabled = False # Default to False
        
        self.get_logger().info("Line Controller Node (Phase 3) started.")

    def mission_callback(self, msg):
        command = msg.data.lower()
        if command == "start":
            self.is_active = True
            if self.kickstart_enabled:
                self.kickstart_count = 5 
                self.get_logger().info("Autonomous Mode: ACTIVATED (Kickstart Pulse Enabled)")
            else:
                self.get_logger().info("Autonomous Mode: ACTIVATED (Kickstart Pulse Disabled)")
        elif command == "stop":
            self.is_active = False
            self.get_logger().info("Autonomous Mode: STOPPED")
            # Publish 0 velocity immediately
            twist = Twist()
            self.publisher_.publish(twist)

    def tuning_callback(self, msg):
        try:
            params = json.loads(msg.data)
            if 'base_speed' in params:
                self.base_speed = float(params['base_speed'])
            if 'kp' in params:
                self.kp = float(params['kp'])
            if 'ki' in params:
                self.ki = float(params['ki'])
            if 'kd' in params:
                self.kd = float(params['kd'])
            if 'kickstart_enabled' in params:
                self.kickstart_enabled = bool(params['kickstart_enabled'])
            self.get_logger().info(f"Tuning Updated: Speed={self.base_speed}, Kp={self.kp}, Kickstart={self.kickstart_enabled}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse tuning parameters: {e}")

    def error_callback(self, msg):
        if not self.is_active:
            return
            
        error = msg.data
        
        # PID Logic
        self.integral += error
        derivative = error - self.prev_error
        
        angular_z = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        twist = Twist()
        
        # Apply Kickstart Pulse if active, otherwise use base_speed
        if self.kickstart_count > 0:
            twist.linear.x = 0.8
            self.kickstart_count -= 1
        else:
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")
        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.listener_callback,
            10
        )
        self.get_logger().info("Motor Driver Node has been started.")

    def listener_callback(self, msg):
        # Placeholder for motor command logic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

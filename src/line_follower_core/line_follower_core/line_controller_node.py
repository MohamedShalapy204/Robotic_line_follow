#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class LineControllerNode(Node):
    def __init__(self):
        super().__init__("line_controller_node")
        self.subscription = self.create_subscription(
            Float32,
            "line_error",
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Line Controller Node has been started.")

    def listener_callback(self, msg):
        twist = Twist()
        # Placeholder for PID logic
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__("encoder_odometry_node")
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.get_logger().info("Encoder Odometry Node has been started.")

    def timer_callback(self):
        msg = Odometry()
        # Placeholder for odom logic
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

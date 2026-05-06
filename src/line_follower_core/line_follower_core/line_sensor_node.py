#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class LineSensorNode(Node):
    def __init__(self):
        super().__init__("line_sensor_node")
        self.publisher_ = self.create_publisher(Float32, "line_error", 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.get_logger().info("Line Sensor Node has been started.")

    def timer_callback(self):
        msg = Float32()
        msg.data = 0.0 # Placeholder
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

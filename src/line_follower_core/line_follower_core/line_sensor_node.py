#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import Float32, Int32, String

class LineSensorNode(Node):
    def __init__(self):
        super().__init__("line_sensor_node")
        
        # Parameters
        # Sensor topics
        self.sensor_names = ["l2", "l1", "mid", "r1", "r2"]
        
        # Weights for each sensor
        self.weights = [2.0, 1.0, 0.0, -1.0, -2.0]
        
        # Current sensor values (1.0 for on line, 0.0 for ground)
        self.sensor_values = [0.0] * len(self.sensor_names)
        
        # Threshold for detecting the line (Sim only: 0.010 is on line, 0.020 is on ground)
        self.detection_threshold = 0.015
        self.hw_threshold = 2000 # Default for 12-bit ADC (0-4095)
        
        # Tuning Subscription
        self.tuning_sub = self.create_subscription(
            String,
            "tuning_params",
            self.tuning_callback,
            10
        )
        
        # Publishers
        self.error_pub = self.create_publisher(Float32, "line_error", 10)
        
        self.get_logger().info("Line Sensor Node: Started (Subscribing to /raw/sensor_*)")
        for i, name in enumerate(self.sensor_names):
            self.create_subscription(Int32, f"raw/sensor_{name}", self.make_hw_callback(i), 10)
        
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz

    def tuning_callback(self, msg):
        try:
            params = json.loads(msg.data)
            if 'sensor_threshold' in params:
                self.hw_threshold = int(params['sensor_threshold'])
                self.get_logger().info(f"Sensor Threshold Updated: {self.hw_threshold}")
        except Exception as e:
            pass

    def make_hw_callback(self, idx):
        def callback(msg):
            # Updated logic: value > threshold means we are on the line (1.0)
            is_line = 1.0 if msg.data < self.hw_threshold else 0.0
            self.sensor_values[idx] = is_line
        return callback

    def timer_callback(self):
        msg = Float32()
        total_on_line = sum(self.sensor_values)
        
        # Calculate weighted average for error, defaulting to 1.0 divisor if total_on_line is 0
        msg.data = sum(val * weight for val, weight in zip(self.sensor_values, self.weights)) / (total_on_line or 1.0)
        self.error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

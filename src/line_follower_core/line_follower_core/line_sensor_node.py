#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LineSensorNode(Node):
    def __init__(self):
        super().__init__("line_sensor_node")
        
        # Sensor topics
        self.sensor_topics = [
            "sensor_l2", "sensor_l1", "sensor_mid", "sensor_r1", "sensor_r2"
        ]
        
        # Weights for each sensor
        self.weights = [2.0, 1.0, 0.0, -1.0, -2.0]
        
        # Current sensor values
        self.sensor_values = [0.0] * len(self.sensor_topics)
        self.last_error = 0.0
        
        # Threshold for detecting the line (0.010 is on line, 0.020 is on ground)
        self.detection_threshold = 0.015
        
        # Create subscribers using a callback factory
        self.subs = []
        for i, topic in enumerate(self.sensor_topics):
            callback = self.make_callback(i)
            sub = self.create_subscription(
                LaserScan,
                topic,
                callback,
                10
            )
            self.subs.append(sub)
            
        self.publisher_ = self.create_publisher(Float32, "line_error", 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        
        self.get_logger().info("Line Sensor Node (Phase 3 - LaserScan) started.")

    def make_callback(self, idx):
        def callback(msg):
            self.sensor_callback(msg, idx)
        return callback

    def sensor_callback(self, msg, idx):
        # LaserScan.ranges is a list. For a single ray, we use index 0.
        if len(msg.ranges) > 0:
            dist = msg.ranges[0]
            if dist < self.detection_threshold:
                self.sensor_values[idx] = 1.0
            else:
                self.sensor_values[idx] = 0.0

    def timer_callback(self):
        msg = Float32()
        
        total_on_line = sum(self.sensor_values)
        
        if total_on_line > 0:
            # Weighted average for error
            error = sum(val * weight for val, weight in zip(self.sensor_values, self.weights)) / total_on_line
            self.last_error = error
            msg.data = error
            self.get_logger().info(f"Line detected! Active sensors: {total_on_line} | Error: {error:.2f}")
        else:
            # If no line is detected, use 50% of the last error to smoothly recover
            # rather than jumping straight to zero.
            self.last_error *= 0.5
            msg.data = self.last_error
            # Use throttle to avoid flooding logs
            self.get_logger().warn("No line detected! Using recovery error.", throttle_duration_sec=2.0)
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

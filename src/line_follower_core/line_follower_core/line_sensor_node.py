#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
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
        
        # Threshold for detecting the line
        # The line is 1mm high, ground is 0mm. 
        # Sensor is at ~43mm from ground.
        # Line detection distance should be ~42mm.
        self.detection_threshold = 0.0425
        
        # Create subscribers
        self.subs = []
        for i, topic in enumerate(self.sensor_topics):
            sub = self.create_subscription(
                Range,
                topic,
                lambda msg, idx=i: self.sensor_callback(msg, idx),
                10
            )
            self.subs.append(sub)
            
        self.publisher_ = self.create_publisher(Float32, "line_error", 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        
        self.get_logger().info("Line Sensor Node (Phase 3) started.")

    def sensor_callback(self, msg, idx):
        # In Gazebo, a ray sensor returns the range.
        # If range < threshold, we assume it's on the line.
        if msg.range < self.detection_threshold:
            self.sensor_values[idx] = 1.0
        else:
            self.sensor_values[idx] = 0.0

    def timer_callback(self):
        msg = Float32()
        
        total_on_line = sum(self.sensor_values)
        
        if total_on_line > 0:
            # Weighted average for error
            error = sum(val * weight for val, weight in zip(self.sensor_values, self.weights)) / total_on_line
            msg.data = error
            self.get_logger().info(f"Line detected! Active sensors: {total_on_line} | Error: {error:.2f}")
        else:
            msg.data = 0.0
            # Use throttle to avoid flooding logs
            self.get_logger().warn("No line detected!", throttle_duration_sec=2.0)
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

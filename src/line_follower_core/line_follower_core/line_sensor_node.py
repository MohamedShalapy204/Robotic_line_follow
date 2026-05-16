#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32, String

class LineSensorNode(Node):
    def __init__(self):
        super().__init__("line_sensor_node")
        
        # Parameters
        self.declare_parameter("hardware_mode", False)
        self.hardware_mode = self.get_parameter("hardware_mode").value
        
        # Sensor topics
        self.sensor_names = ["l2", "l1", "mid", "r1", "r2"]
        
        # Weights for each sensor
        self.weights = [2.0, 1.0, 0.0, -1.0, -2.0]
        
        # Current sensor values (1.0 for on line, 0.0 for ground)
        self.sensor_values = [0.0] * len(self.sensor_names)
        self.last_error = 0.0
        
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
        self.mission_pub = self.create_publisher(String, "mission_control", 10)
        self.gui_pubs = []
        
        # Line lost tracking
        self.line_lost_start_time = None
        self.stopped_by_line_loss = False
        
        if self.hardware_mode:
            self.get_logger().info("Line Sensor Node: HARDWARE MODE (Subscribing to /raw/sensor_*)")
            # In hardware mode, we subscribe to Int32 and publish LaserScan for GUI compatibility
            for i, name in enumerate(self.sensor_names):
                self.create_subscription(Int32, f"raw/sensor_{name}", self.make_hw_callback(i), 10)
                self.gui_pubs.append(self.create_publisher(LaserScan, f"sensor_{name}", 10))
        else:
            self.get_logger().info("Line Sensor Node: SIMULATION MODE (Subscribing to LaserScan)")
            for i, name in enumerate(self.sensor_names):
                self.create_subscription(LaserScan, f"sensor_{name}", self.make_sim_callback(i), 10)
        
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
            
            # Publish LaserScan for GUI
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = f"sensor_{self.sensor_names[idx]}"
            # 0.010 = Line, 0.020 = Ground
            dist = 0.010 if is_line == 1.0 else 0.020
            scan.ranges = [dist]
            # Send raw value in intensities field for GUI display
            scan.intensities = [float(msg.data)]
            self.gui_pubs[idx].publish(scan)
        return callback

    def make_sim_callback(self, idx):
        def callback(msg):
            if len(msg.ranges) > 0:
                dist = msg.ranges[0]
                self.sensor_values[idx] = 1.0 if dist < self.detection_threshold else 0.0
        return callback

    def timer_callback(self):
        msg = Float32()
        total_on_line = sum(self.sensor_values)
        
        if total_on_line > 0:
            # Line found
            self.line_lost_start_time = None
            self.stopped_by_line_loss = False
            # Weighted average for error
            error = sum(val * weight for val, weight in zip(self.sensor_values, self.weights)) / total_on_line
            self.last_error = error
            msg.data = error
            self.error_pub.publish(msg)
        else:
            # Line lost
            if not self.stopped_by_line_loss:
                if self.line_lost_start_time is None:
                    self.line_lost_start_time = self.get_clock().now()
                
                elapsed = (self.get_clock().now() - self.line_lost_start_time).nanoseconds / 1e9
                
                if elapsed >= 0.5:
                    self.get_logger().info("Line lost for 0.5s, stopping.")
                    stop_msg = String()
                    stop_msg.data = "stop"
                    self.mission_pub.publish(stop_msg)
                    self.stopped_by_line_loss = True
                    
                    # Stop publishing error or publish 0.0
                    msg.data = 0.0
                    self.error_pub.publish(msg)
                else:
                    # Within 0.5s grace period: keep last error but don't search
                    msg.data = self.last_error
                    self.error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import tf2_ros

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__("encoder_odometry_node")
        
        # Parameters
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_separation", 0.135)
        self.declare_parameter("ticks_per_rev", 20.0)
        
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.first_run = True
        self.last_time = None
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.get_logger().info("Encoder Odometry: HARDWARE MODE (Subscribing to /raw/encoder_*)")
        self.create_subscription(Int32, "/raw/encoder_l", self.hw_left_callback, 10)
        self.create_subscription(Int32, "/raw/encoder_r", self.hw_right_callback, 10)

    def hw_left_callback(self, msg):
        # Convert ticks to radians: (ticks / PPR) * 2 * PI
        pos_rad = (msg.data / self.ticks_per_rev) * 2.0 * math.pi
        self.update_odometry(pos_rad, self.right_pos)
        self.left_pos = pos_rad

    def hw_right_callback(self, msg):
        pos_rad = (msg.data / self.ticks_per_rev) * 2.0 * math.pi
        self.update_odometry(self.left_pos, pos_rad)
        self.right_pos = pos_rad

    def update_odometry(self, curr_l, curr_r):
        if self.first_run:
            self.left_pos = curr_l
            self.right_pos = curr_r
            self.first_run = False
            self.last_time = self.get_clock().now()
            return

        # Delta position (in radians)
        d_l = curr_l - self.left_pos
        d_r = curr_r - self.right_pos

        # Delta distance
        dist_l = d_l * self.wheel_radius
        dist_r = d_r * self.wheel_radius

        # Unicycle Kinematics
        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self.wheel_separation

        # Update Pose
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocity Calculation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        linear_vel = 0.0
        angular_vel = 0.0
        if dt > 0:
            linear_vel = d_center / dt
            angular_vel = d_theta / dt
        
        self.last_time = now
        self.publish_odom(linear_vel, angular_vel)

    def publish_odom(self, linear_vel, angular_vel):
        now_msg = self.last_time.to_msg()
        
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

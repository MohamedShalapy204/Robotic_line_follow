#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import tf2_ros

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__("encoder_odometry_node")
        
        # Parameters
        self.wheel_radius = 0.033
        self.wheel_separation = 0.17
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.first_run = True
        
        # Subscribers
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info("Encoder Odometry Node (Phase 3) started.")

    def joint_states_callback(self, msg):
        try:
            l_idx = msg.name.index("left_wheel_joint")
            r_idx = msg.name.index("right_wheel_joint")
        except ValueError:
            return

        curr_l_pos = msg.position[l_idx]
        curr_r_pos = msg.position[r_idx]

        if self.first_run:
            self.left_wheel_pos = curr_l_pos
            self.right_wheel_pos = curr_r_pos
            self.first_run = False
            return

        # Delta position (in radians)
        d_l = curr_l_pos - self.left_wheel_pos
        d_r = curr_r_pos - self.right_wheel_pos

        # Delta distance
        dist_l = d_l * self.wheel_radius
        dist_r = d_r * self.wheel_radius

        # Update previous positions
        self.left_wheel_pos = curr_l_pos
        self.right_wheel_pos = curr_r_pos

        # Unicycle Kinematics
        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self.wheel_separation

        # Update Pose
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odometry()

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation = q
        
        self.odom_pub.publish(odom)

        # TF Transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
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

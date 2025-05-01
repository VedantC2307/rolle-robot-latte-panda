#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
import tf_transformations

class PoseToOdomBridge(Node):
    def __init__(self):
        super().__init__('pose_to_odom_bridge')
        
        # Declare and get parameters
        self.declare_parameter("sensor_pose_topic", "/rolle/posestamped_aligned")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_tf", True)
        self.sensor_pose_topic = self.get_parameter("sensor_pose_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # Publisher for Odometry messages
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Subscriber for sensor PoseStamped
        self.sensor_sub = self.create_subscription(
            PoseStamped,
            self.sensor_pose_topic,
            self.pose_callback,
            10
        )

        # TF broadcaster for odom -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Define the transformation matrix to align base_link with odom frame
        self.align_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        self.get_logger().info(f"Subscribed to: {self.sensor_pose_topic}")
        self.get_logger().info(f"Publishing Odometry on: {self.odom_topic}")

    def pose_callback(self, pose_msg: PoseStamped):
        # Get the current time for headers
        now = self.get_clock().now()

        # Extract pose data
        pos = pose_msg.pose.position
        quat = pose_msg.pose.orientation
        
        # Convert pose to homogeneous transformation matrix
        pose_matrix = tf_transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        pose_matrix[0, 3] = pos.x
        pose_matrix[1, 3] = pos.y
        pose_matrix[2, 3] = pos.z
        
        # Apply alignment transform
        aligned_matrix = np.matmul(self.align_transform, pose_matrix)
        
        # Extract aligned position and orientation
        aligned_pos = (aligned_matrix[0, 3], aligned_matrix[1, 3], aligned_matrix[2, 3])
        aligned_quat = tf_transformations.quaternion_from_matrix(aligned_matrix)

        # Create the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "position_sensor_frame"      # Global odometry frame
        odom_msg.child_frame_id = "base_link"   # Robot's base frame

        # Set the transformed pose in the odom message
        odom_msg.pose.pose.position.x = aligned_pos[0]
        odom_msg.pose.pose.position.y = aligned_pos[1]
        odom_msg.pose.pose.position.z = aligned_pos[2]
        odom_msg.pose.pose.orientation.x = aligned_quat[0]
        odom_msg.pose.pose.orientation.y = aligned_quat[1]
        odom_msg.pose.pose.orientation.z = aligned_quat[2]
        odom_msg.pose.pose.orientation.w = aligned_quat[3]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Optionally broadcast TF from odom to base_link
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = "position_sensor_frame"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = aligned_pos[0]
            tf_msg.transform.translation.y = aligned_pos[1]
            tf_msg.transform.translation.z = aligned_pos[2]
            tf_msg.transform.rotation.x = aligned_quat[0]
            tf_msg.transform.rotation.y = aligned_quat[1]
            tf_msg.transform.rotation.z = aligned_quat[2]
            tf_msg.transform.rotation.w = aligned_quat[3]
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

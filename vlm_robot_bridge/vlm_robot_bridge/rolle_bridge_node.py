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
        self.declare_parameter("sensor_pose_topic", "/rolle/posestamped")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_tf", True)
        self.sensor_pose_topic = self.get_parameter("sensor_pose_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # Publisher for Odometry messages (base_link in odom)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Subscriber for sensor PoseStamped (published in position_sensor_frame)
        self.sensor_sub = self.create_subscription(
            PoseStamped,
            self.sensor_pose_topic,
            self.pose_callback,
            10
        )

        # TF broadcaster for odom -> base_link (dynamic)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # TF static broadcaster for base_link -> position_sensor_frame (static)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.child_frame_id = "base_link"

        # Define the fixed transform from base_link to position_sensor_frame.
        # This is T_bs: transformation from base_link to sensor frame.
        # Adjust the translation and rotation as needed.
        # Here we define a transformation with:
        # 1. 180 degrees (pi) rotation about the z-axis
        # 2. 180 degrees (pi) rotation about the x-axis
        # and the existing coordinate transformations.
        
        # Create a rotation matrix for pi radians about z-axis
        theta_z = np.pi  # 180 degrees in radians
        cos_z = np.cos(theta_z)
        sin_z = np.sin(theta_z)
        
        # Create a rotation matrix for pi radians about x-axis
        theta_x = np.pi  # 180 degrees in radians
        cos_x = np.cos(theta_x)
        sin_x = np.sin(theta_x)
        
        # Z-axis rotation matrix
        R_z = np.array([
            [cos_z, sin_z, 0, 0],
            [-sin_z, cos_z, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # X-axis rotation matrix
        R_x = np.array([
            [1, 0, 0, 0],
            [0, cos_x, sin_x, 0],
            [0, -sin_x, cos_x, 0],
            [0, 0, 0, 1]
        ])
        
        # Original coordinate transformation
        T_orig = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ])
        
        # Apply the rotations in sequence: first z, then x, then original transformation
        self.T_bs = R_z @ R_x @ T_orig

        # Publish the static transform from base_link to position_sensor_frame once.
        self.publish_static_transform()

        self.get_logger().info(f"Subscribed to sensor pose on: {self.sensor_pose_topic}")
        self.get_logger().info(f"Publishing Odometry on: {self.odom_topic}")

    def publish_static_transform(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id = "position_sensor_frame"
        static_tf.transform.translation.x = float(self.T_bs[0, 3])
        static_tf.transform.translation.y = float(self.T_bs[1, 3])
        static_tf.transform.translation.z = float(self.T_bs[2, 3])
        # Convert rotation matrix (upper-left 3x3 of T_bs) to quaternion
        quat = tf_transformations.quaternion_from_matrix(self.T_bs)
        static_tf.transform.rotation.x = float(quat[0])
        static_tf.transform.rotation.y = float(quat[1])
        static_tf.transform.rotation.z = float(quat[2])
        static_tf.transform.rotation.w = float(quat[3])
        self.static_broadcaster.sendTransform(static_tf)

    def pose_callback(self, sensor_msg: PoseStamped):
        # The sensor_msg is published in the "position_sensor_frame" coordinate system.
        # It represents T_odom_sensor (the transform from odom to position_sensor_frame)
        # We need to compute T_odom_base such that:
        #    T_odom_sensor = T_odom_base * T_bs   =>   T_odom_base = T_odom_sensor * inv(T_bs)

        # Build sensor (position_sensor_frame) homogeneous matrix from the PoseStamped:
        pos = sensor_msg.pose.position
        quat = sensor_msg.pose.orientation
        T_odom_sensor = tf_transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        T_odom_sensor[0, 3] = pos.x
        T_odom_sensor[1, 3] = pos.y
        T_odom_sensor[2, 3] = pos.z

        # Invert T_bs
        T_bs_inv = np.linalg.inv(self.T_bs)
        # Compute T_odom_base = T_odom_sensor * inv(T_bs)
        T_odom_base = np.dot(T_odom_sensor, T_bs_inv)

        # Extract base_link position and orientation from T_odom_base
        base_pos = (T_odom_base[0, 3], - T_odom_base[1, 3], T_odom_base[2, 3])
        base_quat_ = tf_transformations.quaternion_from_matrix(T_odom_base)

        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(base_quat_)
        yaw = -yaw # Invert the yaw angle
        base_quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Publish the odometry message for base_link relative to odom
        odom_msg = Odometry()
        now = self.get_clock().now()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"        # Global odom frame
        odom_msg.child_frame_id = self.child_frame_id    # Base frame
        odom_msg.pose.pose.position.x = base_pos[0]
        odom_msg.pose.pose.position.y = base_pos[1]
        odom_msg.pose.pose.position.z = base_pos[2]
        odom_msg.pose.pose.orientation.x = base_quat[0]
        odom_msg.pose.pose.orientation.y = base_quat[1]
        odom_msg.pose.pose.orientation.z = base_quat[2]
        odom_msg.pose.pose.orientation.w = base_quat[3]
        self.odom_pub.publish(odom_msg)

        # Broadcast TF from odom to base_link (dynamic)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = base_pos[0]
        tf_msg.transform.translation.y = base_pos[1]
        tf_msg.transform.translation.z = base_pos[2]
        tf_msg.transform.rotation.x = base_quat[0]
        tf_msg.transform.rotation.y = base_quat[1]
        tf_msg.transform.rotation.z = base_quat[2]
        tf_msg.transform.rotation.w = base_quat[3]
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

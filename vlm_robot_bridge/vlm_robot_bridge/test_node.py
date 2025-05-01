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
        
        # Parameters (could also be declared as ROS parameters)
        self.sensor_pose_topic = "/rolle/posestamped"
        self.odom_topic = "/odom"
        self.publish_tf = True
        self.child_frame_id = "base_footprint"

        # Publisher for Odometry messages (odometry of base_link in odom)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        # Subscriber for sensor pose published in "position_sensor_frame"
        self.sensor_sub = self.create_subscription(PoseStamped,
                                                   self.sensor_pose_topic,
                                                   self.pose_callback,
                                                   10)
        # TF broadcasters:
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Define static transform T_bs (from base_link to position_sensor_frame).
        # For example, if your sensor’s mounting relative to base_link is known,
        # include any translation or rotation here.
        # For a simple case, we assume T_bs is the identity matrix.
        # Combine 90° X-rotation with Y flip
        self.T_bs = np.eye(4)
        R_x90 = tf_transformations.quaternion_matrix(
            tf_transformations.quaternion_about_axis(np.pi / 2, [1, 0, 0])
        )[:3, :3]

        R_y_flip = np.diag([1, 1, 1])  # flip Y axis

        # Compose the rotation (first rotate, then flip)
        self.T_bs[:3, :3] = R_y_flip @ R_x90

        self.publish_static_transform()

        self.get_logger().info(f"Subscribed to sensor pose on: {self.sensor_pose_topic}")
        self.get_logger().info(f"Publishing Odometry on: {self.odom_topic}")

    def publish_static_transform(self):
        """Publish a static TF from base_link to position_sensor_frame."""
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id = "position_sensor_frame"
        # For T_bs, assume zero translation (or set if known)
        static_tf.transform.translation.x = float(self.T_bs[0, 3])
        static_tf.transform.translation.y = -float(self.T_bs[1, 3])
        static_tf.transform.translation.z = float(self.T_bs[2, 3])
        # Convert the rotation part of T_bs to a quaternion
        quat = tf_transformations.quaternion_from_matrix(self.T_bs)
        static_tf.transform.rotation.x = quat[0]
        static_tf.transform.rotation.y = quat[1]
        static_tf.transform.rotation.z = quat[2]
        static_tf.transform.rotation.w = quat[3]
        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info("Published static transform: base_link -> position_sensor_frame.")

    def pose_callback(self, sensor_msg: PoseStamped):
        """
        When a PoseStamped message is received (in the 'position_sensor_frame' frame),
        compute T_odom_base as:
          T_odom_base = T_odom_sensor * (T_bs)⁻¹
        and publish an Odometry message and dynamic TF from odom to base_link.
        """
        # Build T_odom_sensor from the PoseStamped message.
        pos = sensor_msg.pose.position
        quat = sensor_msg.pose.orientation
        T_odom_sensor = tf_transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        T_odom_sensor[0, 3] = pos.x
        T_odom_sensor[1, 3] = pos.y
        T_odom_sensor[2, 3] = pos.z

        # Invert T_bs (static transform from base_link to sensor frame)
        T_bs_inv = np.linalg.inv(self.T_bs)
        # Compute T_odom_base = T_odom_sensor * inv(T_bs)
        T_odom_base = np.dot(T_odom_sensor, T_bs_inv)

        # Extract translation and rotation from T_odom_base
        x = T_odom_base[0, 3]
        y = T_odom_base[1, 3]
        z = T_odom_base[2, 3]
        quat_base = tf_transformations.quaternion_from_matrix(T_odom_base)

        # Publish an Odometry message (odometry of base_link in odom)
        odom_msg = Odometry()
        now = self.get_clock().now()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = -y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.x = quat_base[0]
        odom_msg.pose.pose.orientation.y = quat_base[1]
        odom_msg.pose.pose.orientation.z = quat_base[2]
        odom_msg.pose.pose.orientation.w = quat_base[3]
        self.odom_pub.publish(odom_msg)

        # Broadcast dynamic TF from odom to base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = -y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = quat_base[0]
        tf_msg.transform.rotation.y = quat_base[1]
        tf_msg.transform.rotation.z = quat_base[2]
        tf_msg.transform.rotation.w = quat_base[3]
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(tf_msg)

    def __del__(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PoseToOdomBridge.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations
import numpy as np

class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('pose_frame_transformer')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/rolle/posestamped',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/rolle/posestamped_aligned', 10)

        # Define the transformation matrix:
        # New X = Old Z, New Y = -Old X, New Z = Old Y.
        self.T = np.eye(4)
        # Fix the transformation matrix - correct the definition
        self.T[:3, :3] = np.array([[0, 0, 1],
                                   [-1, 0, 0],
                                   [0, 1, 0]])

    def pose_callback(self, msg):
        # --- Transform the position ---
        pos_vec = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            1.0
        ])
        rotated_pos = np.dot(self.T, pos_vec)

        # --- Transform the orientation ---
        # Get the original quaternion
        q = msg.pose.orientation
        quat_orig = [q.x, q.y, q.z, q.w]

        # Convert to rotation matrix
        rot_mat = tf_transformations.quaternion_matrix(quat_orig)
        
        # Apply transformation to rotation part only
        # This preserves the special orthogonal properties needed for valid rotation
        transformed_rot = np.dot(self.T[:3, :3], rot_mat[:3, :3])
        
        # Create a new valid 4x4 transformation matrix
        new_rot_matrix = np.eye(4)
        new_rot_matrix[:3, :3] = transformed_rot
        
        # Convert the new rotation matrix back to a quaternion
        new_quat_temp = tf_transformations.quaternion_from_matrix(new_rot_matrix)
        
        # Apply roll adjustment (subtract pi/2 from roll)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(new_quat_temp)
        roll = roll - 0  # Subtract 90 degrees from roll
        new_quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # --- Create and publish the new PoseStamped ---
        new_msg = PoseStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = "position_sensor_frame"
        new_msg.pose.position.x = rotated_pos[0]
        new_msg.pose.position.y = rotated_pos[1]
        new_msg.pose.position.z = rotated_pos[2]
        new_msg.pose.orientation = Quaternion(
            x=new_quat[0],
            y=new_quat[1],
            z=new_quat[2],
            w=new_quat[3]
        )

        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

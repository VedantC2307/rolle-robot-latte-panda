#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_geometry_msgs
import logging

class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('pose_transform_node')
        
        # Configure logging
        self.get_logger().info("Initializing Pose Transform Node")
        
        # Create subscriber to the absolute sensor pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/rolle/posestamped',
            self.pose_callback,
            10
        )
        
        # Set up TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Set up broadcaster to publish transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # If there's a static transform between sensor and base_link, you could publish it
        # Example (uncomment if needed):
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # self.publish_static_transform()
        
        self.get_logger().info("Pose Transform Node initialized successfully")

    # Uncomment and modify if you need to publish a static transform
    # def publish_static_transform(self):
    #     static_transform = TransformStamped()
    #     static_transform.header.stamp = self.get_clock().now().to_msg()
    #     static_transform.header.frame_id = 'base_link'
    #     static_transform.child_frame_id = 'sensor_frame'
    #     
    #     # Set the static transformation values
    #     static_transform.transform.translation.x = 0.0  # Modify as needed
    #     static_transform.transform.translation.y = 0.0
    #     static_transform.transform.translation.z = 0.0
    #     static_transform.transform.rotation.x = 0.0
    #     static_transform.transform.rotation.y = 0.0
    #     static_transform.transform.rotation.z = 0.0
    #     static_transform.transform.rotation.w = 1.0
    #     
    #     self.static_broadcaster.sendTransform(static_transform)
    #     self.get_logger().info(f"Published static transform from base_link to sensor_frame")

    def pose_callback(self, msg):
        try:
            # Log incoming pose data
            self.get_logger().info(f"Received pose: position={msg.pose.position}, orientation={msg.pose.orientation}")
            
            # Get transform if needed (if your pose isn't already in the frame you want)
            # Example: transform from sensor_frame to base_link
            # transform = self.tf_buffer.lookup_transform(
            #     'base_link',         # target frame
            #     msg.header.frame_id, # source frame
            #     msg.header.stamp,    # time
            #     timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Create a transform to broadcast
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = 'odom'  # Parent frame (adjust as needed)
            transform.child_frame_id = 'base_link'  # Child frame (adjust as needed)
            
            # Copy pose data to transform
            transform.transform.translation.x = msg.pose.position.x
            transform.transform.translation.y = msg.pose.position.y
            transform.transform.translation.z = msg.pose.position.z
            transform.transform.rotation = msg.pose.orientation
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().info(f"Published transform from {transform.header.frame_id} to {transform.child_frame_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error in pose callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by keyboard interrupt")
    except Exception as e:
        node.get_logger().error(f"Error in node execution: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

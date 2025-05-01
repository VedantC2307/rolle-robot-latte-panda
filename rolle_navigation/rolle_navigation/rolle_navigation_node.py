#!/usr/bin/env python3
"""
rolle_navigation_node.py

This node subscribes to a topic that receives pixel coordinates,
converts them to Nav2 goal poses using map metadata,
and publishes the goals to the Nav2 system.
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import yaml
import json
import math
from PIL import Image

class RolleNavigationNode(Node):
    """
    A ROS2 node for converting pixel coordinates to Nav2 goal poses.
    """
    
    def __init__(self):
        super().__init__("rolle_navigation_node")
        
        # Declare parameters
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_image_path", "")
        
        # Get parameters
        self.map_yaml_path = self.get_parameter("map_yaml_path").get_parameter_value().string_value
        self.map_image_path = self.get_parameter("map_image_path").get_parameter_value().string_value
        
        if not self.map_yaml_path or not self.map_image_path:
            self.get_logger().error("Map paths not specified. Please provide map_yaml_path and map_image_path parameters.")
            return
            
        if not os.path.exists(self.map_yaml_path):
            self.get_logger().error(f"Map YAML file not found at: {self.map_yaml_path}")
            return
            
        if not os.path.exists(self.map_image_path):
            self.get_logger().error(f"Map image file not found at: {self.map_image_path}")
            return
        
        # Load map metadata
        self.load_map_metadata()
        
        # Create subscription to pixel coordinates topic
        self.pixel_coord_sub = self.create_subscription(
            String,  # Using String for now, will contain JSON pixel coordinates
            'pixel_coordinates',
            self.pixel_coordinates_callback,
            10
        )
        
        # Create publisher for Nav2 goal poses
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        
        self.get_logger().info("Rolle Navigation Node initialized successfully!")
        self.get_logger().info(f"Map resolution: {self.resolution}, Origin: {self.origin}")
        
    def load_map_metadata(self):
        """Load and store the map metadata from the YAML file"""
        try:
            with open(self.map_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                
            self.resolution = data['resolution']
            self.origin = data.get('origin', [0.0, 0.0, 0.0])
            
            # Load map image to get dimensions
            img = Image.open(self.map_image_path)
            self.map_width, self.map_height = img.size
            
            self.get_logger().info(f"Loaded map metadata. Size: {self.map_width}x{self.map_height}, Resolution: {self.resolution}")
            
        except Exception as e:
            self.get_logger().error(f"Error loading map metadata: {e}")
            raise
    
    def pixel_coordinates_callback(self, msg):
        """Convert received pixel coordinates to Nav2 goal and publish it"""
        try:
            # Parse the JSON string to get pixel coordinates
            data = json.loads(msg.data)
            
            # Extract pixel coordinates (u, v)
            u_px = int(data.get('x', 0))
            v_px = int(data.get('y', 0))
            
            self.get_logger().info(f"Received pixel coordinates: u={u_px}, v={v_px}")
            
            # Convert pixel coordinates to world coordinates (meters)
            x, y = self.pixel_to_world(u_px, v_px)
            
            self.get_logger().info(f"Converted to world coordinates: x={x:.2f}, y={y:.2f}")
            
            # Create and publish the goal pose
            self.publish_goal_pose(x, y)
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON format in message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing pixel coordinates: {e}")
    
    def pixel_to_world(self, u_px, v_px):
        """
        Convert pixel coordinates to world coordinates (meters)
        
        Args:
            u_px: x-coordinate in pixel space
            v_px: y-coordinate in pixel space
            
        Returns:
            tuple: (x, y) coordinates in world space (meters)
        """
        # Extract origin
        ox, oy = self.origin[0], self.origin[1]
        
        # Apply inverse of the formula used in rolle_nav2.py
        # Original: u = (x - ox) / resolution - 0.5
        # Original: v = H - ((y - oy) / resolution) - 0.5
        
        # Solve for x and y:
        x = (u_px + 0.5) * self.resolution + ox
        y = oy + (self.map_height - v_px - 0.5) * self.resolution
        
        return x, y
    
    def publish_goal_pose(self, x, y, theta=0.0):
        """
        Publish a goal pose to the Nav2 system
        
        Args:
            x: x-coordinate in world space (meters)
            y: y-coordinate in world space (meters)
            theta: orientation angle (radians)
        """
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        
        # Set position
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0
        
        # Set orientation (as quaternion)
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.orientation.w = math.cos(theta / 2.0)
        
        # Publish the goal
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = RolleNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
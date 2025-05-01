#!/usr/bin/env python3
"""Image utilities for map visualization."""

import cv2
import math
import base64
import io
import json
from PIL import Image
from typing import Optional, Tuple, Dict, Any

class ImageUtils:
    """Class containing utilities for map visualization."""
    
    def __init__(self, node):
        """
        Initialize image utilities with a reference to the ROS node.
        
        Args:
            node: The ROS node instance to access map data and robot position
        """
        self.node = node
        # Default maximum image dimensions for resizing to reduce token usage
        self.max_image_width = 400
        self.max_image_height = 400
        # Default image quality for JPEG compression (0-100, higher is better but larger)
        self.jpeg_quality = 70
    
    def get_map_image(self, input_str=None) -> str:
        """Return the current map with room colors as a base64 encoded image"""
        try:
            if not hasattr(self.node, 'map_image') or self.node.map_image is None:
                return json.dumps({"error": "Map image not available"})
                
            # Get the base64 encoded image with reduced size
            map_image_base64 = self.encode_image_to_base64(self.node.map_image, resize=True)
            
            # Return a proper JSON object
            return json.dumps({
                "map_image": map_image_base64,
                "message": "This is the colored map of the environment showing different rooms"
            })
        except Exception as e:
            self.node.get_logger().error(f"Error getting map image: {e}")
            return json.dumps({"error": f"Failed to get map image: {str(e)}"})
    
    def get_map_with_robot_position(self) -> str:
        """Generate a map visualization with the robot position marked"""
        try:
            if not hasattr(self.node, 'map_image') or self.node.map_image is None:
                return None
                
            # Create a copy of the map image to avoid modifying the original
            map_with_robot = self.node.map_image.copy()
            
            # Convert robot position from the map coordinate system to image pixel coordinates
            resolution = self.node.map_metadata['resolution']
            origin_x, origin_y, _ = self.node.map_metadata['origin']
            
            # Simply overlay the robot position on the existing colored map
            pixel_x = int((self.node.robot_position.x - origin_x) / resolution)
            pixel_y = int((self.node.robot_position.y - origin_y) / resolution)
            
            # Draw robot position (red circle with a radius of 5 pixels)
            cv2.circle(map_with_robot, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
            
            # Draw robot orientation (line pointing in the direction the robot is facing)
            orient_length = 15
            orient_x = int(pixel_x + orient_length * math.cos(self.node.robot_orientation))
            orient_y = int(pixel_y + orient_length * math.sin(self.node.robot_orientation))
            cv2.line(map_with_robot, (pixel_x, pixel_y), (orient_x, orient_y), (0, 0, 255), 2)
            
            # Encode the image with robot position to base64 with reduced size
            return self.encode_image_to_base64(map_with_robot, resize=True)
            
        except Exception as e:
            self.node.get_logger().error(f"Error creating map visualization: {e}")
            return None
    
    def resize_image(self, image, max_width=None, max_height=None):
        """Resize an image while maintaining aspect ratio"""
        if max_width is None:
            max_width = self.max_image_width
        if max_height is None:
            max_height = self.max_image_height
            
        height, width = image.shape[:2]
        
        # Only resize if the image is larger than the max dimensions
        if width > max_width or height > max_height:
            # Calculate the scaling factor to maintain aspect ratio
            scale_width = max_width / width
            scale_height = max_height / height
            scale = min(scale_width, scale_height)
            
            # Calculate new dimensions
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            # Resize the image
            resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            return resized_image
        
        # No resizing needed
        return image
    
    def encode_image_to_base64(self, image, resize=False) -> str:
        """Encode an OpenCV image to base64 string with optional resizing"""
        try:
            # Resize the image if requested to reduce token usage
            if resize:
                image = self.resize_image(image)
            
            # Convert the image from BGR to RGB (for display purposes)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Convert the image to PIL Image
            pil_image = Image.fromarray(rgb_image)
            
            # Create a byte stream to hold the image data
            img_byte_arr = io.BytesIO()
            
            # Save the image to the byte stream as JPEG with compression to reduce size
            pil_image.save(img_byte_arr, format='JPEG', quality=self.jpeg_quality)
            
            # Get the byte data
            img_byte_arr = img_byte_arr.getvalue()
            
            # Log the size of the image data
            img_size_kb = len(img_byte_arr) / 1024
            self.node.get_logger().info(f"Image size after compression: {img_size_kb:.2f} KB")
            
            # Encode the bytes as base64
            base64_encoded = base64.b64encode(img_byte_arr).decode('utf-8')
            
            # Log only the beginning of the base64 string to avoid flooding the terminal
            preview_length = 50  # Only show first 50 characters
            self.node.get_logger().debug(f"Base64 encoded image (preview): {base64_encoded[:preview_length]}...")
            
            # Return the base64 string with the proper prefix for HTML embedding
            return f"data:image/jpeg;base64,{base64_encoded}"
            
        except Exception as e:
            self.node.get_logger().error(f"Error encoding image to base64: {e}")
            return None
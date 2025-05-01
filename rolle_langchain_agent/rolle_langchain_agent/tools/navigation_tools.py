#!/usr/bin/env python3
"""Navigation tools for the LangChain agent."""

import json
import math
from typing import Dict, List, Any, Optional

class NavigationTools:
    """Class containing navigation tools for the LangChain agent."""
    
    def __init__(self, node):
        """
        Initialize navigation tools with a reference to the ROS node.
        
        Args:
            node: The ROS node instance to access position, orientation, and map data
        """
        self.node = node
        self.map_points_path = '/home/vedant/rolle_robot/src/rolle_langchain_agent/rolle_langchain_agent/map/map_points.json'
        self._load_map_points()
    
    def _load_map_points(self):
        """Load map points data from the JSON file"""
        try:
            import os
            if os.path.exists(self.map_points_path):
                with open(self.map_points_path, 'r') as f:
                    import json
                    self.map_points = json.load(f)
                self.node.get_logger().info(f"Loaded map points from {self.map_points_path}")
            else:
                self.node.get_logger().warning(f"Map points file not found at {self.map_points_path}")
                self.map_points = {}
        except Exception as e:
            self.node.get_logger().error(f"Failed to load map points: {e}")
            self.map_points = {}
    
    def get_current_room(self) -> str:
        """Determine which room the robot is currently in based on position"""
        x, y = self.node.robot_position.x, self.node.robot_position.y
        
        # Ensure map points are loaded
        if not hasattr(self, 'map_points') or not self.map_points:
            self._load_map_points()
            if not self.map_points:
                self.node.get_logger().error("Map points data could not be loaded")
                return "unknown"
        
        # Find which area contains the current point
        min_dist = float('inf')
        closest_area = None
        closest_cell = None
        
        # Check each area in the map_points
        for area_name, area_data in self.map_points.items():
            cells = area_data.get('cells', [])
            
            # Check each cell in the area
            for cell in cells:
                point = cell.get('point', [])
                
                # Skip invalid points
                if not point or len(point) < 2 or point[0] is None or point[1] is None:
                    continue
                
                # Calculate distance to this point
                cell_x, cell_y = point
                dist = math.sqrt((x - cell_x)**2 + (y - cell_y)**2)
                
                # Update closest if this is closer
                if dist < min_dist:
                    min_dist = dist
                    closest_area = area_name
                    closest_cell = cell
        
        # Check if we found a valid area
        if closest_area:
            # If it's a combined area (has underscore or plus sign)
            if '_' in closest_area:
                # For areas named like "kitchen_living_room"
                area_parts = closest_area.split('_')
                return f"intersection of {' and '.join(area_parts)}"
            else:
                return closest_area
        
        # If we didn't find a match, return unknown
        self.node.get_logger().error("Could not determine current room from map points")
        return "unknown"

    
    def get_robot_position(self, input_str=None) -> str:
        """Return the current robot position and the room it's in"""
        current_room = self.get_current_room()
        
        result = {
            'x': self.node.robot_position.x,
            'y': self.node.robot_position.y,
            'orientation': self.node.robot_orientation,
            'room': current_room,
            'message': f"The robot is currently in the {current_room}"
        }
            
        return json.dumps(result)
    
    def get_area_points(self, area_name: str) -> str:
        """
        Get points with attributes for a specific area.
        
        Args:
            area_name: The name of the area to get points for (e.g., kitchen, bedroom)
            
        Returns:
            JSON string containing points with attributes in the specified area
        """
        # Clean up the area name by stripping whitespace and converting to lowercase
        area_name = area_name.strip().lower()
        
        # Check if the map points are loaded
        if not self.map_points:
            self._load_map_points()
            if not self.map_points:
                return json.dumps({
                    "error": "Map points data could not be loaded"
                })
        
        # Check if the area exists in our map data
        if area_name not in self.map_points:
            available_areas = list(self.map_points.keys())
            # Try to find a match with fuzzy matching (in case of typos or slight variations)
            closest_matches = [a for a in available_areas if area_name in a or a in area_name]
            if closest_matches:
                area_name = closest_matches[0]
                self.node.get_logger().info(f"Matched '{area_name}' to '{area_name}'")
            else:
                return json.dumps({
                    "error": f"Area '{area_name}' not found. Available areas are: {', '.join(available_areas)}"
                })
        
        # Get all cells for the specified area
        area_data = self.map_points[area_name]
        cells = area_data.get('cells', [])
        
        # Filter cells to only include those with non-empty attributes
        points_with_attributes = []
        for cell in cells:
            if cell.get('attribute') and len(cell['attribute']) > 0:
                points_with_attributes.append({
                    'id': cell.get('id'),
                    'point': cell.get('point'),
                    'attribute': cell.get('attribute')
                })
        
        # Check if we found any points with attributes
        if not points_with_attributes:
            return json.dumps({
                "area": area_name,
                "message": f"No points with attributes found in {area_name}",
                "points": []
            })
        
        # Return the points with attributes
        return json.dumps({
            "area": area_name,
            "message": f"Found {len(points_with_attributes)} points with attributes in {area_name}",
            "points": points_with_attributes
        })
        
    def get_entry_point(self, rooms_input: str) -> str:
        """Get the entry point between two rooms with attributes"""
        try:
            # Parse input: "room1,room2"
            rooms = rooms_input.split(',')
            if len(rooms) != 2:
                return "Error: Input should be two room names separated by comma"
            
            # Clean up room names by stripping whitespace and converting to lowercase
            room1 = rooms[0].strip().lower()
            room2 = rooms[1].strip().lower()
            
            # Log the input for debugging
            self.node.get_logger().info(f"Finding entry point between: '{room1}' and '{room2}'")
            
            # Check if map points are loaded
            if not self.map_points:
                self._load_map_points()
                if not self.map_points:
                    return json.dumps({
                        "error": "Map points data could not be loaded"
                    })
            
            # Construct the combined area name (both combinations)
            combined_name1 = f"{room1}_{room2}"
            combined_name2 = f"{room2}_{room1}"
            
            # Check if either combination exists in our map data
            area_name = None
            if combined_name1 in self.map_points:
                area_name = combined_name1
            elif combined_name2 in self.map_points:
                area_name = combined_name2
            
            if area_name:
                # Get points with attributes from this combined area
                area_data = self.map_points[area_name]
                cells = area_data.get('cells', [])
                
                # Filter cells to only include those with non-empty attributes
                points_with_attributes = []
                for cell in cells:
                    if cell.get('attribute') and len(cell['attribute']) > 0:
                        points_with_attributes.append({
                            'id': cell.get('id'),
                            'point': cell.get('point'),
                            'attribute': cell.get('attribute')
                        })
                
                # Check if we found any points with attributes
                if not points_with_attributes:
                    return json.dumps({
                        "area": area_name,
                        "message": f"No entry points with attributes found between {room1} and {room2}",
                        "points": []
                    })
                
                # Return the points with attributes
                return json.dumps({
                    "area": area_name,
                    "message": f"Found {len(points_with_attributes)} entry points with attributes between {room1} and {room2}",
                    "points": points_with_attributes
                })
            
            # If we didn't find a combined area in map_points, fall back to the polygon method
            # Check if the room names exist in our mapping
            available_rooms = list(self.node.room_to_color.keys())
            
            if room1 not in self.node.room_to_color:
                # Try to find a match with fuzzy matching (in case of typos or slight variations)
                closest_matches = [r for r in available_rooms if room1 in r or r in room1]
                if closest_matches:
                    room1 = closest_matches[0]
                    self.node.get_logger().info(f"Matched '{rooms[0]}' to '{room1}'")
                else:
                    return json.dumps({
                        "error": f"Room '{room1}' not recognized. Available rooms are: {', '.join(available_rooms)}"
                    })
            
            if room2 not in self.node.room_to_color:
                # Try to find a match with fuzzy matching
                closest_matches = [r for r in available_rooms if room2 in r or r in room2]
                if closest_matches:
                    room2 = closest_matches[0]
                    self.node.get_logger().info(f"Matched '{rooms[1]}' to '{room2}'")
                else:
                    return json.dumps({
                        "error": f"Room '{room2}' not recognized. Available rooms are: {', '.join(available_areas)}"
                    })
            
            # Get the colors for both rooms
            color1 = self.node.room_to_color[room1]
            color2 = self.node.room_to_color[room2]
            
            self.node.get_logger().info(f"Looking for intersection between {room1} ({color1}) and {room2} ({color2})")
            
            combined_color1 = f"{color1}+{color2}"
            combined_color2 = f"{color2}+{color1}"
            
            # Check if we have this combined area in map_points
            for area_key, area_value in self.map_points.items():
                if area_value.get('color') == combined_color1 or area_value.get('color') == combined_color2:
                    # Get only points with attributes
                    points_with_attributes = []
                    for cell in area_value.get('cells', []):
                        if cell.get('attribute') and len(cell['attribute']) > 0:
                            points_with_attributes.append({
                                'id': cell.get('id'),
                                'point': cell.get('point'),
                                'attribute': cell.get('attribute')
                            })
                    
                    if points_with_attributes:
                        return json.dumps({
                            "area": area_key,
                            "message": f"Found {len(points_with_attributes)} entry points with attributes between {room1} and {room2}",
                            "points": points_with_attributes
                        })
            
            # If we still haven't found anything, return no points found
            return json.dumps({
                "message": f"No entry points with attributes found between {room1} and {room2}",
                "points": []
            })
            
        except Exception as e:
            self.node.get_logger().error(f"Error processing entry point request: {e}")
            return json.dumps({
                "error": f"Error processing entry point request: {e}"
            })
    
    def send_speech(self, speech_text: str) -> str:
        """
        Send speech to be spoken by the robot and track it in conversation memory.
        
        Args:
            speech_text: The text to be spoken
            
        Returns:
            A JSON string containing the status of the speech request and conversation history
        """
        import time
        
        # Log the speech request
        self.node.get_logger().info(f"Speech requested: '{speech_text}'")

        # Wait for 2 seconds (simulating speech being sent and processed)
        time.sleep(2)
        
        # Create speech output object
        speech_output = {
            "status": "success",
            "message": f"Speech sent: '{speech_text}'",
            "speech": {
                "text": speech_text
}
        }
        
        # Add conversation to memory if we have the conversation memory system initialized
        # We'll use the input that generated this speech as user input
        # This would typically be stored elsewhere or passed to this method
        if hasattr(self.node, "conversation_memory"):
            # Get the last command or use a generic one
            user_input = getattr(self.node, "last_command", "User request")
            self.node.conversation_memory.add_conversation(user_input, {"speech": speech_text})
            
            # Add conversation history to the output
            speech_output["memory"] = {
                "current_summary": self.node.conversation_memory.get_latest_summary(),
                "conversation_history": self.node.conversation_memory.get_all_summaries()
            }
        
        # Return a success response with the sent speech and memory information
        return json.dumps(speech_output)
    
    def calculate_point_in_direction(self, direction_distance: str) -> str:
        """Calculate target point in a specific direction and distance"""
        try:
            # Parse input: "direction,distance"
            parts = direction_distance.split(',')
            if len(parts) != 2:
                return "Error: Input should be 'direction,distance'"
                
            direction = parts[0].strip().lower()
            try:
                distance = float(parts[1].strip())
            except ValueError:
                return f"Error: Distance must be a number, got: {parts[1]}"
            
            # Get current position and orientation
            current_x = self.node.robot_position.x
            current_y = self.node.robot_position.y
            current_orientation = self.node.robot_orientation  # In radians
            
            # Calculate target position based on direction and distance
            if direction == "forward":
                # Forward means in the direction of current orientation
                target_x = current_x + distance * math.cos(current_orientation)
                target_y = current_y + distance * math.sin(current_orientation)
            elif direction == "backward":
                # Backward means opposite to current orientation
                target_x = current_x - distance * math.cos(current_orientation)
                target_y = current_y - distance * math.sin(current_orientation)
            elif direction == "left":
                # Left means 90 degrees counterclockwise from current orientation
                target_x = current_x + distance * math.cos(current_orientation + math.pi/2)
                target_y = current_y + distance * math.sin(current_orientation + math.pi/2)
            elif direction == "right":
                # Right means 90 degrees clockwise from current orientation
                target_x = current_x + distance * math.cos(current_orientation - math.pi/2)
                target_y = current_y + distance * math.sin(current_orientation - math.pi/2)
            else:
                return f"Error: Direction must be 'forward', 'backward', 'left', or 'right', got: {direction}"
            
            return json.dumps({
                'x': target_x,
                'y': target_y,
                'message': f"Calculated point {distance} meters {direction} from current position"
            })
            
        except Exception as e:
            return f"Error calculating point in direction: {e}"

#!/usr/bin/env python3
"""
Unified navigation agent using LangChain for ROS2.

This is the main entry point for the navigation agent which combines
voice commands, LLM processing, and robot navigation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Vector3
from rolle_langchain_agent.msg import NavGoalWithSpeech
import json
import yaml
import os
import numpy as np
import math
import re
from typing import Dict, List, Any, Optional

# LangChain imports
from langchain.agents import Tool
from langchain_openai import ChatOpenAI

# Import our modular components
from rolle_langchain_agent.tools.navigation_tools import NavigationTools
from rolle_langchain_agent.utils.image_utils import ImageUtils
from rolle_langchain_agent.utils.agent_config import AgentConfig, ConversationSummaryMemory

class UnifiedNavAgent(Node):
    def __init__(self):
        super().__init__('unified_nav_agent')
        
        # Initialize conversation memory
        self.conversation_memory = ConversationSummaryMemory(max_conversations=5)
        
        # Load environment variables for API keys
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError("OpenAI API key not found")
        
        # Declare parameters
        self.declare_parameter('polygons_file', '/home/vedant/rolle_robot/src/rolle_langchain_agent/rolle_langchain_agent/map/map_points.json')
        
        polygons_file = self.get_parameter('polygons_file').get_parameter_value().string_value
        
        # Load map data
        self.load_polygons_data(polygons_file)
        
        # Initialize robot state
        self.robot_position = Point()
        self.robot_orientation = 0.0  # In radians
        
        # Initialize room-color mapping
        self.room_to_color = {
            "kitchen": "yellow",
            "dining room": "blue",
            "dining hall": "blue",
            "hallway": "green",
            "bedroom": "red",
            "living room": "blue"  # Assuming the living room is also blue
        }
        
        # Reverse mapping for color to room name
        self.color_to_room = {v: k for k, v in self.room_to_color.items()}
        
        # Set up subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # Set up command subscription
        self.command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10)
        
        
        # Create custom message publisher with speech
        self.nav_speech_publisher = self.create_publisher(
            NavGoalWithSpeech,
            'rolle_agent/nav_goal_with_speech',
            10
        )
        
        # Initialize utility modules
        self.nav_tools = NavigationTools(self)
        self.image_utils = ImageUtils(self)
        
        # Configure LLM
        self.llm = ChatOpenAI(
            model="gpt-4o",
            # temperature=0,
            api_key=self.openai_api_key
        )
        
        # Set up LangChain agent
        self.agent_config = AgentConfig(self)
        self.setup_langchain_agent()
        
        self.get_logger().info('Unified navigation agent initialized successfully!')
    
    def load_polygons_data(self, polygons_file):
        """Load the polygons data from JSON file"""
        try:
            with open(polygons_file, 'r') as f:
                self.polygons = json.load(f)
            self.get_logger().info(f"Loaded {len(self.polygons)} polygons from {polygons_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load polygons data: {e}")
            self.polygons = []
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        self.robot_position = msg.pose.pose.position
        
        # Extract yaw (orientation around z-axis) from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_orientation = math.atan2(t3, t4)
        
        self.get_logger().debug(f"Robot position updated: ({self.robot_position.x}, {self.robot_position.y}), orientation: {self.robot_orientation}")
    
    def command_callback(self, msg):
        """Process voice command and publish navigation goal"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Store the command for the conversation memory system
        self.last_command = command
        
        try:
            # Process the command through our navigation agent
            result = self.process_command(command)
            
            if result:
                self.get_logger().info(f'Target coordinates: {result}')

                    
            else:
                self.get_logger().error('Failed to process command')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            # Record error in conversation memory
            if hasattr(self, "conversation_memory"):
                self.conversation_memory.add_conversation(command, {"error": str(e)})
    
    def get_map_with_robot_position(self):
        """Generate a map visualization with the robot position marked"""
        return self.image_utils.get_map_with_robot_position()
            
    def process_command(self, command):
        """Process a voice command through the LangChain agent"""
        try:
            # Run the command through the agent with the proper input key
            result = self.agent.run({"input": command})
            self.get_logger().info(f"Agent response: {result}")
            
            # Try to parse the result as JSON
            try:
                parsed_result = None
                
                # First check the type of result returned
                if isinstance(result, dict):
                    # Result is already a dictionary
                    parsed_result = result
                elif isinstance(result, str):
                    # Try to extract a JSON object from the response string
                    json_match = re.search(r'(\{.*\})', result, re.DOTALL)
                    if json_match:
                        json_str = json_match.group(1)
                        try:
                            parsed_result = json.loads(json_str)
                        except json.JSONDecodeError:
                            self.get_logger().warning(f"Extracted JSON is invalid: {json_str}")
                            # Try to parse the entire response as JSON
                            parsed_result = json.loads(result)  
                    else:
                        # If no JSON pattern found, try the entire string
                        parsed_result = json.loads(result)
                else:
                    # Unknown result type
                    self.get_logger().error(f"Unexpected result type: {type(result)}")
                    return [0.0, 0.0, 0.0]
                
                # Check if we have a proper nav2_goal structure
                if parsed_result and "nav2_goal" in parsed_result:
                    nav_goal = parsed_result["nav2_goal"]
                    # Convert values to float to handle both string and numeric types
                    x = float(nav_goal.get("x", 0.0))
                    y = float(nav_goal.get("y", 0.0))
                    yaw = float(nav_goal.get("yaw", 0.0))
                    
                    # Extract speech text
                    speech_obj = parsed_result.get("speech", {})
                    # Handle both formats: {"text": "..."} and "..."
                    if isinstance(speech_obj, dict):
                        speech_text = speech_obj.get("text", "")
                    elif isinstance(speech_obj, str):
                        speech_text = speech_obj
                    else:
                        speech_text = str(speech_obj)
                    
                    # Add conversation summary to the output if using memory
                    if hasattr(self, "conversation_memory"):
                        memory_output = {
                            "speech": speech_text
                        }
                        self.conversation_memory.add_conversation(command, memory_output)
                        
                        # Add summaries to the result
                        parsed_result["memory"] = {
                            "current_summary": self.conversation_memory.get_latest_summary(),
                            "conversation_history": self.conversation_memory.get_all_summaries()
                        }
                        
                        self.get_logger().info(f"Memory added to response: {parsed_result['memory']}")
                    
                    # Publish to the custom message topic
                    self.publish_nav_goal_with_speech(x, y, yaw, speech_text)
                    
                    # Return coordinates as a list [x, y, yaw]
                    return [x, y, yaw]
                else:
                    self.get_logger().error(f"Missing nav2_goal in agent response: {parsed_result}")
                    return [0.0, 0.0, 0.0]  # Return default values on failure
                    
            except json.JSONDecodeError as e:
                # Fallback to regex parsing for non-JSON responses
                self.get_logger().warning(f"Response is not valid JSON ({e}), using regex fallback")
                
                # Extract coordinates from the result using regex
                coords_match = re.search(r'(-?\d+\.?\d*),\s*(-?\d+\.?\d*)', result)
                if coords_match:
                    x = float(coords_match.group(1))
                    y = float(coords_match.group(2))
                    return [x, y, 0.0]  # Default yaw to 0.0
                else:
                    self.get_logger().error("No coordinates found in agent response")
                    return [0.0, 0.0, 0.0]  # Return default values on failure
                
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return [0.0, 0.0, 0.0]  # Return default values on failure
    
    def publish_nav_goal_with_speech(self, x, y, yaw=0.0, speech_text=""):
        """Publish a goal pose with speech for the robot"""
        # Create the custom message
        nav_speech_msg = NavGoalWithSpeech()
        
        # Set the goal pose
        nav_speech_msg.goal = Vector3()
        nav_speech_msg.goal.header.stamp = self.get_clock().now().to_msg()
        nav_speech_msg.goal.header.frame_id = "map"
        nav_speech_msg.goal.pose.position.x = float(x)
        nav_speech_msg.goal.pose.position.y = float(y)
        nav_speech_msg.goal.pose.position.z = 0.0
        
        # Set orientation based on yaw (convert to quaternion if needed)
        nav_speech_msg.goal.pose.orientation.x = 0.0
        nav_speech_msg.goal.pose.orientation.y = 0.0
        nav_speech_msg.goal.pose.orientation.z = 0.0
        nav_speech_msg.goal.pose.orientation.w = 1.0
        
        # Set the speech text
        nav_speech_msg.speech_text = speech_text
        
        # Publish the message
        self.nav_speech_publisher.publish(nav_speech_msg)
        self.get_logger().info(f'Published nav goal with speech: x={x}, y={y}, speech="{speech_text}"')
        
    def setup_langchain_agent(self):
        """Set up the LangChain agent with tools"""
        # Define tools
        """
        Tools required:
        - get points of interest from an area with attritutes decribing each point ++
        - get orientation for goal pose of the robot
        - get entry point between two areas ++ (put orientation output in entry points)
        - get robot position ++
        - calculate a point in a specific direction from the robot's current position ++
        - implement tool to send speech ++
        - output json of navigation
        - closest point to a distance
        """
        
        tools = [
            Tool(
                name="get_area_points",
                func=self.nav_tools.get_area_points,
                description="Gets the points inside a specific area with attributes. Attributes decribe the importance of the point on the map. " \
                            "Input should be the area name like kitchen, bedroom, etc."
            ),
            Tool(
                name="get_entry_point",
                func=self.nav_tools.get_entry_point,
                description="Gets the entry point and desired orientation of the robot between two areas with attribute. " \
                "Input should be two area names separated by a comma, e.g., 'hallway,kitchen'"
            ),
            Tool(
                name="get_robot_position",
                func=self.nav_tools.get_robot_position,
                description="Gets the current position of the robot with orientation along with which room in the house it is in."
            ),
            # Tool(
            #     name="calculate_point_in_direction",
            #     func=self.nav_tools.calculate_point_in_direction,
            #     description="Gives a command to the robot to move in a specific direction or rotate from the robot's current position." \
            #     "Input should be 'direction,distance' e.g., 'forward,5' for 5 meters forward. or 'rotate, 90' for 90 degrees rotation."
            # ),
            Tool(
                name="send_speech",
                func=self.nav_tools.send_speech,
                description="Speech to talk to the user. Use this to communicate information to the user verbally." \
                "Input should be the text that the robot should speak."
            ),
        ]
        
        # Create the agent using our agent configuration class
        self.agent = self.agent_config.setup_agent(tools)
        
def main(args=None):
    rclpy.init(args=args)
    node = UnifiedNavAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

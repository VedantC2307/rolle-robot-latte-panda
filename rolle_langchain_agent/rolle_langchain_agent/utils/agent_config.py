#!/usr/bin/env python3
"""Configuration for the LangChain agent."""

from langchain.agents import Tool, AgentExecutor, ZeroShotAgent
from langchain.memory import ConversationBufferMemory
from langchain_openai import ChatOpenAI
from langchain.chains import LLMChain
from typing import List, Dict, Any
from collections import deque
import json

class ConversationSummaryMemory:
    """A custom memory class to track the last 5 conversations and generate summaries."""
    
    def __init__(self, max_conversations=5):
        """Initialize with a deque to store the last n conversations."""
        self.conversations = deque(maxlen=max_conversations)
        
    def add_conversation(self, user_input: str, agent_output: Dict[str, Any]) -> None:
        """Add a new conversation to memory."""
        # Parse agent output if it's in string format
        if isinstance(agent_output, str):
            try:
                # Try to parse as JSON if it's a string
                parsed_output = json.loads(agent_output)
            except json.JSONDecodeError:
                # If not valid JSON, use as is
                parsed_output = {"text": agent_output}
        else:
            parsed_output = agent_output
            
        # Generate a simple summary
        summary = f"User asked: {user_input[:50]}..." if len(user_input) > 50 else f"User asked: {user_input}"
        
        # Add conversation to memory
        self.conversations.append({
            "user_input": user_input,
            "agent_output": parsed_output,
            "summary": summary
        })
    
    def get_conversation_history(self) -> List[Dict[str, Any]]:
        """Return the conversation history."""
        return list(self.conversations)
    
    def get_latest_summary(self) -> str:
        """Get the summary of the latest conversation."""
        if not self.conversations:
            return "No previous conversations."
        return self.conversations[-1]["summary"]
    
    def get_all_summaries(self) -> List[str]:
        """Get summaries of all stored conversations."""
        return [conv["summary"] for conv in self.conversations]

class AgentConfig:
    """Class for configuring the LangChain agent."""
    
    def __init__(self, node):
        """
        Initialize agent configuration with a reference to the ROS node.
        
        Args:
            node: The ROS node instance to access tools and API keys
        """
        self.node = node
        
    def setup_agent(self, tools: List[Tool]) -> AgentExecutor:
        """
        Set up the LangChain agent with provided tools.
        
        Args:
            tools: List of LangChain tools to provide to the agent
            
        Returns:
            The configured AgentExecutor
        """
        # Define prompt
        prefix = """
                You are Homebot, a friendly intelligent assistant for a differential-drive home robot. Your job is to interpret user navigation commands, call the right tools, speak naturally, and then output only the final x,y (and optional orientation) coordinates for Nav2.

                House layout (with map colors):  
                • kitchen = yellow  
                • living room / dining hall = blue  
                • hallway = green  
                • Vedant's bedroom = red  

                Intersection Areas (where rooms connect):  
                • bedroom ↔ hallway  
                • hallway ↔ living room  
                • living room ↔ kitchen  

                Always follow this exact workflow when a user asks you to navigate:

                1. LISTEN & UNDERSTAND  
                • Parse the user's request: Is it "go to [room]" or "move [direction] [distance]"?  
                • Decide the intent (named location vs. relative movement).

                2. GET CURRENT STATE  
                • Call `get_robot_position()` to obtain your current x,y, orientation, and room.

                3. SELECT & CALL NAV TOOLS  
                • **Named location:**  
                    – Identify which area intersects your target room (e.g., for "kitchen" that's "living room↔kitchen").  
                    – Call `get_entry_point("target_room, intersecting_area")` to retrieve the exact doorway intersection point.  
                    – If already inside the target room, communicate to the user that you're already there and nav2_goal should be 0.0 as no movement is required.
                • **Relative move:**  
                    – For commands like "move forward 2 meters" or "go left 1 meter", include a nav2_movement field in your output with format "direction,distance" (e.g. "forward,2.0") else it should be null,null.

                4. SPEAK YOUR PLAN  
                • Before moving, call `send_speech()` with a friendly summary of what you'll do.  
                    e.g. "Sure thing—I'm heading to the kitchen now!"

                5. OUTPUT GOAL COORDINATES  
                • Finally, return only a JSON with `nav2_goal`, `nav2_movement`, and `speech` fields.
                • The JSON format must follow this exact structure:
                {
                  "nav2_goal": {
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0
                  },
                  "speech": {
                    "text": "Your speech goes here"
                  },
                  "nav2_movement": "direction,distance"  // only for relative movement commands
                }
                
                • For navigation to a point, only include nav2_goal.
                • For directional movement (forward, backward, left, right), include both nav2_goal AND nav2_movement.
                • Do not include path details or planning instructions—Nav2 will handle that.

                If no navigation is required by the robot, then x, y & yaw should be 0.0, 0.0 & 0.0 respectively.
                

                Personality notes:  
                – You are warm, helpful, and concise.  
                – Use natural phrases ("Alright!", "Got it!", "On my way!") when speaking.  
                – Keep the user informed but stay focused on delivering the correct coordinates.
                """
        
        suffix = """Begin!

Question: {input}
{agent_scratchpad}

"""
        
        # Initialize agent
        prompt = ZeroShotAgent.create_prompt(
            tools, 
            prefix=prefix, 
            suffix=suffix,
            input_variables=["input", "agent_scratchpad"]
        )
        
        memory = ConversationBufferMemory(memory_key="chat_history")
        
        # Update the LLM with increased token limits
        if not hasattr(self.node, 'llm'):
            self.node.llm = ChatOpenAI(
                model="gpt-4o",
                temperature=0.1,
                api_key=self.node.openai_api_key,
                max_tokens=4096,  # Increase output token limit
                model_kwargs={
                    "max_tokens": 4096  # Ensure model-specific max tokens is set
                }
            )
        
        llm_chain = LLMChain(llm=self.node.llm, prompt=prompt)
        
        return AgentExecutor.from_agent_and_tools(
            agent=ZeroShotAgent(llm_chain=llm_chain, tools=tools, verbose=True),
            tools=tools,
            verbose=True,
            memory=memory,
            handle_parsing_errors=True,  # Add error handling for parsing issues
            max_iterations=5,  # Limit the number of iterations to avoid excessive API calls
            max_execution_time=30  # Set a maximum execution time in seconds
        )
import os
from typing import Dict, Any, Optional, List
import json
import base64
from io import BytesIO
from PIL import Image

from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough
from langchain_core.tools import Tool
from langchain_openai import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain_community.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings
from langchain_core.documents import Document

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

# Set up memory
memory = ConversationBufferMemory(return_messages=True)

# Set up vector store for long-term memory
embeddings = OpenAIEmbeddings()
# Initialize with sample documents - in production, this would be pre-populated
sample_docs = [
    Document(page_content="Robot successfully navigated to the kitchen at coordinates (2.5, 3.0)", metadata={"timestamp": "2025-04-20T10:30:00"}),
    Document(page_content="User requested to navigate to the living room at coordinates (5.0, 4.2)", metadata={"timestamp": "2025-04-20T11:15:00"}),
    Document(page_content="Robot encountered obstacle at coordinates (3.2, 2.8)", metadata={"timestamp": "2025-04-20T12:00:00"}),
]
vector_store = Chroma.from_documents(documents=sample_docs, embedding=embeddings)
retriever = vector_store.as_retriever()

# Set up the LLM
model = ChatOpenAI(model="gpt-4o-mini", temperature=0.2)

# Define tools
def memory_retriever(query: str) -> str:
    """Retrieve relevant information from long-term memory based on query."""
    docs = retriever.get_relevant_documents(query)
    if not docs:
        return "No relevant information found in memory."
    
    results = []
    for doc in docs:
        results.append(f"[{doc.metadata.get('timestamp', 'Unknown time')}] {doc.page_content}")
    
    return "\n".join(results)

def nav_goal_publisher(x: float, y: float) -> str:
    """Publish a navigation goal to move the robot."""
    # In a real implementation, this would publish a ROS2 PoseStamped message
    print(f"Publishing navigation goal: x={x}, y={y}")
    # Add the navigation event to memory
    vector_store.add_documents([
        Document(
            page_content=f"Robot navigated to coordinates ({x}, {y})",
            metadata={"timestamp": "2025-04-21T14:30:00"}  # In production, use actual timestamp
        )
    ])
    return f"Navigation goal sent: x={x}, y={y}"

def tts_publisher(text: str) -> str:
    """Send text to be spoken by the robot."""
    # In a real implementation, this would publish to a ROS2 topic
    print(f"Robot says: {text}")
    # Add the speech event to memory
    vector_store.add_documents([
        Document(
            page_content=f"Robot said: '{text}'",
            metadata={"timestamp": "2025-04-21T14:30:00"}  # In production, use actual timestamp
        )
    ])
    return f"Speech sent: '{text}'"

# Define the tools
tools = [
    Tool(
        name="memory_retriever",
        func=memory_retriever,
        description="Query long-term memory for relevant past events"
    ),
    Tool(
        name="nav_goal_publisher",
        func=lambda coords: nav_goal_publisher(coords["x"], coords["y"]),
        description="Publish a navigation goal with x and y coordinates"
    ),
    Tool(
        name="tts_publisher",
        func=tts_publisher,
        description="Send text to be spoken by the robot"
    )
]

# Encode images to base64 for inclusion in the prompt
def encode_image_to_base64(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

# Process the agent request
def process_agent_request(map_image_path: str, camera_image_path: str, user_prompt: str):
    # Encode images
    map_base64 = encode_image_to_base64(map_image_path)
    camera_base64 = encode_image_to_base64(camera_image_path)
    
    # Get recent memory context
    memory_messages = memory.load_memory_variables({})["messages"]
    memory_context = "\n".join([f"{msg.type}: {msg.content}" for msg in memory_messages[-5:]])
    
    # Query long-term memory for relevant information
    relevant_memory = memory_retriever(user_prompt)
    
    # Construct the system message
    system_prompt = """
    You are an AI assistant that helps control a robot. You'll receive:
    1. A map image with the robot's current position
    2. A camera image showing what the robot sees
    3. The user's request
    4. Recent conversation context
    5. Relevant past events from memory
    
    Based on this information, generate a JSON response with:
    - nav_goal: Coordinates to navigate to (null if no navigation needed)
    - speech: Text for the robot to say
    - action_flags: Indicate if the robot is moving or talking
    
    Example response format:
    ```json
    {
      "nav_goal": {"x": 4.5, "y": 2.3},
      "speech": "I'm heading to the kitchen now.",
      "action_flags": {"moving": true, "talking": true}
    }
    ```
    
    If no navigation is needed:
    ```json
    {
      "nav_goal": null,
      "speech": "I'll stay right here.",
      "action_flags": {"moving": false, "talking": true}
    }
    ```
    """
    
    # Construct the human message with all inputs
    human_message = f"""
    USER REQUEST: {user_prompt}
    
    RECENT CONVERSATION:
    {memory_context}
    
    RELEVANT MEMORY:
    {relevant_memory}
    
    MAP IMAGE (showing robot position):
    [Image encoded in base64, not displaying it here for brevity]
    
    CAMERA IMAGE (robot's view):
    [Image encoded in base64, not displaying it here for brevity]
    
    Please analyze the situation and respond with appropriate navigation and speech actions.
    """
    
    # Add the message to memory
    memory.save_context({"input": user_prompt}, {"output": ""})
    
    # Send to LLM for processing
    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=human_message)
    ]
    
    response = model.invoke(messages)
    
    # Extract JSON from the response
    try:
        response_text = response.content
        # Extract JSON if it's within code blocks
        if "```json" in response_text:
            json_text = response_text.split("```json")[1].split("```")[0].strip()
        elif "```" in response_text:
            json_text = response_text.split("```")[1].strip()
        else:
            json_text = response_text
            
        result = json.loads(json_text)
        
        # Execute actions based on the response
        speech_result = None
        nav_result = None
        
        if result.get("speech"):
            speech_result = tts_publisher(result["speech"])
            
        if result.get("nav_goal"):
            nav_coords = result["nav_goal"]
            nav_result = nav_goal_publisher(nav_coords["x"], nav_coords["y"])
        
        # Save the model's response to memory
        memory.save_context({"input": ""}, {"output": response.content})
        
        return {
            "processed_result": result,
            "speech_result": speech_result,
            "nav_result": nav_result
        }
        
    except json.JSONDecodeError:
        return {"error": "Failed to parse JSON response", "raw_response": response.content}
    except Exception as e:
        return {"error": str(e), "raw_response": response.content}

# Example usage (commented out since we don't have actual images)
"""
result = process_agent_request(
    map_image_path="path/to/map_image.jpg", 
    camera_image_path="path/to/camera_image.jpg",
    user_prompt="Can you take me to the kitchen?"
)
print(json.dumps(result, indent=2))
"""

# For a real implementation, this would be integrated with ROS2 nodes
# that subscribe to image topics and publish to navigation and speech topics

# Sample test function for demonstration
def test_agent_with_mock_data():
    # Create temporary mock image files
    mock_map = Image.new('RGB', (300, 300), color='white')
    mock_camera = Image.new('RGB', (640, 480), color='blue')
    
    map_path = "mock_map.jpg"
    camera_path = "mock_camera.jpg"
    
    mock_map.save(map_path)
    mock_camera.save(camera_path)
    
    try:
        # Test with sample prompt
        result = process_agent_request(
            map_image_path=map_path,
            camera_image_path=camera_path,
            user_prompt="Can you take me to the kitchen please?"
        )
        print(json.dumps(result, indent=2))
    finally:
        # Clean up temporary files
        os.remove(map_path)
        os.remove(camera_path)

if __name__ == "__main__":
    test_agent_with_mock_data()
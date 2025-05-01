#!/usr/bin/env python3
"""
Publish an entire audio file (WAV or MP3) as a single ROS 2 message.
"""

import pathlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

class AudioFilePublisher(Node):
    def __init__(self):
        super().__init__("audio_file_publisher")

        # Parameters
        self.declare_parameter("audio_file", "/home/vedant/rolle_robot/src/audio_testing_wav/audio_testing_wav/full_conversation.wav")
        self.declare_parameter("topic", "tts_audio")

        fpath = pathlib.Path(
            self.get_parameter("audio_file").value).expanduser()
        topic = self.get_parameter("topic").value

        if not fpath.is_file():
            self.get_logger().fatal(f"{fpath} not found"); rclpy.shutdown(); return

        # Detect file type from extension
        file_extension = fpath.suffix.lower()
        
        if file_extension not in ['.wav', '.mp3']:
            self.get_logger().fatal(f"Unsupported file type: {file_extension}. Only .wav and .mp3 are supported."); 
            rclpy.shutdown(); 
            return

        # ---- read the whole file ----
        audio_bytes = fpath.read_bytes()
        self.get_logger().info(f"Loaded {len(audio_bytes):,} bytes from {fpath.name} ({file_extension[1:]} format)")

        # Prepare main audio data message
        msg = UInt8MultiArray()
        msg.data = bytearray(audio_bytes)
        
        # Create metadata publisher
        metadata_pub = self.create_publisher(String, f"{topic}_metadata", 10)
        
        # Create and publish metadata message
        metadata_msg = String()
        metadata_msg.data = f'{{"format": "{file_extension[1:]}", "filename": "{fpath.name}", "size": {len(audio_bytes)}}}'
        metadata_pub.publish(metadata_msg)
        self.get_logger().info(f"Published metadata on /{topic}_metadata")

        # ---- publisher, default RELIABLE QoS is fine ----
        pub = self.create_publisher(UInt8MultiArray, topic, 10)
        pub.publish(msg)
        self.get_logger().info(f"Published {file_extension[1:]} data on /{topic}; shutting down")
        rclpy.shutdown()

def main():
    rclpy.init()
    AudioFilePublisher()
    rclpy.spin_once(rclpy.get_global_executor())    # node shuts itself down

if __name__ == "__main__":
    main()

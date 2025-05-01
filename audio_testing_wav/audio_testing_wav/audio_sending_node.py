#!/usr/bin/env python3
"""
ros2 node: read a .wav file, preserve the header, and publish on a topic
"""
import wave
import pathlib
import rclpy
import struct
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

class WavToUint8Node(Node):
    def __init__(self):
        super().__init__('wav_to_uint8_node')
        # Parameters
        self.declare_parameter('wav_file', '/home/vedant/rolle_robot/src/audio_testing_wav/audio_testing_wav/full_conversation.wav')
        self.declare_parameter('topic', '/mobile_sensor/wav_bytes')
        self.declare_parameter('publish_rate', 0.2)  # Hz
        
        wav_path_str = self.get_parameter('wav_file').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.wav_path = pathlib.Path(wav_path_str)
        if not self.wav_path.is_file():
            self.get_logger().error(f"WAV file not found: {wav_path_str}")
            rclpy.shutdown()
            return
        
        # Read .wav file fully WITH header
        with open(str(self.wav_path), 'rb') as f:
            self.wav_data = f.read()
        
        # Extract WAV properties for logging if possible
        channels = 0
        sample_width = 0
        framerate = 0
        n_frames = 0
        
        try:
            wf = wave.open(str(self.wav_path), 'rb')
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            framerate = wf.getframerate()
            n_frames = wf.getnframes()
            wf.close()
            self.get_logger().info(f"WAV properties: {channels} channels, {sample_width} bytes/sample, {framerate} Hz")
        except wave.Error as e:
            self.get_logger().warn(f"Cannot parse WAV properties using standard wave module: {e}")
            self.get_logger().info("Continuing with raw data only")
            # Try to extract basic info from the header directly for A-law encoded files
            try:
                if len(self.wav_data) >= 44:  # Standard WAV header size
                    # Extract format tag from WAV header
                    format_tag = struct.unpack_from('<H', self.wav_data, 20)[0]
                    channels = struct.unpack_from('<H', self.wav_data, 22)[0]
                    framerate = struct.unpack_from('<I', self.wav_data, 24)[0]
                    self.get_logger().info(f"Raw WAV header info - Format tag: {format_tag}, Channels: {channels}, Sample rate: {framerate} Hz")
            except Exception as header_e:
                self.get_logger().warn(f"Failed to extract header info manually: {header_e}")
        
        self.get_logger().info(f"Loaded {len(self.wav_data)} bytes from {self.wav_path.name}")
        
        # Publisher for WAV data (including header)
        self.publisher_ = self.create_publisher(
            UInt8MultiArray,
            topic,
            10
        )
        
        # Also publish WAV info for debugging
        self.info_publisher = self.create_publisher(
            String,
            f"{topic}_info",
            10
        )
        
        # Timer to publish at fixed rate
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 1.0
        self.timer = self.create_timer(timer_period, self.publish_callback)
    
    def publish_callback(self):
        # Convert bytes to uint8 list
        data_uint8 = list(self.wav_data)
        
        # Publish WAV data with header
        msg = UInt8MultiArray(data=data_uint8)
        self.publisher_.publish(msg)
        
        # Publish info
        info_msg = String(data=f"WAV file: {self.wav_path.name}, Size: {len(data_uint8)} bytes")
        self.info_publisher.publish(info_msg)
        
        self.get_logger().info(f"Published complete WAV file of length {len(data_uint8)} bytes")

def main(args=None):
    rclpy.init(args=args)
    node = WavToUint8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
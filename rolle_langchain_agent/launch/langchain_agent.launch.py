#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='map.yaml',
        description='Path to the map YAML file'
    )
    
    polygons_file_arg = DeclareLaunchArgument(
        'polygons_file',
        default_value='polygons.json',
        description='Path to the polygons JSON file'
    )
    
    # Get directory path
    pkg_dir = get_package_share_directory('rolle_langchain_agent')
    
    # Create launch description
    ld = LaunchDescription([
        map_file_arg,
        polygons_file_arg,
        
        # Robot Navigation Node
        Node(
            package='rolle_langchain_agent',
            executable='navigation_node',
            name='robot_navigation_node',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'polygons_file': LaunchConfiguration('polygons_file'),
            }],
            # Ensure the node has access to the OpenAI API key
            additional_env={'OPENAI_API_KEY': EnvironmentVariable('OPENAI_API_KEY')},
        ),
        
        # Command Processor Node
        Node(
            package='rolle_langchain_agent',
            executable='command_processor',
            name='command_processor_node',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
            }],
            # Ensure the node has access to the OpenAI API key
            additional_env={'OPENAI_API_KEY': EnvironmentVariable('OPENAI_API_KEY')},
        ),
    ])
    
    return ld
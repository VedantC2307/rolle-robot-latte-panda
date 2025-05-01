#!/usr/bin/env python3

"""
Launch file for the Rolle Navigation Node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Default to the rolle home map
    default_map_yaml = os.path.join(
        get_package_share_directory('my_2wd_robot'),
        'map',
        'rolle_home_map_1.yaml'
    )
    
    default_map_image = os.path.join(
        get_package_share_directory('my_2wd_robot'),
        'map',
        'rolle_home_map_1.pgm'
    )
    
    # Launch arguments
    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value=default_map_yaml,
        description='Path to the map YAML file'
    )
    
    map_image_path_arg = DeclareLaunchArgument(
        'map_image_path',
        default_value=default_map_image,
        description='Path to the map image file'
    )
    
    # Navigation node
    navigation_node = Node(
        package='rolle_navigation',
        executable='rolle_navigation_node',
        name='rolle_navigation_node',
        output='screen',
        parameters=[{
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
            'map_image_path': LaunchConfiguration('map_image_path'),
        }],
    )
    
    return LaunchDescription([
        map_yaml_path_arg,
        map_image_path_arg,
        navigation_node
    ])
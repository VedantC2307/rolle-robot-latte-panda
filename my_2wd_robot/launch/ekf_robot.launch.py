# filepath: /home/vedant/rolle_robot/src/my_2wd_robot/launch/sensor_fusion.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = '/home/vedant/rolle_robot/src/my_2wd_robot/'
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    robot_pkg_share = get_package_share_directory('my_2wd_robot')

    launch_robot= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'robot_bridge.launch.py')
        )
    )
    
    robot_localization= Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('/odometry/filtered', '/odom')]
        )
    
    return LaunchDescription([
        launch_robot,
        robot_localization
    ])
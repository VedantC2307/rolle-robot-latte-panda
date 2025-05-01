import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_pkg_share = get_package_share_directory('my_2wd_robot')

    robot_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'robot_bridge.launch.py')
        )
    )

    slam_params_file = PathJoinSubstitution([
        FindPackageShare('my_2wd_robot'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    slam_params_file = '/home/vedant/rolle_robot/config/mapper_params_online_async.yaml'


    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        robot_bridge,
        start_async_slam_toolbox_node
    ])
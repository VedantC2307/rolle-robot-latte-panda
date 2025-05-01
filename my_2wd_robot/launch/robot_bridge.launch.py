import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # slam_params_file = LaunchConfiguration('slam_params_file')

    robot_pkg_share = get_package_share_directory('my_2wd_robot')

    launch_rviz= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'view_robot.launch.py')
        )
    )

    real_robot_bridge_node = Node(
        package='vlm_robot_bridge',
        executable='robot_bridge',
        name='robot_bridge_node',
        output='screen'
    )

    robot_bridge_node = Node(
        package='vlm_robot_bridge',
        executable='robot_bridge_odom',
        name='robot_bridge_node',
        output='screen'
    )

    return LaunchDescription([
        launch_rviz,
        # real_robot_bridge_node,
        robot_bridge_node
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the URDF file, processed via xacro
    urdf_file = PathJoinSubstitution([
        FindPackageShare('my_2wd_robot'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
    }

    twist_mux_config = "/home/vedant/rolle_robot/src/my_2wd_robot/config/twist_mux.yaml"
    joy_config = '/home/vedant/rolle_robot/src/my_2wd_robot/config/xbox_joy.yaml'
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value=urdf_file,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        # Launch joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Launch RViz2 with your RViz configuration (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_rolle_robot_viewer',
            namespace='rolle_robot',  # Add a namespace to further isolate the node
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_2wd_robot'),
                'rviz',
                'robot.rviz'
            ])]
        ),
        # Launch joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        # Launch teleop_twist_joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('cmd_vel_out', '/cmd_vel_rolle')
            ],
        )
    ])
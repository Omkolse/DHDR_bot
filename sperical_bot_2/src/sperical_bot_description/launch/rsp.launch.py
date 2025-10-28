import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = LaunchConfiguration('robot_description')

    default_urdf_path = PathJoinSubstitution([
        get_package_share_directory('sperical_bot_description'),
        'urdf', 
        'sperical_bot.xacro'
    ])

    robot_description_fallback = Command([
        FindExecutable(name='xacro'), ' ', default_urdf_path
         
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_fallback,
            description='Robot description content'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }]
        ),
    ])  
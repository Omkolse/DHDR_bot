from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('sperical_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'sperical_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_description_config = Command(['xacro ', xacro_file])

    robot_description = {'robot_description': robot_description_config}


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py')),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output = 'screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
  
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sperical_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        urdf_spawn_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])

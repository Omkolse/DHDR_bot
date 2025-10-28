import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('sperical_bot_description'), 'config')
    
    return LaunchDescription([
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(config_dir, 'controllers.yaml')],
            output='screen'
        ),
        
        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        
        # Diff Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
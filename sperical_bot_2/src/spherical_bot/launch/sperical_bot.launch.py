#!/usr/bin/env python3
"""
Spherical Bot - Main Launch File
Launches all nodes with proper configuration and parameters
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    # Package paths
    pkg_path = FindPackageShare('spherical_bot')
    config_dir = PathJoinSubstitution([pkg_path, 'config'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_mode = LaunchConfiguration('debug_mode')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_debug_mode_arg = DeclareLaunchArgument(
        'debug_mode', 
        default_value='false',
        description='Enable debug output and logging'
    )
    
    # Node configurations
    core_controller_node = Node(
        package='spherical_bot',
        executable='core_controller',
        name='core_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'params.yaml']),
            PathJoinSubstitution([config_dir, 'pid_gains.yaml']),
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {'use_sim_time': use_sim_time},
            {'debug.enable_debug_output': debug_mode}
        ],
        # Set high priority for real-time control
        ros_arguments=['--log-level', 'info'],
        # CPU affinity and priority (Linux only)
        additional_env={'PYTHONASYNCIODEBUG': '0'},
        respawn=True,
        respawn_delay=2
    )
    
    sensor_manager_node = Node(
        package='spherical_bot',
        executable='sensor_manager',
        name='sensor_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'params.yaml']),
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {'use_sim_time': use_sim_time},
            {'debug.enable_debug_output': debug_mode}
        ],
        ros_arguments=['--log-level', 'info'],
        respawn=True,
        respawn_delay=2
    )
    
    cloud_bridge_node = Node(
        package='spherical_bot',
        executable='cloud_bridge',
        name='cloud_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'params.yaml']),
            {'use_sim_time': use_sim_time},
            {'debug.enable_debug_output': debug_mode}
        ],
        ros_arguments=['--log-level', 'info'],
        respawn=True,
        respawn_delay=5
    )
    
    system_guardian_node = Node(
        package='spherical_bot',
        executable='system_guardian',
        name='system_guardian',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'params.yaml']),
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {'use_sim_time': use_sim_time},
            {'debug.enable_debug_output': debug_mode}
        ],
        ros_arguments=['--log-level', 'warn'],
        respawn=True,
        respawn_delay=3
    )
    
    # Hardware setup script (run once at startup)
    hardware_setup = ExecuteProcess(
        cmd=[
            'bash', 
            PathJoinSubstitution([pkg_path, 'scripts', 'setup_hardware.sh'])
        ],
        output='screen',
        shell=False
    )
    
    # System monitor script
    system_monitor = ExecuteProcess(
        cmd=[
            'bash',
            PathJoinSubstitution([pkg_path, 'scripts', 'system_monitor.sh'])
        ],
        output='screen',
        shell=False
    )
    
    # Event handlers for proper startup sequence
    startup_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=hardware_setup,
            on_start=[
                core_controller_node,
                sensor_manager_node,
            ]
        )
    )
    
    sensor_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=sensor_manager_node,
            on_start=[
                system_guardian_node,
                cloud_bridge_node,
            ]
        )
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_debug_mode_arg)
    
    # Add hardware setup first
    ld.add_action(hardware_setup)
    
    # Add nodes with event handlers for proper startup sequence
    ld.add_action(startup_event_handler)
    ld.add_action(sensor_start_handler)
    
    # Add system monitor
    ld.add_action(system_monitor)
    
    return ld
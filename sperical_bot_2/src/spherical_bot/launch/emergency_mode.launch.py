#!/usr/bin/env python3
"""
Spherical Bot - Emergency Mode Launch File
Minimal system for emergency recovery and safe operation
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    pkg_path = FindPackageShare('spherical_bot')
    config_dir = PathJoinSubstitution([pkg_path, 'config'])
    
    # Launch arguments
    emergency_reason = LaunchConfiguration('emergency_reason')
    
    declare_emergency_reason_arg = DeclareLaunchArgument(
        'emergency_reason',
        default_value='unknown',
        description='Reason for entering emergency mode'
    )
    
    # Emergency system guardian (highest priority)
    emergency_guardian = Node(
        package='spherical_bot',
        executable='system_guardian',
        name='emergency_guardian',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'params.yaml']),
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {
                'use_sim_time': False,
                'debug.enable_debug_output': True,
                'safety.emergency_mode': True
            }
        ],
        # Highest priority in emergency
        ros_arguments=['--log-level', 'warn'],
        respawn=False  # Don't respawn in emergency mode
    )
    
    # Minimal core controller (balance only, no navigation)
    minimal_core_controller = Node(
        package='spherical_bot',
        executable='core_controller',
        name='minimal_core_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'pid_gains.yaml']),
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {
                'use_sim_time': False,
                'control_rates.core_controller': 50,  # Reduced rate
                'balance.emergency_tilt_threshold': 0.4,  # More conservative
                'motors.max_speed': 0.3,  # Limited speed
                'safety.emergency_mode': True
            }
        ],
        ros_arguments=['--log-level', 'info'],
        respawn=False
    )
    
    # Basic sensor manager (IMU only, no TOF)
    basic_sensor_manager = Node(
        package='spherical_bot',
        executable='sensor_manager',
        name='basic_sensor_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
            {
                'use_sim_time': False,
                'control_rates.sensor_manager': 25,  # Reduced rate
                'sensors.tof_sample_rate': 0,  # Disable TOF
                'sensors.encoder_sample_rate': 10,  # Reduced rate
                'safety.emergency_mode': True
            }
        ],
        ros_arguments=['--log-level', 'warn'],
        respawn=False
    )
    
    # Emergency status reporter
    emergency_status = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([pkg_path, 'scripts', 'emergency_status.py']),
            '--reason', emergency_reason
        ],
        output='screen',
        shell=False
    )
    
    # Safety shutdown monitor
    safety_monitor = ExecuteProcess(
        cmd=[
            'bash',
            PathJoinSubstitution([pkg_path, 'scripts', 'safety_shutdown.sh'])
        ],
        output='screen',
        shell=False
    )
    
    # Create emergency launch description
    ld = LaunchDescription()
    
    # Add launch argument
    ld.add_action(declare_emergency_reason_arg)
    
    # Add emergency nodes in order of priority
    ld.add_action(emergency_guardian)
    ld.add_action(minimal_core_controller)
    ld.add_action(basic_sensor_manager)
    ld.add_action(emergency_status)
    ld.add_action(safety_monitor)
    
    return ld
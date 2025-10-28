#!/usr/bin/env python3
"""
Odometry Calculator Library
Wheel odometry from encoder readings
"""

import math
import numpy as np

class OdometryCalculator:
    """Wheel odometry calculation from encoder ticks"""
    
    def __init__(self):
        # Robot physical parameters
        self.wheel_base = 0.15  # meters (distance between wheels)
        self.wheel_radius = 0.035  # meters
        self.ticks_per_revolution = 1000  # Encoder ticks per wheel revolution
        
        # Current pose
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        
        # Current velocity
        self.vx = 0.0  # m/s
        self.vy = 0.0  # m/s
        self.vtheta = 0.0  # rad/s
        
        # Encoder state
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.left_ticks = 0
        self.right_ticks = 0
        
        # Calibration
        self.left_calibration = 1.0
        self.right_calibration = 1.0
        
        # Timing
        self.last_update_time = None
        
    def update(self, left_ticks, right_ticks, current_time=None):
        """
        Update odometry with new encoder readings
        
        Args:
            left_ticks: Left encoder tick count
            right_ticks: Right encoder tick count
            current_time: Current time (seconds). If None, uses system time
            
        Returns:
            tuple: (x, y, theta, vx, vy, vtheta)
        """
        import time
        
        if current_time is None:
            current_time = time.time()
        
        # First update - initialize only
        if self.last_update_time is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_update_time = current_time
            return self.x, self.y, self.theta, self.vx, self.vy, self.vtheta
        
        # Calculate time delta
        dt = current_time - self.last_update_time
        if dt <= 0:
            return self.x, self.y, self.theta, self.vx, self.vy, self.vtheta
        
        # Calculate tick differences
        delta_left = (left_ticks - self.prev_left_ticks) * self.left_calibration
        delta_right = (right_ticks - self.prev_right_ticks) * self.right_calibration
        
        # Update previous ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.left_ticks = left_ticks
        self.right_ticks = right_ticks
        
        # Calculate distances traveled by each wheel
        left_distance = (delta_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        
        # Calculate linear and angular displacement
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Update pose
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        self.vx = distance * math.cos(self.theta) / dt
        self.vy = distance * math.sin(self.theta) / dt
        self.vtheta = delta_theta / dt
        
        self.last_update_time = current_time
        
        return self.x, self.y, self.theta, self.vx, self.vy, self.vtheta
    
    def get_pose(self):
        """Get current pose"""
        return self.x, self.y, self.theta
    
    def get_velocity(self):
        """Get current velocity"""
        return self.vx, self.vy, self.vtheta
    
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset odometry to specified pose"""
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_update_time = None
    
    def set_calibration(self, left_calibration, right_calibration):
        """Set wheel calibration factors"""
        self.left_calibration = left_calibration
        self.right_calibration = right_calibration
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def calculate_heading(self, x1, y1, x2, y2):
        """Calculate heading from point1 to point2"""
        return math.atan2(y2 - y1, x2 - x1)
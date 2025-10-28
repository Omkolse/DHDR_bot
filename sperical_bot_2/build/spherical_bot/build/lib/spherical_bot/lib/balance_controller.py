#!/usr/bin/env python3
"""
Balance Controller Library
PID-based balance control for spherical bot
"""

import math
import time

class BalanceController:
    """PID balance controller with anti-windup and feedforward"""
    
    def __init__(self, kp=25.0, ki=0.5, kd=0.8, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Limits
        self.output_limits = (-1.0, 1.0)
        self.integral_limits = (-2.0, 2.0)
        
        # Feedforward for velocity
        self.velocity_ff = 0.1
        
        # Debug
        self.debug = False
    
    def update(self, current_angle, current_velocity=0.0, dt=None):
        """
        Calculate balance output
        
        Args:
            current_angle: Current tilt angle (radians)
            current_velocity: Current angular velocity (rad/s)
            dt: Time delta (seconds). If None, calculates automatically
            
        Returns:
            float: Motor output (-1.0 to 1.0)
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
        
        # Ensure dt is reasonable
        dt = max(0.001, min(dt, 0.1))
        
        # Calculate error
        error = self.setpoint - current_angle
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limits[1]), 
                           self.integral_limits[0])
        i_term = self.ki * self.integral
        
        # Derivative term (from error)
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Feedforward from velocity (helps with movement)
        ff_term = self.velocity_ff * current_velocity
        
        # Calculate output
        output = p_term + i_term + d_term + ff_term
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        if self.debug:
            print(f"Balance PID: P={p_term:.3f}, I={i_term:.3f}, "
                  f"D={d_term:.3f}, FF={ff_term:.3f}, Out={output:.3f}")
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def set_gains(self, kp, ki, kd):
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def set_setpoint(self, setpoint):
        """Update balance setpoint"""
        self.setpoint = setpoint
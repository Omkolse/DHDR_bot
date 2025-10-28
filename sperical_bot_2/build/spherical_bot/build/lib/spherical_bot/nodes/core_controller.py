#!/usr/bin/env python3
"""
Core Controller Node - REAL-TIME BALANCE CONTROL
Runs at 100Hz on Core 0 with highest priority
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
import threading
import math
import time
import os

# Set real-time priority and CPU affinity
os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
os.sched_setaffinity(0, {0})  # Pin to core 0

class BalanceController:
    """PID-based balance controller for spherical bot"""
    
    def __init__(self, kp=25.0, ki=0.5, kd=0.8):
        self.kp = kp
        self.ki = ki 
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.setpoint = 0.0  # Upright position
        self.output_limits = (-1.0, 1.0)
        
    def update(self, current_angle, dt):
        """Calculate motor output based on current tilt angle"""
        error = self.setpoint - current_angle
        
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 2.0), -2.0)  # Clamp integral
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        return output

class MotorDriver:
    """TB6612 Motor Driver Interface"""
    
    def __init__(self):
        self.motor_a_speed = 0.0
        self.motor_b_speed = 0.0
        self.emergency_stop = False
        
    def set_speeds(self, balance_output, linear_cmd=0.0, angular_cmd=0.0):
        """Set motor speeds with balance correction"""
        if self.emergency_stop:
            self.motor_a_speed = 0.0
            self.motor_b_speed = 0.0
            return
            
        # Combine balance control with movement commands
        left_speed = balance_output + linear_cmd - angular_cmd
        right_speed = balance_output + linear_cmd + angular_cmd
        
        # Apply speed limits
        self.motor_a_speed = max(min(left_speed, 1.0), -1.0)
        self.motor_b_speed = max(min(right_speed, 1.0), -1.0)
        
    def stop_motors(self):
        """Emergency stop"""
        self.emergency_stop = True
        self.motor_a_speed = 0.0
        self.motor_b_speed = 0.0

class TiltEstimator:
    """IMU-based tilt angle estimation"""
    
    def __init__(self):
        self.current_angle = 0.0
        self.angular_velocity = 0.0
        self.calibration_samples = 100
        self.angle_offset = 0.0
        
    def update_from_imu(self, imu_msg):
        """Update tilt angle from IMU data (quaternion)"""
        # Extract quaternion
        x = imu_msg.orientation.x
        y = imu_msg.orientation.y
        z = imu_msg.orientation.z
        w = imu_msg.orientation.w
        
        # Convert quaternion to Euler angles (simplified for pitch)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        self.current_angle = pitch - self.angle_offset
        self.angular_velocity = imu_msg.angular_velocity.y
        
    def calibrate(self, imu_readings):
        """Calibrate zero position"""
        if len(imu_readings) < self.calibration_samples:
            return False
            
        self.angle_offset = sum(imu_readings) / len(imu_readings)
        return True

class CoreController(Node):
    """Main Core Controller Node"""
    
    def __init__(self):
        super().__init__('core_controller')
        
        # Real-time QoS settings
        qos_realtime = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Initialize components
        self.balance_controller = BalanceController()
        self.motor_driver = MotorDriver()
        self.tilt_estimator = TiltEstimator()
        
        # Publishers
        self.motor_cmd_pub = self.create_publisher(Twist, 'motor_commands', qos_realtime)
        self.balance_status_pub = self.create_publisher(Float32, 'balance_status', qos_realtime)
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, qos_realtime)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_realtime)
        self.emergency_sub = self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, qos_realtime)
        
        # Control loop timer (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
        # State variables
        self.last_time = self.get_clock().now()
        self.current_cmd_vel = Twist()
        self.emergency_stop = False
        self.calibration_mode = True
        self.calibration_samples = []
        
        self.get_logger().info("🚀 Core Controller initialized - Real-time balance control ready")

    def imu_callback(self, msg):
        """Process IMU data for tilt estimation"""
        self.tilt_estimator.update_from_imu(msg)
        
        if self.calibration_mode:
            self.calibration_samples.append(self.tilt_estimator.current_angle)
            if len(self.calibration_samples) >= 100:
                if self.tilt_estimator.calibrate(self.calibration_samples):
                    self.calibration_mode = False
                    self.get_logger().info(" IMU calibration complete")

    def cmd_vel_callback(self, msg):
        """Process velocity commands from navigation"""
        self.current_cmd_vel = msg

    def emergency_callback(self, msg):
        """Handle emergency stop commands"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.motor_driver.stop_motors()
            self.get_logger().warn("EMERGENCY STOP ACTIVATED")

    def control_loop(self):
        """Main control loop - runs at 100Hz"""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            
            if self.emergency_stop:
                return
                
            if self.calibration_mode:
                return
                
            # Get balance control output
            balance_output = self.balance_controller.update(
                self.tilt_estimator.current_angle, 
                max(dt, 0.001)  # Prevent division by zero
            )
            
            # Set motor speeds
            self.motor_driver.set_speeds(
                balance_output,
                self.current_cmd_vel.linear.x,
                self.current_cmd_vel.angular.z
            )
            
            # Publish motor commands
            motor_msg = Twist()
            motor_msg.linear.x = self.motor_driver.motor_a_speed
            motor_msg.linear.y = self.motor_driver.motor_b_speed
            self.motor_cmd_pub.publish(motor_msg)
            
            # Publish balance status
            status_msg = Float32()
            status_msg.data = self.tilt_estimator.current_angle
            self.balance_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")
            # Emergency stop on error
            self.motor_driver.stop_motors()

    def destroy_node(self):
        """Clean shutdown"""
        self.motor_driver.stop_motors()
        super().destroy_node()

def main(args=None):
    # Set process priority
    try:
        import psutil
        p = psutil.Process()
        p.nice(-10)  # High priority
    except:
        pass
        
    rclpy.init(args=args)
    
    try:
        core_controller = CoreController()
        rclpy.spin(core_controller)
    except KeyboardInterrupt:
        pass
    finally:
        core_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
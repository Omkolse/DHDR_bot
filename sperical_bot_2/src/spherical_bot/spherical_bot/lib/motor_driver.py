#!/usr/bin/env python3
"""
Motor Driver Library
TB6612FNG Motor Driver Interface for Raspberry Pi
"""

import time

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available - motor control simulated")

class MotorDriver:
    """TB6612FNG Dual Motor Driver Controller"""
    
    def __init__(self):
        # Motor A (Left) pins
        self.MOTOR_A_IN1 = 17  # GPIO17 - Physical pin 11
        self.MOTOR_A_IN2 = 27  # GPIO27 - Physical pin 13
        self.MOTOR_A_PWM = 13  # GPIO13 - Physical pin 33
        
        # Motor B (Right) pins  
        self.MOTOR_B_IN1 = 22  # GPIO22 - Physical pin 15
        self.MOTOR_B_IN2 = 23  # GPIO23 - Physical pin 16
        self.MOTOR_B_PWM = 18  # GPIO18 - Physical pin 12
        
        # PWM frequency (Hz)
        self.PWM_FREQ = 1000
        
        # Motor state
        self.motor_a_speed = 0.0
        self.motor_b_speed = 0.0
        self.emergency_stop = False
        self.initialized = False
        
        # PWM objects
        self.pwm_a = None
        self.pwm_b = None
        
    def initialize(self):
        """Initialize GPIO and PWM for motors"""
        if not GPIO_AVAILABLE:
            print("MotorDriver: Running in simulation mode")
            self.initialized = True
            return True
            
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor A pins
            GPIO.setup(self.MOTOR_A_IN1, GPIO.OUT)
            GPIO.setup(self.MOTOR_A_IN2, GPIO.OUT)
            GPIO.setup(self.MOTOR_A_PWM, GPIO.OUT)
            
            # Setup motor B pins
            GPIO.setup(self.MOTOR_B_IN1, GPIO.OUT)
            GPIO.setup(self.MOTOR_B_IN2, GPIO.OUT)
            GPIO.setup(self.MOTOR_B_PWM, GPIO.OUT)
            
            # Initialize PWM
            self.pwm_a = GPIO.PWM(self.MOTOR_A_PWM, self.PWM_FREQ)
            self.pwm_b = GPIO.PWM(self.MOTOR_B_PWM, self.PWM_FREQ)
            self.pwm_a.start(0)
            self.pwm_b.start(0)
            
            self.initialized = True
            print("MotorDriver: Initialized successfully")
            return True
            
        except Exception as e:
            print(f"MotorDriver: Initialization failed - {e}")
            return False
    
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set motor speeds (-1.0 to 1.0)
        
        Args:
            left_speed: Left motor speed (-1.0 to 1.0)
            right_speed: Right motor speed (-1.0 to 1.0)
        """
        if self.emergency_stop:
            left_speed = 0.0
            right_speed = 0.0
        
        # Clamp speeds
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        self.motor_a_speed = left_speed
        self.motor_b_speed = right_speed
        
        if not self.initialized or not GPIO_AVAILABLE:
            # Simulation mode
            return
        
        try:
            # Set motor A (left)
            self._set_motor(self.MOTOR_A_IN1, self.MOTOR_A_IN2, 
                          self.pwm_a, left_speed)
            
            # Set motor B (right)
            self._set_motor(self.MOTOR_B_IN1, self.MOTOR_B_IN2, 
                          self.pwm_b, right_speed)
                          
        except Exception as e:
            print(f"MotorDriver: Speed setting error - {e}")
    
    def _set_motor(self, in1_pin, in2_pin, pwm, speed):
        """Set individual motor direction and speed"""
        # Convert -1.0 to 1.0 to 0-100 duty cycle
        duty_cycle = abs(speed) * 100
        
        if speed > 0:
            # Forward
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        elif speed < 0:
            # Backward
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        else:
            # Brake
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.LOW)
        
        pwm.ChangeDutyCycle(duty_cycle)
    
    def stop_motors(self):
        """Stop both motors immediately"""
        self.set_motor_speeds(0.0, 0.0)
    
    def emergency_stop(self):
        """Emergency stop - disable all motors"""
        self.emergency_stop = True
        self.stop_motors()
    
    def clear_emergency(self):
        """Clear emergency stop state"""
        self.emergency_stop = False
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        if self.initialized and GPIO_AVAILABLE:
            try:
                self.stop_motors()
                if self.pwm_a:
                    self.pwm_a.stop()
                if self.pwm_b:
                    self.pwm_b.stop()
                GPIO.cleanup()
                self.initialized = False
            except Exception as e:
                print(f"MotorDriver: Cleanup error - {e}")
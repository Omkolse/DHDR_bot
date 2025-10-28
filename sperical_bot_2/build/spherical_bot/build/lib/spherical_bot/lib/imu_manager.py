#!/usr/bin/env python3
"""
IMU Manager Library
ICM-20948 9-DOF IMU Interface
"""

import math
import time
import numpy as np

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("WARNING: smbus not available - IMU running in simulation mode")

class IMUManager:
    """ICM-20948 IMU Manager with sensor fusion"""
    
    def __init__(self, i2c_bus=1, i2c_address=0x68):
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.bus = None
        
        # Sensor data
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        
        # Orientation (quaternion)
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        
        # Tilt angles (radians)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        # Calibration
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.mag_offset = [0.0, 0.0, 0.0]
        
        self.calibrated = False
        self.calibration_samples = 1000
        
        # Simple complementary filter
        self.filter_alpha = 0.98
        self.last_time = time.time()
        
        # Initialize IMU
        self._init_imu()
    
    def _init_imu(self):
        """Initialize ICM-20948 IMU"""
        if not I2C_AVAILABLE:
            print("IMUManager: Running in simulation mode")
            return True
            
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            
            # Wake up the device (exit sleep mode)
            self._write_byte(0x06, 0x01)  # PWR_MGMT_1
            time.sleep(0.1)
            
            # Configure accelerometer
            self._write_byte(0x14, 0x18)  # ACCEL_CONFIG - ±16g
            
            # Configure gyroscope  
            self._write_byte(0x01, 0x18)  # GYRO_CONFIG - ±2000dps
            
            print("IMUManager: ICM-20948 initialized successfully")
            return True
            
        except Exception as e:
            print(f"IMUManager: Initialization failed - {e}")
            self.bus = None
            return False
    
    def _write_byte(self, register, value):
        """Write byte to IMU register"""
        if self.bus:
            self.bus.write_byte_data(self.i2c_address, register, value)
    
    def _read_bytes(self, register, length):
        """Read bytes from IMU register"""
        if self.bus:
            return self.bus.read_i2c_block_data(self.i2c_address, register, length)
        return [0] * length
    
    def read_data(self):
        """Read all IMU data"""
        if not self.bus:
            # Generate simulated IMU data
            return self._read_simulated_data()
        
        try:
            # Read accelerometer (registers 0x2D-0x32)
            accel_data = self._read_bytes(0x2D, 6)
            self.accel[0] = self._convert_accel(accel_data[0], accel_data[1])
            self.accel[1] = self._convert_accel(accel_data[2], accel_data[3])  
            self.accel[2] = self._convert_accel(accel_data[4], accel_data[5])
            
            # Read gyroscope (registers 0x33-0x38)
            gyro_data = self._read_bytes(0x33, 6)
            self.gyro[0] = self._convert_gyro(gyro_data[0], gyro_data[1])
            self.gyro[1] = self._convert_gyro(gyro_data[2], gyro_data[3])
            self.gyro[2] = self._convert_gyro(gyro_data[4], gyro_data[5])
            
            # Apply calibration
            if self.calibrated:
                self.accel = [a - o for a, o in zip(self.accel, self.accel_offset)]
                self.gyro = [g - o for g, o in zip(self.gyro, self.gyro_offset)]
            
            # Update orientation using complementary filter
            self._update_orientation()
            
            return True
            
        except Exception as e:
            print(f"IMUManager: Read error - {e}")
            return False
    
    def _convert_accel(self, high_byte, low_byte):
        """Convert accelerometer raw data to m/s²"""
        raw_value = (high_byte << 8) | low_byte
        if raw_value >= 0x8000:
            raw_value -= 0x10000
        return raw_value / 16384.0 * 9.8  # ±16g range
    
    def _convert_gyro(self, high_byte, low_byte):
        """Convert gyroscope raw data to rad/s"""
        raw_value = (high_byte << 8) | low_byte
        if raw_value >= 0x8000:
            raw_value -= 0x10000
        return raw_value / 16.384 * math.pi / 180.0  # ±2000dps range
    
    def _read_simulated_data(self):
        """Generate simulated IMU data for testing"""
        # Simulate slight noise and movement
        self.accel = [
            np.random.normal(0, 0.1),
            np.random.normal(0, 0.1), 
            np.random.normal(9.8, 0.1)
        ]
        self.gyro = [
            np.random.normal(0, 0.01),
            np.random.normal(0, 0.01),
            np.random.normal(0, 0.01)
        ]
        
        self._update_orientation()
        return True
    
    def _update_orientation(self):
        """Update orientation using complementary filter"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate tilt from accelerometer
        accel_pitch = math.atan2(self.accel[1], 
                                math.sqrt(self.accel[0]**2 + self.accel[2]**2))
        accel_roll = math.atan2(-self.accel[0], 
                               math.sqrt(self.accel[1]**2 + self.accel[2]**2))
        
        # Integrate gyroscope
        gyro_pitch = self.pitch + self.gyro[1] * dt
        gyro_roll = self.roll + self.gyro[0] * dt
        
        # Complementary filter
        self.pitch = (self.filter_alpha * gyro_pitch + 
                     (1 - self.filter_alpha) * accel_pitch)
        self.roll = (self.filter_alpha * gyro_roll + 
                    (1 - self.filter_alpha) * accel_roll)
        
        # Convert to quaternion (simplified)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)
        
        self.orientation[0] = cr * cp * cy + sr * sp * sy
        self.orientation[1] = sr * cp * cy - cr * sp * sy
        self.orientation[2] = cr * sp * cy + sr * cp * sy
        self.orientation[3] = cr * cp * sy - sr * sp * cy
    
    def calibrate(self, samples=1000):
        """Calibrate IMU offsets"""
        print("IMUManager: Starting calibration...")
        
        accel_samples = []
        gyro_samples = []
        
        for i in range(samples):
            if self.read_data():
                accel_samples.append(self.accel[:])
                gyro_samples.append(self.gyro[:])
            time.sleep(0.01)
        
        if accel_samples and gyro_samples:
            # Calculate offsets (assume device is stationary)
            self.accel_offset = np.mean(accel_samples, axis=0)
            self.accel_offset[2] -= 9.8  # Remove gravity from Z-axis
            
            self.gyro_offset = np.mean(gyro_samples, axis=0)
            
            self.calibrated = True
            print("IMUManager: Calibration complete")
            return True
        
        return False
    
    def get_tilt_angle(self):
        """Get current tilt angle (pitch) in radians"""
        return self.pitch
    
    def get_angular_velocity(self):
        """Get current angular velocity in rad/s"""
        return self.gyro[1]  # Pitch rate
    
    def cleanup(self):
        """Cleanup IMU resources"""
        if self.bus:
            try:
                # Put IMU to sleep
                self._write_byte(0x06, 0x40)  # PWR_MGMT_1 - sleep mode
            except:
                pass
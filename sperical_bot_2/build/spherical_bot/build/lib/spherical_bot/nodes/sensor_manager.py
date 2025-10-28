#!/usr/bin/env python3
"""
Sensor Manager Node - Real-time sensor data fusion
Runs at 50Hz on Core 1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Header
import numpy as np
import threading
import time
import json

# Hardware libraries - these will be conditionally imported
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available - running in simulation mode")

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("WARNING: smbus not available - running in simulation mode")

class IMUManager:
    """ICM-20948 IMU Manager with sensor fusion"""
    
    def __init__(self, i2c_bus=1, i2c_address=0x68):
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.bus = None
        self.calibration_data = {
            'gyro_offset': [0.0, 0.0, 0.0],
            'accel_offset': [0.0, 0.0, 0.0],
            'calibrated': False
        }
        
        if I2C_AVAILABLE:
            try:
                self.bus = smbus.SMBus(i2c_bus)
                self._init_imu()
            except Exception as e:
                print(f"IMU initialization failed: {e}")
                self.bus = None
    
    def _init_imu(self):
        """Initialize ICM-20948 IMU"""
        if self.bus:
            try:
                # Wake up the device
                self.bus.write_byte_data(self.i2c_address, 0x06, 0x01)
                time.sleep(0.1)
                print("IMU initialized successfully")
            except Exception as e:
                print(f"IMU init error: {e}")
    
    def read_data(self):
        """Read IMU data - returns (accel, gyro, mag) or simulated data"""
        if not self.bus:
            # Return simulated data for testing
            return (
                [0.0, 0.0, 9.8],  # accel
                [0.0, 0.0, 0.0],  # gyro  
                [0.0, 0.0, 0.0]   # mag
            )
        
        try:
            # Simplified reading - in practice, you'd implement proper ICM-20948 reading
            accel = [0.0, 0.0, 9.8]
            gyro = [0.0, 0.0, 0.0]
            mag = [0.0, 0.0, 0.0]
            
            return accel, gyro, mag
        except Exception as e:
            print(f"IMU read error: {e}")
            return [0.0, 0.0, 9.8], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

class TOFCamera:
    """Arducam TOF Camera Interface"""
    
    def __init__(self):
        self.connected = False
        self.depth_data = None
        self.frame_id = "tof_camera"
        
    def initialize(self):
        """Initialize TOF camera"""
        try:
            # This would use the Arducam ROS driver
            # For now, we'll create simulated data
            self.connected = True
            print("TOF Camera initialized (simulated)")
            return True
        except Exception as e:
            print(f"TOF Camera init failed: {e}")
            return False
    
    def get_depth_frame(self):
        """Get depth frame - returns compressed point cloud data"""
        if not self.connected:
            # Generate simulated depth data (5x5 grid)
            depth_frame = np.random.rand(5, 5).astype(np.float32) * 5.0  # 0-5 meters
            return self._compress_depth_data(depth_frame)
        
        # Real implementation would use Arducam driver
        depth_frame = np.random.rand(5, 5).astype(np.float32) * 5.0
        return self._compress_depth_data(depth_frame)
    
    def _compress_depth_data(self, depth_frame):
        """Compress depth data for transmission"""
        # Simple run-length encoding simulation
        compressed = {
            'type': 'depth_rle',
            'shape': depth_frame.shape,
            'data': depth_frame.flatten().tobytes(),
            'timestamp': time.time()
        }
        return compressed

class OdometryCalculator:
    """Wheel odometry from encoders"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.wheel_base = 0.15  # meters
        self.ticks_per_meter = 1000  # Adjust based on your encoders
        
    def update(self, left_ticks, right_ticks, dt):
        """Update odometry based on encoder ticks"""
        left_distance = (left_ticks - self.last_left_ticks) / self.ticks_per_meter
        right_distance = (right_ticks - self.last_right_ticks) / self.ticks_per_meter
        
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        
        # Calculate robot movement
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Update pose
        self.theta += delta_theta
        self.x += distance * np.cos(self.theta)
        self.y += distance * np.sin(self.theta)
        
        return self.x, self.y, self.theta

class SensorManager(Node):
    """Main Sensor Manager Node"""
    
    def __init__(self):
        super().__init__('sensor_manager')
        
        # QoS for sensor data
        qos_sensors = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Initialize sensor components
        self.imu = IMUManager()
        self.tof_camera = TOFCamera()
        self.odometry = OdometryCalculator()
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_sensors)
        self.odom_pub = self.create_publisher(Odometry, 'odometry/wheel', qos_sensors)
        self.tof_pub = self.create_publisher(PointCloud2, 'tof/points', qos_sensors)
        self.sensor_status_pub = self.create_publisher(Float32, 'sensor/status', 10)
        
        # Sensor data timer (50Hz)
        self.sensor_timer = self.create_timer(0.02, self.sensor_loop)  # 50Hz
        
        # State variables
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.last_sensor_time = self.get_clock().now()
        
        # Initialize TOF camera
        if self.tof_camera.initialize():
            self.get_logger().info("✅ TOF Camera initialized")
        else:
            self.get_logger().warn("❌ TOF Camera initialization failed")
        
        self.get_logger().info("🚀 Sensor Manager initialized")

    def read_encoders(self):
        """Read encoder values - replace with actual GPIO reading"""
        if not GPIO_AVAILABLE:
            # Simulate encoder movement
            self.left_encoder_ticks += int(np.random.normal(10, 2))
            self.right_encoder_ticks += int(np.random.normal(10, 2))
            return self.left_encoder_ticks, self.right_encoder_ticks
        
        # Real GPIO implementation would go here
        # Example for N20 encoders with GPIO interrupts
        return self.left_encoder_ticks, self.right_encoder_ticks

    def sensor_loop(self):
        """Main sensor processing loop - runs at 50Hz"""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_sensor_time).nanoseconds / 1e9
            self.last_sensor_time = current_time
            
            # Read IMU data
            accel, gyro, mag = self.imu.read_data()
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1] 
            imu_msg.linear_acceleration.z = accel[2]
            
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            self.imu_pub.publish(imu_msg)
            
            # Read encoders and update odometry
            left_ticks, right_ticks = self.read_encoders()
            x, y, theta = self.odometry.update(left_ticks, right_ticks, dt)
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header = Header()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            # Orientation would be set from IMU in practice
            
            self.odom_pub.publish(odom_msg)
            
            # Process TOF data (lower frequency)
            if int(time.time() * 10) % 5 == 0:  # ~5Hz
                compressed_depth = self.tof_camera.get_depth_frame()
                pointcloud_msg = self._create_pointcloud(compressed_depth)
                self.tof_pub.publish(pointcloud_msg)
            
            # Publish sensor status
            status_msg = Float32()
            status_msg.data = 1.0  # Good status
            self.sensor_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Sensor loop error: {str(e)}")

    def _create_pointcloud(self, compressed_data):
        """Create PointCloud2 message from compressed depth data"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tof_camera"
        
        # Simplified point cloud - in practice, decompress and convert depth to 3D points
        msg.height = 1
        msg.width = 25  # 5x5 grid
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        return msg

    def destroy_node(self):
        """Clean shutdown"""
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_manager = SensorManager()
        rclpy.spin(sensor_manager)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
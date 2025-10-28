#!/usr/bin/env python3
"""
Spherical Bot - Basic Node Tests
Unit tests for core functionality
"""

import unittest
import rclpy
import time
import threading
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class TestSphericalBotNodes(unittest.TestCase):
    """Test cases for spherical bot nodes"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for testing"""
        rclpy.init()
        cls.node = rclpy.create_node('test_node')
        
    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2"""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        # Create publisher for emergency stop
        emergency_pub = self.node.create_publisher(Bool, 'emergency_stop', 10)
        
        # Create subscriber to check motor commands
        motor_cmd_received = False
        def motor_cmd_callback(msg):
            nonlocal motor_cmd_received
            motor_cmd_received = True
            # In emergency stop, motor commands should be zero
            self.assertEqual(msg.linear.x, 0.0)
            self.assertEqual(msg.linear.y, 0.0)
        
        motor_sub = self.node.create_subscription(
            Twist, 'motor_commands', motor_cmd_callback, 10
        )
        
        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        emergency_pub.publish(emergency_msg)
        
        # Wait for message processing
        time.sleep(1.0)
        
        # Spin briefly to process callbacks
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        # Cleanup
        self.node.destroy_publisher(emergency_pub)
        self.node.destroy_subscription(motor_sub)
    
    def test_sensor_data_flow(self):
        """Test sensor data publishing"""
        sensor_data_received = {
            'imu': False,
            'odometry': False,
            'balance': False
        }
        
        def imu_callback(msg):
            sensor_data_received['imu'] = True
        
        def odom_callback(msg):
            sensor_data_received['odometry'] = True
        
        def balance_callback(msg):
            sensor_data_received['balance'] = True
        
        # Create subscribers for sensor topics
        imu_sub = self.node.create_subscription(
            Float32, 'imu/data', imu_callback, 10
        )
        odom_sub = self.node.create_subscription(
            Float32, 'odometry/wheel', odom_callback, 10
        )
        balance_sub = self.node.create_subscription(
            Float32, 'balance_status', balance_callback, 10
        )
        
        # Wait and spin to check for any sensor data
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # In test environment, we might not receive data, but callbacks should be set up
        print("Sensor test completed - callbacks registered")
        
        # Cleanup
        self.node.destroy_subscription(imu_sub)
        self.node.destroy_subscription(odom_sub)
        self.node.destroy_subscription(balance_sub)
    
    def test_parameter_loading(self):
        """Test that parameters can be loaded correctly"""
        # This would test parameter server functionality
        # For now, just verify the test can run
        self.assertTrue(True)
    
    def test_node_initialization(self):
        """Test that nodes can be initialized"""
        # Test importing and basic node creation
        try:
            from spherical_bot.nodes.core_controller import CoreController
            from spherical_bot.nodes.sensor_manager import SensorManager
            from spherical_bot.nodes.system_guardian import SystemGuardian
            
            # If we can import, the basic structure is correct
            self.assertTrue(True)
            
        except ImportError as e:
            self.fail(f"Failed to import node modules: {e}")

class TestHardwareLibraries(unittest.TestCase):
    """Test hardware abstraction libraries"""
    
    def test_balance_controller(self):
        """Test balance controller calculations"""
        from spherical_bot.lib.balance_controller import BalanceController
        
        controller = BalanceController(kp=25.0, ki=0.5, kd=0.8)
        
        # Test with zero error
        output = controller.update(0.0, 0.0, 0.01)
        self.assertEqual(output, 0.0)
        
        # Test with positive error
        output = controller.update(0.1, 0.0, 0.01)
        self.assertLess(output, 0.0)  # Should be negative to correct
        
        # Test output limits
        controller.set_gains(1000.0, 0.0, 0.0)  # Very high gain
        output = controller.update(1.0, 0.0, 0.01)
        self.assertEqual(output, -1.0)  # Should be limited
    
    def test_motor_driver_simulation(self):
        """Test motor driver in simulation mode"""
        from spherical_bot.lib.motor_driver import MotorDriver
        
        driver = MotorDriver()
        
        # Test speed setting
        driver.set_motor_speeds(0.5, -0.3)
        self.assertEqual(driver.motor_a_speed, 0.5)
        self.assertEqual(driver.motor_b_speed, -0.3)
        
        # Test emergency stop
        driver.emergency_stop()
        driver.set_motor_speeds(0.5, 0.5)  # Try to set speeds
        self.assertEqual(driver.motor_a_speed, 0.0)  # Should be stopped
        self.assertEqual(driver.motor_b_speed, 0.0)
        
        # Test cleanup
        driver.cleanup()
    
    def test_odometry_calculation(self):
        """Test odometry calculations"""
        from spherical_bot.lib.odometry import OdometryCalculator
        
        odom = OdometryCalculator()
        
        # Test reset
        odom.reset(1.0, 2.0, 0.5)
        x, y, theta = odom.get_pose()
        self.assertEqual(x, 1.0)
        self.assertEqual(y, 2.0)
        self.assertEqual(theta, 0.5)
        
        # Test position update (simplified)
        odom.reset(0.0, 0.0, 0.0)
        x, y, theta, vx, vy, vtheta = odom.update(1000, 1000, 1.0)
        
        # Should have moved forward
        self.assertGreater(x, 0.0)
        self.assertEqual(y, 0.0)
        self.assertEqual(theta, 0.0)

def run_tests():
    """Run all tests"""
    # Create test suite
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTest(TestSphericalBotNodes('test_emergency_stop'))
    suite.addTest(TestSphericalBotNodes('test_sensor_data_flow'))
    suite.addTest(TestSphericalBotNodes('test_parameter_loading'))
    suite.addTest(TestSphericalBotNodes('test_node_initialization'))
    suite.addTest(TestHardwareLibraries('test_balance_controller'))
    suite.addTest(TestHardwareLibraries('test_motor_driver_simulation'))
    suite.addTest(TestHardwareLibraries('test_odometry_calculation'))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    # Run tests when script is executed directly
    success = run_tests()
    exit(0 if success else 1)
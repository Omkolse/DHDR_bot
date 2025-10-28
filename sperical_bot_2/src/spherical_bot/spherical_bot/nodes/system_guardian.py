#!/usr/bin/env python3
"""
System Guardian Node - Health monitoring and safety
Runs at 1Hz on Core 2 - Background priority
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
import psutil
import time
import json
import os

# GPIO for hardware monitoring
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available")

class ThermalMonitor:
    """CPU temperature monitoring and management"""
    
    def __init__(self):
        self.current_temp = 0.0
        self.temp_thresholds = {
            'normal': 65.0,    # °C - Full performance
            'warning': 75.0,   # °C - Reduce non-critical tasks
            'critical': 80.0,  # °C - Disable TOF processing
            'emergency': 85.0  # °C - Shutdown non-essential nodes
        }
        self.performance_mode = 'normal'
        
    def read_temperature(self):
        """Read CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
            self.current_temp = temp
            return temp
        except Exception as e:
            print(f"Temperature read error: {e}")
            return 45.0  # Safe default
    
    def check_thermal_state(self):
        """Check and return thermal state"""
        temp = self.read_temperature()
        
        if temp >= self.temp_thresholds['emergency']:
            new_mode = 'emergency'
        elif temp >= self.temp_thresholds['critical']:
            new_mode = 'critical' 
        elif temp >= self.temp_thresholds['warning']:
            new_mode = 'warning'
        else:
            new_mode = 'normal'
        
        if new_mode != self.performance_mode:
            self.performance_mode = new_mode
            print(f"Thermal mode changed to: {new_mode} ({temp}°C)")
        
        return self.performance_mode, temp

class PowerMonitor:
    """Battery voltage and power management"""
    
    def __init__(self):
        self.voltage = 12.6  # Default - would read from ADC
        self.current = 0.0
        self.capacity = 100.0  # Percentage
        
        # LiPo battery thresholds (3S)
        self.voltage_thresholds = {
            'full': 12.6,
            'warning': 11.1,
            'critical': 10.5,
            'shutdown': 9.5
        }
    
    def read_battery(self):
        """Read battery status - simulate for now"""
        # In practice, read from ADC over I2C/SPI
        self.voltage = max(9.0, self.voltage - 0.01)  # Simulate discharge
        self.capacity = (self.voltage - 9.0) / (12.6 - 9.0) * 100
        
        return self.voltage, self.capacity

class ResourceManager:
    """System resource monitoring and management"""
    
    def __init__(self):
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.disk_usage = 0.0
        
    def get_system_resources(self):
        """Get current system resource usage"""
        self.cpu_usage = psutil.cpu_percent(interval=0.1)
        self.memory_usage = psutil.virtual_memory().percent
        self.disk_usage = psutil.disk_usage('/').percent
        
        return self.cpu_usage, self.memory_usage, self.disk_usage

class EmergencyManager:
    """Emergency procedures and safety monitoring"""
    
    def __init__(self):
        self.emergency_state = False
        self.emergency_reason = ""
        self.last_tilt_alert = 0.0
        
    def check_emergency_conditions(self, tilt_angle, system_health):
        """Check for emergency conditions"""
        # Check tilt angle (radians)
        if abs(tilt_angle) > 0.5:  # ~30 degrees
            current_time = time.time()
            if current_time - self.last_tilt_alert > 5.0:  # Throttle alerts
                self.emergency_state = True
                self.emergency_reason = f"Excessive tilt: {tilt_angle:.2f}rad"
                self.last_tilt_alert = current_time
                return True
        
        # Check system health
        if system_health < 0.3:  # Health score threshold
            self.emergency_state = True
            self.emergency_reason = "System health critical"
            return True
            
        self.emergency_state = False
        self.emergency_reason = ""
        return False
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop procedure"""
        return self.emergency_state, self.emergency_reason

class SystemGuardian(Node):
    """Main System Guardian Node"""
    
    def __init__(self):
        super().__init__('system_guardian')
        
        # QoS for system messages
        qos_system = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Initialize monitoring components
        self.thermal_monitor = ThermalMonitor()
        self.power_monitor = PowerMonitor()
        self.resource_manager = ResourceManager()
        self.emergency_manager = EmergencyManager()
        
        # Publishers
        self.thermal_pub = self.create_publisher(Float32, 'system/temperature', qos_system)
        self.power_pub = self.create_publisher(Float32, 'system/battery', qos_system)
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', qos_system)
        self.health_pub = self.create_publisher(Float32, 'system/health', qos_system)
        self.status_pub = self.create_publisher(String, 'system/status', qos_system)
        
        # Subscribers
        self.tilt_sub = self.create_subscription(
            Float32, 'balance_status', self.tilt_callback, 10)
        
        # System monitoring timer (1Hz)
        self.monitor_timer = self.create_timer(1.0, self.monitor_loop)  # 1Hz
        
        # State variables
        self.current_tilt = 0.0
        self.system_health_score = 1.0
        self.performance_mode = 'normal'
        
        self.get_logger().info("🚀 System Guardian initialized")

    def tilt_callback(self, msg):
        """Store current tilt angle for safety checking"""
        self.current_tilt = msg.data

    def monitor_loop(self):
        """Main monitoring loop - runs at 1Hz"""
        try:
            # Monitor thermal conditions
            thermal_mode, temperature = self.thermal_monitor.check_thermal_state()
            
            # Monitor power
            voltage, capacity = self.power_monitor.read_battery()
            
            # Monitor system resources
            cpu_usage, memory_usage, disk_usage = self.resource_manager.get_system_resources()
            
            # Calculate system health score
            self.system_health_score = self._calculate_health_score(
                temperature, voltage, cpu_usage, memory_usage)
            
            # Check for emergency conditions
            emergency, reason = self.emergency_manager.check_emergency_conditions(
                self.current_tilt, self.system_health_score)
            
            # Publish system status
            self._publish_system_status(
                temperature, voltage, capacity, 
                cpu_usage, memory_usage, 
                self.system_health_score, thermal_mode)
            
            # Handle emergency state
            if emergency:
                self._handle_emergency(reason)
            
            # Apply performance scaling based on thermal conditions
            self._apply_performance_scaling(thermal_mode)
            
        except Exception as e:
            self.get_logger().error(f"Monitor loop error: {str(e)}")

    def _calculate_health_score(self, temp, voltage, cpu, memory):
        """Calculate overall system health score (0.0 to 1.0)"""
        # Normalize factors
        temp_score = max(0.0, 1.0 - (temp - 45.0) / 40.0)  # 45°C=1.0, 85°C=0.0
        voltage_score = max(0.0, min(1.0, (voltage - 9.0) / (12.6 - 9.0)))
        cpu_score = max(0.0, 1.0 - cpu / 100.0)
        memory_score = max(0.0, 1.0 - memory / 100.0)
        
        # Weighted average
        health_score = (
            temp_score * 0.3 +
            voltage_score * 0.3 + 
            cpu_score * 0.2 +
            memory_score * 0.2
        )
        
        return max(0.0, min(1.0, health_score))

    def _publish_system_status(self, temp, voltage, capacity, cpu, memory, health, thermal_mode):
        """Publish all system status messages"""
        # Publish temperature
        temp_msg = Float32()
        temp_msg.data = temp
        self.thermal_pub.publish(temp_msg)
        
        # Publish battery
        battery_msg = Float32()
        battery_msg.data = capacity
        self.power_pub.publish(battery_msg)
        
        # Publish health score
        health_msg = Float32()
        health_msg.data = health
        self.health_pub.publish(health_msg)
        
        # Publish detailed status
        status_msg = String()
        status_data = {
            'timestamp': time.time(),
            'temperature': temp,
            'voltage': voltage,
            'battery_capacity': capacity,
            'cpu_usage': cpu,
            'memory_usage': memory,
            'health_score': health,
            'thermal_mode': thermal_mode,
            'tilt_angle': self.current_tilt
        }
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

    def _handle_emergency(self, reason):
        """Handle emergency conditions"""
        self.get_logger().error(f"🚨 EMERGENCY: {reason}")
        
        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        # Additional emergency procedures would go here

    def _apply_performance_scaling(self, thermal_mode):
        """Apply performance scaling based on thermal conditions"""
        if thermal_mode == 'emergency' and self.performance_mode != 'emergency':
            self.get_logger().warn("🔄 Entering emergency performance mode")
            # Would stop non-essential nodes in practice
            self.performance_mode = 'emergency'
            
        elif thermal_mode == 'critical' and self.performance_mode != 'critical':
            self.get_logger().warn("🔄 Entering critical performance mode")
            # Would reduce TOF processing frequency
            self.performance_mode = 'critical'
            
        elif thermal_mode == 'warning' and self.performance_mode != 'warning':
            self.get_logger().info("🔄 Entering warning performance mode")
            # Would reduce non-critical task frequencies
            self.performance_mode = 'warning'
            
        elif thermal_mode == 'normal' and self.performance_mode != 'normal':
            self.get_logger().info("🔄 Returning to normal performance mode")
            self.performance_mode = 'normal'

    def destroy_node(self):
        """Clean shutdown"""
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        system_guardian = SystemGuardian()
        rclpy.spin(system_guardian)
    except KeyboardInterrupt:
        pass
    finally:
        system_guardian.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Power Monitor Library
Battery voltage and current monitoring
"""

import time
import math

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("WARNING: smbus not available - power monitor running in simulation")

class PowerMonitor:
    """Battery power monitoring system"""
    
    def __init__(self, i2c_bus=1, i2c_address=0x40):
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.bus = None
        
        # Battery parameters (3S LiPo)
        self.cell_count = 3
        self.nominal_voltage = 3.7 * self.cell_count  # 11.1V
        self.full_voltage = 4.2 * self.cell_count     # 12.6V
        self.empty_voltage = 3.0 * self.cell_count    # 9.0V
        
        # Current readings
        self.voltage = 12.6
        self.current = 0.0
        self.power = 0.0
        
        # Battery state
        self.capacity_percent = 100.0
        self.remaining_capacity = 2200  # mAh
        self.full_capacity = 2200  # mAh
        
        # Statistics
        self.energy_used = 0.0  # Wh
        self.operation_time = 0.0  # seconds
        self.start_time = time.time()
        
        # Alerts
        self.low_voltage_alert = False
        self.critical_voltage_alert = False
        
        # Initialize I2C
        self._init_i2c()
    
    def _init_i2c(self):
        """Initialize I2C communication for power monitoring"""
        if not I2C_AVAILABLE:
            print("PowerMonitor: Running in simulation mode")
            return True
            
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            # Initialize INA219 or similar power monitor IC
            print("PowerMonitor: I2C initialized for power monitoring")
            return True
        except Exception as e:
            print(f"PowerMonitor: I2C initialization failed - {e}")
            self.bus = None
            return False
    
    def read_power_data(self):
        """Read current power data from sensors"""
        if not self.bus:
            return self._read_simulated_data()
        
        try:
            # Read from INA219 or similar power monitor
            # This is a simplified implementation
            # Actual implementation would depend on your specific hardware
            
            # Simulated read for now
            self.voltage = max(9.0, self.voltage - 0.001)  # Simulate discharge
            self.current = 0.5 + 0.1 * math.sin(time.time())  # Simulate current draw
            self.power = self.voltage * self.current
            
            self._update_battery_state()
            return True
            
        except Exception as e:
            print(f"PowerMonitor: Read error - {e}")
            return False
    
    def _read_simulated_data(self):
        """Generate simulated power data"""
        # Simulate battery discharge over time
        elapsed = time.time() - self.start_time
        discharge_rate = 0.0001  # volts per second
        
        self.voltage = max(9.0, 12.6 - elapsed * discharge_rate)
        self.current = 0.8 + 0.3 * math.sin(elapsed * 0.1)  # Oscillating current
        self.power = self.voltage * self.current
        
        self._update_battery_state()
        return True
    
    def _update_battery_state(self):
        """Update battery state based on current readings"""
        # Calculate capacity percentage
        voltage_range = self.full_voltage - self.empty_voltage
        current_range = self.voltage - self.empty_voltage
        
        if current_range <= 0:
            self.capacity_percent = 0.0
        else:
            self.capacity_percent = min(100.0, (current_range / voltage_range) * 100)
        
        # Update remaining capacity (simplified)
        self.remaining_capacity = (self.capacity_percent / 100.0) * self.full_capacity
        
        # Update energy used (integrate power over time)
        current_time = time.time()
        if hasattr(self, 'last_update_time'):
            dt = current_time - self.last_update_time
            self.energy_used += (self.power * dt) / 3600.0  # Convert to Wh
        self.last_update_time = current_time
        
        self.operation_time = current_time - self.start_time
        
        # Check for alerts
        self._check_alerts()
    
    def _check_alerts(self):
        """Check for low voltage alerts"""
        self.low_voltage_alert = self.voltage < (3.3 * self.cell_count)  # 9.9V for 3S
        self.critical_voltage_alert = self.voltage < (3.0 * self.cell_count)  # 9.0V for 3S
        
        if self.critical_voltage_alert:
            print("POWER CRITICAL: Voltage below safe minimum!")
        elif self.low_voltage_alert:
            print("POWER WARNING: Voltage low!")
    
    def get_voltage(self):
        """Get current battery voltage"""
        return self.voltage
    
    def get_current(self):
        """Get current draw"""
        return self.current
    
    def get_power(self):
        """Get current power consumption"""
        return self.power
    
    def get_capacity_percent(self):
        """Get remaining battery capacity percentage"""
        return self.capacity_percent
    
    def get_remaining_time(self):
        """Estimate remaining operation time in hours"""
        if self.current > 0:
            return (self.remaining_capacity / 1000.0) / self.current  # hours
        return float('inf')
    
    def get_stats(self):
        """Get all power statistics"""
        return {
            'voltage': self.voltage,
            'current': self.current,
            'power': self.power,
            'capacity_percent': self.capacity_percent,
            'remaining_capacity_mah': self.remaining_capacity,
            'energy_used_wh': self.energy_used,
            'operation_time_hours': self.operation_time / 3600.0,
            'remaining_time_hours': self.get_remaining_time(),
            'low_voltage_alert': self.low_voltage_alert,
            'critical_voltage_alert': self.critical_voltage_alert
        }
    
    def reset_stats(self):
        """Reset energy usage statistics"""
        self.energy_used = 0.0
        self.start_time = time.time()
#!/usr/bin/env python3
"""
Thermal Monitor Library
CPU and system temperature monitoring
"""

import time
import os
import psutil

class ThermalMonitor:
    """System thermal monitoring and management"""
    
    def __init__(self):
        self.current_temp = 0.0
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        
        # Temperature thresholds (Celsius)
        self.thresholds = {
            'normal': 65.0,    # Full performance
            'warning': 75.0,   # Reduce non-critical tasks
            'critical': 80.0,  # Disable TOF processing
            'emergency': 85.0  # Shutdown non-essential nodes
        }
        
        # Performance states
        self.performance_modes = {
            'normal': {'tof_fps': 10, 'cloud_rate': 10, 'sensor_rate': 50},
            'warning': {'tof_fps': 5, 'cloud_rate': 5, 'sensor_rate': 30},
            'critical': {'tof_fps': 2, 'cloud_rate': 2, 'sensor_rate': 20},
            'emergency': {'tof_fps': 0, 'cloud_rate': 1, 'sensor_rate': 10}
        }
        
        self.current_mode = 'normal'
        self.last_mode_change = time.time()
        
        # Cooling control
        self.fan_enabled = False
        self.fan_pin = 19  # GPIO pin for cooling fan
        
    def read_temperature(self):
        """Read CPU temperature from system"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_str = f.read().strip()
                self.current_temp = float(temp_str) / 1000.0
            return self.current_temp
        except Exception as e:
            print(f"ThermalMonitor: Temperature read error - {e}")
            # Return safe default temperature
            self.current_temp = 45.0
            return self.current_temp
    
    def read_system_resources(self):
        """Read CPU and memory usage"""
        self.cpu_usage = psutil.cpu_percent(interval=0.1)
        self.memory_usage = psutil.virtual_memory().percent
        return self.cpu_usage, self.memory_usage
    
    def check_thermal_state(self):
        """
        Check current thermal state and return appropriate performance mode
        
        Returns:
            tuple: (performance_mode, temperature, cpu_usage, memory_usage)
        """
        temp = self.read_temperature()
        cpu_usage, memory_usage = self.read_system_resources()
        
        # Determine performance mode based on temperature
        if temp >= self.thresholds['emergency']:
            new_mode = 'emergency'
        elif temp >= self.thresholds['critical']:
            new_mode = 'critical'
        elif temp >= self.thresholds['warning']:
            new_mode = 'warning'
        else:
            new_mode = 'normal'
        
        # Handle mode change
        if new_mode != self.current_mode:
            self._handle_mode_change(new_mode, temp)
        
        return self.current_mode, temp, cpu_usage, memory_usage
    
    def _handle_mode_change(self, new_mode, temperature):
        """Handle performance mode changes"""
        old_mode = self.current_mode
        self.current_mode = new_mode
        self.last_mode_change = time.time()
        
        print(f"ThermalMonitor: Mode change {old_mode} -> {new_mode} "
              f"({temperature:.1f}°C)")
        
        # Enable fan if temperature is high
        if new_mode in ['warning', 'critical', 'emergency']:
            self._enable_fan(True)
        else:
            self._enable_fan(False)
        
        # Take additional actions based on mode
        if new_mode == 'emergency':
            self._emergency_actions()
        elif new_mode == 'critical':
            self._critical_actions()
    
    def _enable_fan(self, enable):
        """Enable or disable cooling fan"""
        if enable != self.fan_enabled:
            self.fan_enabled = enable
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.fan_pin, GPIO.OUT)
                GPIO.output(self.fan_pin, enable)
                print(f"ThermalMonitor: Fan {'enabled' if enable else 'disabled'}")
            except ImportError:
                print(f"ThermalMonitor: Fan control simulated - {'ON' if enable else 'OFF'}")
            except Exception as e:
                print(f"ThermalMonitor: Fan control error - {e}")
    
    def _emergency_actions(self):
        """Actions to take in emergency thermal state"""
        print("THERMAL EMERGENCY: Taking protective actions")
        # In a real implementation, this would signal other nodes to reduce activity
        # or trigger a safe shutdown of non-critical processes
    
    def _critical_actions(self):
        """Actions to take in critical thermal state"""
        print("THERMAL CRITICAL: Reducing system load")
        # Would communicate with other nodes to reduce processing load
    
    def get_performance_settings(self):
        """Get performance settings for current thermal mode"""
        return self.performance_modes[self.current_mode].copy()
    
    def get_thermal_stats(self):
        """Get complete thermal statistics"""
        mode, temp, cpu, memory = self.check_thermal_state()
        settings = self.get_performance_settings()
        
        return {
            'temperature': temp,
            'cpu_usage': cpu,
            'memory_usage': memory,
            'performance_mode': mode,
            'performance_settings': settings,
            'fan_enabled': self.fan_enabled,
            'time_in_mode': time.time() - self.last_mode_change
        }
    
    def set_custom_thresholds(self, normal=None, warning=None, critical=None, emergency=None):
        """Set custom temperature thresholds"""
        if normal is not None:
            self.thresholds['normal'] = normal
        if warning is not None:
            self.thresholds['warning'] = warning
        if critical is not None:
            self.thresholds['critical'] = critical
        if emergency is not None:
            self.thresholds['emergency'] = emergency
        
        print(f"ThermalMonitor: Thresholds updated - {self.thresholds}")
    
    def cleanup(self):
        """Cleanup resources"""
        self._enable_fan(False)
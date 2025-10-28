"""
Spherical Bot - Self-Balancing Robot with Cloud Navigation
"""

__version__ = "1.0.0"
__author__ = "Om Kolse"
__email__ = "omkolse25@gmail.com"
__description__ = "Self-balancing spherical robot with real-time control and cloud navigation"

# Package metadata
PACKAGE_NAME = "spherical_bot"
VERSION = "1.0.0"
ROS_DISTRO = "humble"

# Export main components
from .nodes.core_controller import CoreController
from .nodes.sensor_manager import SensorManager
from .nodes.cloud_bridge import CloudBridge
from .nodes.system_guardian import SystemGuardian

__all__ = [
    'CoreController',
    'SensorManager', 
    'CloudBridge',
    'System_Guardian'
]
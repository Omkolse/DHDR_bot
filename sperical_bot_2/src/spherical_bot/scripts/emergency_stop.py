#!/usr/bin/env python3
"""
Spherical Bot - Emergency Stop Script
Immediately stops all robot operations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # Create emergency stop publisher
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.get_logger().error("🚨 EMERGENCY STOP ACTIVATED!")
        
        # Publish emergency stop
        msg = Bool()
        msg.data = True
        self.emergency_pub.publish(msg)
        
        self.get_logger().info("All motors should be stopped now")
        
    def clear_emergency_stop(self):
        """Clear emergency stop"""
        self.get_logger().info("🟢 Clearing emergency stop...")
        
        msg = Bool()
        msg.data = False
        self.emergency_pub.publish(msg)
        
        self.get_logger().info("Emergency stop cleared")

def main():
    rclpy.init()
    
    import sys
    
    try:
        node = EmergencyStopNode()
        
        if len(sys.argv) > 1 and sys.argv[1] == '--clear':
            node.clear_emergency_stop()
        else:
            node.trigger_emergency_stop()
            
        # Give time for message to be published
        time.sleep(1)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
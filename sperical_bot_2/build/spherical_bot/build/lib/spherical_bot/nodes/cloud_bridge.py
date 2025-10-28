#!/usr/bin/env python3
"""
Cloud Bridge Node - Oracle Cloud Communication
Runs at 10Hz on Core 1 - Network tolerant
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
import json
import time
import threading
from queue import Queue, Empty
import socket
import asyncio

# Cloud communication libraries
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("WARNING: paho-mqtt not available")

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("WARNING: websockets not available")

class AutoConnector:
    """Automatic connection manager for Oracle Cloud"""
    
    def __init__(self):
        self.connected = False
        self.connection_type = None  # 'mqtt' or 'websocket'
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 1.0  # seconds
        
        # Cloud configuration - you'll need to set these
        self.cloud_config = {
            'mqtt_broker': 'your-oracle-cloud-broker',
            'mqtt_port': 8883,
            'websocket_url': 'wss://your-oracle-cloud-websocket',
            'robot_id': 'spherical_bot_001',
            'username': 'your-username',
            'password': 'your-password'
        }
        
        self.mqtt_client = None
        self.websocket = None
    
    def discover_cloud(self):
        """Discover and connect to Oracle cloud instance"""
        # Try MQTT first
        if self._connect_mqtt():
            self.connection_type = 'mqtt'
            self.connected = True
            return True
        
        # Fallback to WebSocket
        if self._connect_websocket():
            self.connection_type = 'websocket' 
            self.connected = True
            return True
        
        return False
    
    def _connect_mqtt(self):
        """Connect via MQTT with SSL"""
        if not MQTT_AVAILABLE:
            return False
            
        try:
            self.mqtt_client = mqtt.Client(client_id=self.cloud_config['robot_id'])
            self.mqtt_client.username_pw_set(
                self.cloud_config['username'], 
                self.cloud_config['password']
            )
            # Configure SSL for secure connection
            self.mqtt_client.tls_set()
            
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
            
            self.mqtt_client.connect(
                self.cloud_config['mqtt_broker'],
                self.cloud_config['mqtt_port'],
                60
            )
            
            self.mqtt_client.loop_start()
            return True
            
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            return False
    
    def _connect_websocket(self):
        """Connect via WebSocket"""
        if not WEBSOCKETS_AVAILABLE:
            return False
            
        try:
            # This would be implemented with asyncio in production
            # For now, simulate connection
            print("WebSocket connection simulated")
            return True
        except Exception as e:
            print(f"WebSocket connection failed: {e}")
            return False
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("✅ Connected to Oracle Cloud via MQTT")
            self.connected = True
            self.reconnect_attempts = 0
            # Subscribe to command topics
            client.subscribe(f"robot/{self.cloud_config['robot_id']}/commands")
        else:
            print(f"MQTT connection failed with code: {rc}")
            self.connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        print("❌ Disconnected from Oracle Cloud")
        self.connected = False
        self._handle_reconnection()
    
    def _handle_reconnection(self):
        """Handle reconnection with exponential backoff"""
        if self.reconnect_attempts < self.max_reconnect_attempts:
            delay = self.reconnect_delay * (2 ** self.reconnect_attempts)
            print(f"Reconnecting in {delay} seconds...")
            time.sleep(delay)
            self.reconnect_attempts += 1
            self.discover_cloud()
        else:
            print("Max reconnection attempts reached")

class DataCompressor:
    """Compress sensor data for cloud transmission"""
    
    def __init__(self):
        self.compression_enabled = True
    
    def compress_telemetry(self, imu_data, odom_data, system_status):
        """Compress telemetry data"""
        telemetry = {
            'timestamp': time.time(),
            'imu': {
                'accel': imu_data['accel'],
                'gyro': imu_data['gyro'],
                'mag': imu_data['mag']
            },
            'odometry': {
                'x': odom_data['x'],
                'y': odom_data['y'],
                'theta': odom_data['theta']
            },
            'system': system_status
        }
        
        if self.compression_enabled:
            return json.dumps(telemetry).encode('utf-8')
        else:
            return json.dumps(telemetry)
    
    def compress_pointcloud(self, pointcloud_data):
        """Compress point cloud data"""
        # Simple compression - in practice, use proper point cloud compression
        compressed = {
            'type': 'compressed_pointcloud',
            'data': pointcloud_data,
            'original_size': len(str(pointcloud_data)),
            'compressed_size': len(str(pointcloud_data)) // 2,  # Simulated
            'timestamp': time.time()
        }
        return json.dumps(compressed)

class CloudBridge(Node):
    """Main Cloud Bridge Node"""
    
    def __init__(self):
        super().__init__('cloud_bridge')
        
        # QoS for cloud communication
        qos_cloud = QoSProfile(
            depth=50,  # Larger buffer for network delays
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Initialize components
        self.connector = AutoConnector()
        self.compressor = DataCompressor()
        self.data_queue = Queue(maxsize=100)
        
        # Subscribers for sensor data
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, qos_cloud)
        self.odom_sub = self.create_subscription(
            Odometry, 'odometry/wheel', self.odom_callback, qos_cloud)
        self.tof_sub = self.create_subscription(
            PointCloud2, 'tof/points', self.tof_callback, qos_cloud)
        
        # Publisher for cloud commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_cloud)
        
        # Cloud communication timer (10Hz)
        self.cloud_timer = self.create_timer(0.1, self.cloud_loop)  # 10Hz
        
        # State variables
        self.last_imu_data = None
        self.last_odom_data = None
        self.last_system_status = {
            'battery_level': 12.6,
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'temperature': 45.0
        }
        
        # Connect to cloud
        if self.connector.discover_cloud():
            self.get_logger().info("✅ Cloud connection established")
        else:
            self.get_logger().warn("❌ Cloud connection failed - operating offline")
        
        self.get_logger().info("🚀 Cloud Bridge initialized")

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.last_imu_data = {
            'accel': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y, 
                msg.linear_acceleration.z
            ],
            'gyro': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'mag': [0.0, 0.0, 0.0]  # Would come from actual magnetometer
        }

    def odom_callback(self, msg):
        """Store latest odometry data"""
        self.last_odom_data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': 0.0  # Would calculate from orientation
        }

    def tof_callback(self, msg):
        """Queue TOF data for cloud transmission"""
        try:
            # Compress and queue point cloud data
            compressed_data = self.compressor.compress_pointcloud(msg)
            self.data_queue.put_nowait(('pointcloud', compressed_data))
        except Exception as e:
            self.get_logger().warn(f"Failed to queue TOF data: {e}")

    def cloud_loop(self):
        """Main cloud communication loop - runs at 10Hz"""
        try:
            if not self.connector.connected:
                # Try to reconnect periodically
                if int(time.time()) % 10 == 0:  # Every 10 seconds
                    self.connector.discover_cloud()
                return
            
            # Send telemetry data
            if self.last_imu_data and self.last_odom_data:
                telemetry_data = self.compressor.compress_telemetry(
                    self.last_imu_data,
                    self.last_odom_data, 
                    self.last_system_status
                )
                
                if self.connector.connection_type == 'mqtt' and self.connector.mqtt_client:
                    self.connector.mqtt_client.publish(
                        f"robot/{self.connector.cloud_config['robot_id']}/telemetry",
                        telemetry_data
                    )
            
            # Send queued data (point clouds, etc.)
            self._process_data_queue()
            
        except Exception as e:
            self.get_logger().error(f"Cloud loop error: {str(e)}")

    def _process_data_queue(self):
        """Process queued data for cloud transmission"""
        try:
            # Process up to 5 items per cycle to avoid blocking
            for _ in range(5):
                data_type, data = self.data_queue.get_nowait()
                
                if (self.connector.connected and 
                    self.connector.connection_type == 'mqtt' and 
                    self.connector.mqtt_client):
                    
                    topic = f"robot/{self.connector.cloud_config['robot_id']}/{data_type}"
                    self.connector.mqtt_client.publish(topic, data)
                    
        except Empty:
            pass  # Queue is empty, that's fine
        except Exception as e:
            self.get_logger().warn(f"Data queue processing error: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        if (self.connector.mqtt_client and 
            self.connector.connection_type == 'mqtt'):
            self.connector.mqtt_client.disconnect()
            self.connector.mqtt_client.loop_stop()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        cloud_bridge = CloudBridge()
        rclpy.spin(cloud_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        cloud_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
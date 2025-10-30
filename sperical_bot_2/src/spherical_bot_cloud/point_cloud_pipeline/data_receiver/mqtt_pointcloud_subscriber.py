# spherical_bot_cloud/point_cloud_pipeline/data_receiver/mqtt_pointcloud_subscriber.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import zlib
import base64
from threading import Lock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud

class MQTTPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('mqtt_pointcloud_subscriber')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/pointcloud/compressed')
        self.declare_parameter('robot_id', 'spherical_bot_001')
        
        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.robot_id = self.get_parameter('robot_id').value
        
        # ROS2 Publishers
        self.compressed_pub = self.create_publisher(CompressedPointCloud, 'pointcloud/compressed', 10)
        self.raw_pub = self.create_publisher(PointCloud2, 'pointcloud/raw', 10)
        
        # MQTT Client
        self.mqtt_client = mqtt.Client(client_id=f"cloud_receiver_{self.robot_id}")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # Connection management
        self.is_connected = False
        self.connection_lock = Lock()
        
        self.get_logger().info(f'Starting MQTT PointCloud Subscriber for robot {self.robot_id}')
        self._connect_mqtt()
    
    def _connect_mqtt(self):
        """Connect to MQTT broker with retry logic"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Connected to MQTT broker {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')
            # Retry after 5 seconds
            self.create_timer(5.0, self._connect_mqtt)
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        with self.connection_lock:
            if rc == 0:
                topic = f"{self.mqtt_topic}/{self.robot_id}"
                client.subscribe(topic)
                self.is_connected = True
                self.get_logger().info(f'MQTT connected, subscribed to {topic}')
            else:
                self.get_logger().error(f'MQTT connection failed with code: {rc}')
                self.is_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        with self.connection_lock:
            self.is_connected = False
            if rc != 0:
                self.get_logger().warning('MQTT unexpected disconnect, reconnecting...')
                self._connect_mqtt()
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Validate message structure
            if not self._validate_message(payload):
                self.get_logger().warning('Invalid message structure received')
                return
            
            # Create CompressedPointCloud message
            compressed_msg = CompressedPointCloud()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = payload.get('frame_id', 'tof_camera')
            compressed_msg.robot_id = self.robot_id
            compressed_msg.compression_type = payload.get('compression_type', 'rle_delta')
            compressed_msg.point_count = payload.get('point_count', 0)
            compressed_msg.original_size = payload.get('original_size', 0)
            compressed_msg.compressed_size = payload.get('compressed_size', 0)
            
            # Decode base64 compressed data
            compressed_data = base64.b64decode(payload['data'])
            compressed_msg.data = list(compressed_data)
            
            # Publish compressed message for further processing
            self.compressed_pub.publish(compressed_msg)
            
            self.get_logger().debug(
                f'Received compressed pointcloud: {compressed_msg.point_count} points, '
                f'compression: {compressed_msg.compressed_size}/{compressed_msg.original_size} bytes'
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def _validate_message(self, payload):
        """Validate incoming message structure"""
        required_fields = ['data', 'point_count', 'compression_type']
        return all(field in payload for field in required_fields)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MQTTPointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
MQTT Point Cloud Subscriber
Receives compressed point cloud data from Raspberry Pi via MQTT
"""

import asyncio
import json
import logging
import paho.mqtt.client as mqtt
from typing import Dict, Any, Callable, Optional
import time
from dataclasses import dataclass
import numpy as np

@dataclass
class PointCloudMessage:
    """Point cloud message container"""
    robot_id: str
    timestamp: float
    data: bytes
    compression_type: str
    original_size: int
    compressed_size: int
    sequence_number: int

class MQTTPointCloudSubscriber:
    """MQTT subscriber for receiving point cloud data from robots"""
    
    def __init__(self, host: str = "localhost", port: int = 1883):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.message_queue = asyncio.Queue(maxsize=1000)
        self.robot_sessions: Dict[str, Dict] = {}
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'bytes_received': 0,
            'errors': 0,
            'last_message_time': 0
        }
        
        # Configure logging
        self.logger = logging.getLogger('mqtt_subscriber')
        
    async def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client = mqtt.Client(
                client_id=f"pointcloud_subscriber_{int(time.time())}",
                protocol=mqtt.MQTTv311
            )
            
            # Set callbacks
            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
            self.client.on_disconnect = self._on_disconnect
            
            # Connect
            self.client.connect(self.host, self.port, 60)
            self.client.loop_start()
            
            self.logger.info(f"MQTT subscriber connecting to {self.host}:{self.port}")
            return True
            
        except Exception as e:
            self.logger.error(f"MQTT connection failed: {e}")
            return False
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.connected = True
            self.logger.info("✅ MQTT subscriber connected successfully")
            
            # Subscribe to point cloud topics
            topics = [
                "spherical_bot/+/pointcloud/compressed",
                "spherical_bot/+/telemetry",
                "spherical_bot/+/status"
            ]
            
            for topic in topics:
                client.subscribe(topic, qos=1)
                self.logger.info(f"Subscribed to topic: {topic}")
                
        else:
            self.connected = False
            self.logger.error(f"MQTT connection failed with code: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            self.stats['messages_received'] += 1
            self.stats['bytes_received'] += len(msg.payload)
            self.stats['last_message_time'] = time.time()
            
            topic_parts = msg.topic.split('/')
            robot_id = topic_parts[1] if len(topic_parts) > 1 else "unknown"
            
            if "pointcloud/compressed" in msg.topic:
                asyncio.create_task(self._process_pointcloud_message(robot_id, msg.payload))
            elif "telemetry" in msg.topic:
                self._process_telemetry_message(robot_id, msg.payload)
            elif "status" in msg.topic:
                self._process_status_message(robot_id, msg.payload)
                
        except Exception as e:
            self.stats['errors'] += 1
            self.logger.error(f"Error processing MQTT message: {e}")
    
    async def _process_pointcloud_message(self, robot_id: str, payload: bytes):
        """Process incoming point cloud message"""
        try:
            # Parse message header
            message_data = json.loads(payload.decode('utf-8'))
            
            # Validate message structure
            if not self._validate_pointcloud_message(message_data):
                self.logger.warning(f"Invalid point cloud message from {robot_id}")
                return
            
            # Create message object
            pointcloud_msg = PointCloudMessage(
                robot_id=robot_id,
                timestamp=message_data.get('timestamp', time.time()),
                data=message_data['data'],
                compression_type=message_data.get('compression_type', 'unknown'),
                original_size=message_data.get('original_size', 0),
                compressed_size=message_data.get('compressed_size', len(payload)),
                sequence_number=message_data.get('sequence_number', 0)
            )
            
            # Update robot session
            self._update_robot_session(robot_id, pointcloud_msg)
            
            # Add to processing queue
            await self.message_queue.put(pointcloud_msg)
            
            self.logger.debug(f"Queued point cloud from {robot_id}, "
                            f"seq: {pointcloud_msg.sequence_number}, "
                            f"compression: {pointcloud_msg.compression_ratio:.2f}")
            
        except Exception as e:
            self.stats['errors'] += 1
            self.logger.error(f"Error processing point cloud from {robot_id}: {e}")
    
    def _validate_pointcloud_message(self, message_data: Dict) -> bool:
        """Validate point cloud message structure"""
        required_fields = ['data', 'timestamp']
        return all(field in message_data for field in required_fields)
    
    def _update_robot_session(self, robot_id: str, message: PointCloudMessage):
        """Update robot connection session"""
        if robot_id not in self.robot_sessions:
            self.robot_sessions[robot_id] = {
                'first_seen': time.time(),
                'last_seen': time.time(),
                'total_messages': 0,
                'last_sequence': 0,
                'compression_stats': {}
            }
        
        session = self.robot_sessions[robot_id]
        session['last_seen'] = time.time()
        session['total_messages'] += 1
        
        # Check for sequence gaps
        expected_sequence = session['last_sequence'] + 1
        if message.sequence_number > expected_sequence and session['last_sequence'] > 0:
            gap = message.sequence_number - expected_sequence
            self.logger.warning(f"Sequence gap for {robot_id}: "
                              f"expected {expected_sequence}, got {message.sequence_number} "
                              f"(gap: {gap})")
        
        session['last_sequence'] = message.sequence_number
        
        # Update compression statistics
        comp_type = message.compression_type
        if comp_type not in session['compression_stats']:
            session['compression_stats'][comp_type] = {
                'count': 0,
                'total_original_size': 0,
                'total_compressed_size': 0
            }
        
        stats = session['compression_stats'][comp_type]
        stats['count'] += 1
        stats['total_original_size'] += message.original_size
        stats['total_compressed_size'] += message.compressed_size
    
    def _process_telemetry_message(self, robot_id: str, payload: bytes):
        """Process telemetry messages"""
        try:
            telemetry_data = json.loads(payload.decode('utf-8'))
            self.logger.debug(f"Telemetry from {robot_id}: {telemetry_data.get('timestamp')}")
        except Exception as e:
            self.logger.error(f"Error processing telemetry from {robot_id}: {e}")
    
    def _process_status_message(self, robot_id: str, payload: bytes):
        """Process status messages"""
        try:
            status_data = json.loads(payload.decode('utf-8'))
            self.logger.info(f"Status from {robot_id}: {status_data.get('status', 'unknown')}")
        except Exception as e:
            self.logger.error(f"Error processing status from {robot_id}: {e}")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        self.connected = False
        if rc != 0:
            self.logger.warning("Unexpected MQTT disconnection")
        else:
            self.logger.info("MQTT subscriber disconnected")
    
    async def get_message(self) -> Optional[PointCloudMessage]:
        """Get next point cloud message from queue"""
        try:
            return await asyncio.wait_for(self.message_queue.get(), timeout=1.0)
        except asyncio.TimeoutError:
            return None
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get subscriber statistics"""
        active_robots = {
            robot_id: session for robot_id, session in self.robot_sessions.items()
            if time.time() - session['last_seen'] < 60  # Active in last minute
        }
        
        return {
            'connected': self.connected,
            'active_robots': len(active_robots),
            'total_robots': len(self.robot_sessions),
            'queue_size': self.message_queue.qsize(),
            **self.stats
        }
    
    async def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
            self.logger.info("MQTT subscriber disconnected")

# Async manager for MQTT subscriber
class MQTTSubscriberManager:
    """Manager for MQTT subscriber with async support"""
    
    def __init__(self, host: str = "localhost", port: int = 1883):
        self.subscriber = MQTTPointCloudSubscriber(host, port)
        self.running = False
        
    async def start(self):
        """Start the MQTT subscriber"""
        if await self.subscriber.connect():
            self.running = True
            self.subscriber.logger.info("MQTT subscriber manager started")
        else:
            raise ConnectionError("Failed to connect to MQTT broker")
    
    async def stop(self):
        """Stop the MQTT subscriber"""
        self.running = False
        await self.subscriber.disconnect()
        self.subscriber.logger.info("MQTT subscriber manager stopped")
    
    async def message_generator(self):
        """Async generator for point cloud messages"""
        while self.running:
            message = await self.subscriber.get_message()
            if message:
                yield message
            else:
                await asyncio.sleep(0.01)  # Small sleep to prevent busy waiting

# Example usage
async def main():
    """Example usage of MQTT subscriber"""
    subscriber_manager = MQTTSubscriberManager()
    
    try:
        await subscriber_manager.start()
        
        async for message in subscriber_manager.message_generator():
            print(f"Received point cloud from {message.robot_id}, "
                  f"size: {message.compressed_size} bytes")
                  
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        await subscriber_manager.stop()

if __name__ == "__main__":
    asyncio.run(main())
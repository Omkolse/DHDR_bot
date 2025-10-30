# spherical_bot_cloud/point_cloud_pipeline/data_receiver/websocket_data_handler.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json
import threading
from std_msgs.msg import Header
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud, RobotTelemetry

class WebSocketDataHandler(Node):
    def __init__(self):
        super().__init__('websocket_data_handler')
        
        # Parameters
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 8081)
        self.declare_parameter('robot_id', 'spherical_bot_001')
        
        self.host = self.get_parameter('websocket_host').value
        self.port = self.get_parameter('websocket_port').value
        self.robot_id = self.get_parameter('robot_id').value
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(CompressedPointCloud, 'pointcloud/compressed', 10)
        self.telemetry_pub = self.create_publisher(RobotTelemetry, 'robot/telemetry', 10)
        
        # WebSocket server
        self.clients = set()
        self.server = None
        self.server_thread = None
        
        self.get_logger().info(f'Starting WebSocket server on {self.host}:{self.port}')
        self._start_websocket_server()
    
    def _start_websocket_server(self):
        """Start WebSocket server in a separate thread"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            start_server = websockets.serve(
                self._websocket_handler,
                self.host,
                self.port
            )
            
            self.server = loop.run_until_complete(start_server)
            self.get_logger().info(f'WebSocket server started on {self.host}:{self.port}')
            loop.run_forever()
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
    
    async def _websocket_handler(self, websocket, path):
        """Handle WebSocket connections"""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        self.get_logger().info(f'WebSocket client connected: {client_addr}')
        
        try:
            async for message in websocket:
                await self._process_websocket_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f'WebSocket client disconnected: {client_addr}')
        finally:
            self.clients.discard(websocket)
    
    async def _process_websocket_message(self, websocket, message):
        """Process incoming WebSocket messages"""
        try:
            data = json.loads(message)
            message_type = data.get('type', '')
            
            if message_type == 'pointcloud':
                await self._handle_pointcloud_data(data)
            elif message_type == 'telemetry':
                await self._handle_telemetry_data(data)
            elif message_type == 'command':
                await self._handle_command_ack(data)
            else:
                self.get_logger().warning(f'Unknown message type: {message_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'WebSocket JSON decode error: {e}')
        except Exception as e:
            self.get_logger().error(f'WebSocket message processing error: {e}')
    
    async def _handle_pointcloud_data(self, data):
        """Handle compressed pointcloud data from WebSocket"""
        try:
            msg = CompressedPointCloud()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = data.get('frame_id', 'tof_camera')
            msg.robot_id = data.get('robot_id', self.robot_id)
            msg.compression_type = data.get('compression_type', 'unknown')
            msg.point_count = data.get('point_count', 0)
            msg.original_size = data.get('original_size', 0)
            msg.compressed_size = data.get('compressed_size', 0)
            
            # Convert base64 data to bytes
            compressed_data = data['data'].encode('utf-8')
            msg.data = list(compressed_data)
            
            self.pointcloud_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error handling pointcloud data: {e}')
    
    async def _handle_telemetry_data(self, data):
        """Handle robot telemetry data"""
        try:
            msg = RobotTelemetry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.robot_id = data.get('robot_id', self.robot_id)
            
            # IMU data
            imu_data = data.get('imu', {})
            msg.imu.orientation.x = imu_data.get('orientation_x', 0.0)
            msg.imu.orientation.y = imu_data.get('orientation_y', 0.0)
            msg.imu.orientation.z = imu_data.get('orientation_z', 0.0)
            msg.imu.linear_acceleration.x = imu_data.get('accel_x', 0.0)
            msg.imu.linear_acceleration.y = imu_data.get('accel_y', 0.0)
            msg.imu.linear_acceleration.z = imu_data.get('accel_z', 0.0)
            
            # Odometry
            odom_data = data.get('odometry', {})
            msg.odometry.pose.pose.position.x = odom_data.get('x', 0.0)
            msg.odometry.pose.pose.position.y = odom_data.get('y', 0.0)
            msg.odometry.twist.twist.linear.x = odom_data.get('vx', 0.0)
            msg.odometry.twist.twist.angular.z = odom_data.get('vtheta', 0.0)
            
            # Battery
            msg.battery_voltage = data.get('battery_voltage', 0.0)
            msg.battery_percentage = data.get('battery_percentage', 0.0)
            
            self.telemetry_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error handling telemetry data: {e}')
    
    async def _handle_command_ack(self, data):
        """Handle command acknowledgments from robot"""
        command_id = data.get('command_id', 'unknown')
        status = data.get('status', 'unknown')
        self.get_logger().info(f'Command {command_id} acknowledged with status: {status}')
    
    async def broadcast_to_clients(self, message):
        """Broadcast message to all connected WebSocket clients"""
        if self.clients:
            await asyncio.wait([client.send(message) for client in self.clients])
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.server:
            self.server.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketDataHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
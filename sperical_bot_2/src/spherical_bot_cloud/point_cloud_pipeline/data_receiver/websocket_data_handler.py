#!/usr/bin/env python3
"""
WebSocket Data Handler
Handles real-time WebSocket connections for point cloud data
"""

import asyncio
import json
import logging
import websockets
from typing import Dict, Set, Any, Optional
from dataclasses import dataclass
import time
from enum import Enum

class ClientType(Enum):
    ROBOT = "robot"
    DASHBOARD = "dashboard"
    PROCESSOR = "processor"

@dataclass
class WebSocketClient:
    """WebSocket client information"""
    websocket: websockets.WebSocketServerProtocol
    client_id: str
    client_type: ClientType
    connected_at: float
    last_activity: float

class WebSocketDataHandler:
    """WebSocket server for real-time point cloud data"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self.clients: Dict[str, WebSocketClient] = {}
        self.robot_clients: Set[str] = set()
        self.dashboard_clients: Set[str] = set()
        self.processor_clients: Set[str] = set()
        self.server = None
        self.running = False
        
        # Statistics
        self.stats = {
            'connections_total': 0,
            'messages_sent': 0,
            'messages_received': 0,
            'errors': 0
        }
        
        # Configure logging
        self.logger = logging.getLogger('websocket_handler')
    
    async def start_server(self):
        """Start WebSocket server"""
        try:
            self.server = await websockets.serve(
                self._handle_client,
                self.host,
                self.port
            )
            self.running = True
            self.logger.info(f"WebSocket server started on {self.host}:{self.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start WebSocket server: {e}")
            return False
    
    async def _handle_client(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Handle new WebSocket client connection"""
        client_id = f"client_{int(time.time())}_{id(websocket)}"
        
        try:
            # Wait for client identification
            auth_message = await asyncio.wait_for(websocket.recv(), timeout=10.0)
            auth_data = json.loads(auth_message)
            
            client_type = ClientType(auth_data.get('client_type', 'dashboard'))
            client_name = auth_data.get('client_name', 'unknown')
            
            # Register client
            client = WebSocketClient(
                websocket=websocket,
                client_id=client_id,
                client_type=client_type,
                connected_at=time.time(),
                last_activity=time.time()
            )
            
            self.clients[client_id] = client
            
            # Add to appropriate client set
            if client_type == ClientType.ROBOT:
                self.robot_clients.add(client_id)
                self.logger.info(f"🤖 Robot connected: {client_name} ({client_id})")
            elif client_type == ClientType.DASHBOARD:
                self.dashboard_clients.add(client_id)
                self.logger.info(f"📊 Dashboard connected: {client_name} ({client_id})")
            elif client_type == ClientType.PROCESSOR:
                self.processor_clients.add(client_id)
                self.logger.info(f"⚙️ Processor connected: {client_name} ({client_id})")
            
            self.stats['connections_total'] += 1
            
            # Send connection confirmation
            await self._send_to_client(client_id, {
                'type': 'connection_established',
                'client_id': client_id,
                'timestamp': time.time()
            })
            
            # Handle client messages
            async for message in websocket:
                client.last_activity = time.time()
                await self._handle_client_message(client_id, message)
                
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client {client_id} disconnected")
        except asyncio.TimeoutError:
            self.logger.warning(f"Client {client_id} authentication timeout")
        except Exception as e:
            self.logger.error(f"Error handling client {client_id}: {e}")
            self.stats['errors'] += 1
        finally:
            await self._remove_client(client_id)
    
    async def _handle_client_message(self, client_id: str, message: str):
        """Handle incoming message from client"""
        try:
            data = json.loads(message)
            message_type = data.get('type', 'unknown')
            
            self.stats['messages_received'] += 1
            
            client = self.clients.get(client_id)
            if not client:
                return
            
            if client.client_type == ClientType.ROBOT:
                await self._handle_robot_message(client_id, data)
            elif client.client_type == ClientType.DASHBOARD:
                await self._handle_dashboard_message(client_id, data)
            elif client.client_type == ClientType.PROCESSOR:
                await self._handle_processor_message(client_id, data)
                
        except json.JSONDecodeError:
            self.logger.error(f"Invalid JSON from client {client_id}")
        except Exception as e:
            self.logger.error(f"Error processing message from {client_id}: {e}")
    
    async def _handle_robot_message(self, client_id: str, data: Dict):
        """Handle message from robot client"""
        message_type = data.get('type')
        
        if message_type == 'pointcloud_data':
            # Forward point cloud data to processors and dashboards
            await self._broadcast_to_processors(data)
            await self._broadcast_to_dashboards({
                'type': 'pointcloud_update',
                'robot_id': client_id,
                'timestamp': data.get('timestamp', time.time()),
                'data_size': len(str(data.get('data', '')))
            })
            
        elif message_type == 'telemetry':
            # Forward telemetry to dashboards
            await self._broadcast_to_dashboards({
                'type': 'telemetry_update',
                'robot_id': client_id,
                **data
            })
    
    async def _handle_dashboard_message(self, client_id: str, data: Dict):
        """Handle message from dashboard client"""
        message_type = data.get('type')
        
        if message_type == 'control_command':
            # Forward control commands to robots
            robot_id = data.get('robot_id')
            if robot_id and robot_id in self.robot_clients:
                await self._send_to_client(robot_id, data)
    
    async def _handle_processor_message(self, client_id: str, data: Dict):
        """Handle message from processor client"""
        message_type = data.get('type')
        
        if message_type == 'processed_pointcloud':
            # Forward processed point clouds to dashboards
            await self._broadcast_to_dashboards(data)
    
    async def _send_to_client(self, client_id: str, data: Dict):
        """Send data to specific client"""
        try:
            client = self.clients.get(client_id)
            if client and client.websocket.open:
                await client.websocket.send(json.dumps(data))
                self.stats['messages_sent'] += 1
        except Exception as e:
            self.logger.error(f"Error sending to client {client_id}: {e}")
    
    async def _broadcast_to_dashboards(self, data: Dict):
        """Broadcast data to all dashboard clients"""
        tasks = []
        for client_id in self.dashboard_clients:
            tasks.append(self._send_to_client(client_id, data))
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def _broadcast_to_processors(self, data: Dict):
        """Broadcast data to all processor clients"""
        tasks = []
        for client_id in self.processor_clients:
            tasks.append(self._send_to_client(client_id, data))
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def _broadcast_to_robots(self, data: Dict):
        """Broadcast data to all robot clients"""
        tasks = []
        for client_id in self.robot_clients:
            tasks.append(self._send_to_client(client_id, data))
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def _remove_client(self, client_id: str):
        """Remove client from tracking"""
        client = self.clients.pop(client_id, None)
        if client:
            self.robot_clients.discard(client_id)
            self.dashboard_clients.discard(client_id)
            self.processor_clients.discard(client_id)
            
            if client.client_type == ClientType.ROBOT:
                self.logger.info(f"🤖 Robot disconnected: {client_id}")
            elif client.client_type == ClientType.DASHBOARD:
                self.logger.info(f"📊 Dashboard disconnected: {client_id}")
            elif client.client_type == ClientType.PROCESSOR:
                self.logger.info(f"⚙️ Processor disconnected: {client_id}")
    
    async def send_pointcloud_to_dashboards(self, pointcloud_data: Dict):
        """Send processed point cloud data to all dashboards"""
        await self._broadcast_to_dashboards({
            'type': 'pointcloud_processed',
            'timestamp': time.time(),
            'data': pointcloud_data
        })
    
    async def send_control_to_robot(self, robot_id: str, control_data: Dict):
        """Send control command to specific robot"""
        await self._send_to_client(robot_id, {
            'type': 'control_command',
            'timestamp': time.time(),
            **control_data
        })
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get WebSocket server statistics"""
        return {
            'running': self.running,
            'clients_total': len(self.clients),
            'robots_connected': len(self.robot_clients),
            'dashboards_connected': len(self.dashboard_clients),
            'processors_connected': len(self.processor_clients),
            **self.stats
        }
    
    async def stop_server(self):
        """Stop WebSocket server"""
        self.running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            self.logger.info("WebSocket server stopped")

# WebSocket client for processors
class WebSocketProcessorClient:
    """WebSocket client for point cloud processors"""
    
    def __init__(self, server_uri: str, client_name: str = "processor"):
        self.server_uri = server_uri
        self.client_name = client_name
        self.websocket = None
        self.connected = False
        
        self.logger = logging.getLogger('websocket_processor')
    
    async def connect(self):
        """Connect to WebSocket server"""
        try:
            self.websocket = await websockets.connect(self.server_uri)
            
            # Send authentication
            auth_message = {
                'client_type': 'processor',
                'client_name': self.client_name,
                'timestamp': time.time()
            }
            await self.websocket.send(json.dumps(auth_message))
            
            # Wait for confirmation
            response = await self.websocket.recv()
            response_data = json.loads(response)
            
            if response_data.get('type') == 'connection_established':
                self.connected = True
                self.logger.info(f"Processor client connected to {self.server_uri}")
                return True
            else:
                self.logger.error("Failed to authenticate with WebSocket server")
                return False
                
        except Exception as e:
            self.logger.error(f"Failed to connect to WebSocket server: {e}")
            return False
    
    async def send_processed_data(self, processed_data: Dict):
        """Send processed point cloud data to server"""
        if self.connected and self.websocket:
            message = {
                'type': 'processed_pointcloud',
                'timestamp': time.time(),
                'data': processed_data
            }
            await self.websocket.send(json.dumps(message))
    
    async def receive_messages(self):
        """Receive messages from server"""
        try:
            async for message in self.websocket:
                yield json.loads(message)
        except websockets.exceptions.ConnectionClosed:
            self.connected = False
            self.logger.info("WebSocket connection closed")
    
    async def disconnect(self):
        """Disconnect from WebSocket server"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            self.logger.info("Processor client disconnected")

async def main():
    """Example usage"""
    # Start WebSocket server
    handler = WebSocketDataHandler()
    await handler.start_server()
    
    try:
        # Keep server running
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        await handler.stop_server()

if __name__ == "__main__":
    asyncio.run(main())
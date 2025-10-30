# spherical_bot_cloud/point_cloud_pipeline/decompression_engine/delta_decoder.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import struct
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud
from sensor_msgs.msg import PointCloud2, PointField

class DeltaDecoder(Node):
    def __init__(self):
        super().__init__('delta_decoder')
        
        # Subscription
        self.subscription = self.create_subscription(
            CompressedPointCloud,
            'pointcloud/compressed',
            self.decode_delta,
            10
        )
        
        # Publisher
        self.decompressed_pub = self.create_publisher(PointCloud2, 'pointcloud/decompressed/delta', 10)
        
        self.get_logger().info('Delta Decoder started')
    
    def decode_delta(self, msg):
        """Decode delta-encoded pointcloud data"""
        if msg.compression_type not in ['delta', 'rle_delta']:
            return
        
        try:
            compressed_data = bytes(msg.data)
            decompressed_data = self._decode_delta_compression(compressed_data)
            
            pointcloud_msg = self._create_pointcloud_msg(decompressed_data, msg)
            self.decompressed_pub.publish(pointcloud_msg)
            
            self.get_logger().debug(
                f'Delta decoded: {len(decompressed_data)//12} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Delta decoding error: {e}')
    
    def _decode_delta_compression(self, data):
        """Decode delta compression"""
        decompressed = bytearray()
        
        if len(data) < 16:  # Need at least 4 floats
            return decompressed
        
        # Read base point (absolute coordinates)
        base_x, base_y, base_z = struct.unpack('fff', data[0:12])
        decompressed.extend(data[0:12])
        
        # Process delta-encoded points
        i = 12
        last_x, last_y, last_z = base_x, base_y, base_z
        
        while i + 12 <= len(data):
            # Read delta values
            dx, dy, dz = struct.unpack('fff', data[i:i+12])
            i += 12
            
            # Apply delta to get absolute coordinates
            x = last_x + dx
            y = last_y + dy
            z = last_z + dz
            
            last_x, last_y, last_z = x, y, z
            
            # Add to decompressed data
            point_bytes = struct.pack('fff', x, y, z)
            decompressed.extend(point_bytes)
        
        return bytes(decompressed)
    
    def _create_pointcloud_msg(self, data, original_msg):
        """Create PointCloud2 message"""
        msg = PointCloud2()
        msg.header = original_msg.header
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = len(data)
        msg.data = data
        msg.is_dense = True
        
        point_count = len(data) // msg.point_step
        msg.height = 1
        msg.width = point_count
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = DeltaDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
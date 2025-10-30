# spherical_bot_cloud/point_cloud_pipeline/decompression_engine/run_length_decoder.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import struct
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud
from sensor_msgs.msg import PointCloud2, PointField

class RunLengthDecoder(Node):
    def __init__(self):
        super().__init__('run_length_decoder')
        
        # Subscription to compressed pointcloud
        self.subscription = self.create_subscription(
            CompressedPointCloud,
            'pointcloud/compressed',
            self.decode_rle,
            10
        )
        
        # Publisher for decompressed pointcloud
        self.decompressed_pub = self.create_publisher(PointCloud2, 'pointcloud/decompressed/rle', 10)
        
        self.get_logger().info('Run-Length Decoder started')
    
    def decode_rle(self, msg):
        """Decode Run-Length Encoded pointcloud data"""
        if msg.compression_type not in ['rle', 'rle_delta']:
            return  # Skip non-RLE compressed data
        
        try:
            # Convert message data to bytes
            compressed_data = bytes(msg.data)
            
            # Decode RLE
            if msg.compression_type == 'rle':
                decompressed_data = self._decode_simple_rle(compressed_data)
            else:  # rle_delta
                decompressed_data = self._decode_rle_delta(compressed_data)
            
            # Create PointCloud2 message
            pointcloud_msg = self._create_pointcloud_msg(decompressed_data, msg)
            
            # Publish decompressed pointcloud
            self.decompressed_pub.publish(pointcloud_msg)
            
            self.get_logger().debug(
                f'RLE decoded: {len(decompressed_data)//12} points '
                f'from {len(compressed_data)} bytes'
            )
            
        except Exception as e:
            self.get_logger().error(f'RLE decoding error: {e}')
    
    def _decode_simple_rle(self, data):
        """Decode simple Run-Length Encoding"""
        decompressed = bytearray()
        i = 0
        
        while i < len(data):
            if i + 16 > len(data):  # Need at least 4 floats + count
                break
                
            # Read count (4 bytes)
            count = struct.unpack('I', data[i:i+4])[0]
            i += 4
            
            # Read point data (12 bytes: 3 floats)
            point_data = data[i:i+12]
            i += 12
            
            # Repeat point data 'count' times
            for _ in range(count):
                decompressed.extend(point_data)
        
        return bytes(decompressed)
    
    def _decode_rle_delta(self, data):
        """Decode RLE with delta encoding"""
        decompressed = bytearray()
        i = 0
        last_point = None
        
        while i < len(data):
            if i + 16 > len(data):
                break
                
            # Read count
            count = struct.unpack('I', data[i:i+4])[0]
            i += 4
            
            # Read delta values (3 floats)
            dx, dy, dz = struct.unpack('fff', data[i:i+12])
            i += 12
            
            if last_point is None:
                # First point, use absolute coordinates
                x, y, z = dx, dy, dz
            else:
                # Apply delta to last point
                x = last_point[0] + dx
                y = last_point[1] + dy
                z = last_point[2] + dz
            
            last_point = (x, y, z)
            
            # Repeat point 'count' times
            point_bytes = struct.pack('fff', x, y, z)
            for _ in range(count):
                decompressed.extend(point_bytes)
        
        return bytes(decompressed)
    
    def _create_pointcloud_msg(self, data, original_msg):
        """Create PointCloud2 message from decompressed data"""
        msg = PointCloud2()
        msg.header = original_msg.header
        msg.header.frame_id = original_msg.header.frame_id
        
        # Set point cloud fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = len(data)
        msg.data = data
        msg.is_dense = True
        
        point_count = len(data) // msg.point_step
        msg.height = 1
        msg.width = point_count
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = RunLengthDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
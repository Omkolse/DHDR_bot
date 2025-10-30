# spherical_bot_cloud/point_cloud_pipeline/decompression_engine/format_converter.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class FormatConverter(Node):
    def __init__(self):
        super().__init__('format_converter')
        
        # Subscriptions to various decompressed formats
        self.subscriptions = []
        
        topics = [
            ('pointcloud/decompressed/rle', self.convert_rle),
            ('pointcloud/decompressed/delta', self.convert_delta),
            ('pointcloud/upsampled', self.convert_upsampled),
        ]
        
        for topic, callback in topics:
            sub = self.create_subscription(PointCloud2, topic, callback, 10)
            self.subscriptions.append(sub)
        
        # Publisher for standardized pointcloud
        self.standard_pub = self.create_publisher(PointCloud2, 'pointcloud/standard', 10)
        
        self.get_logger().info('Format Converter started')
    
    def convert_rle(self, msg):
        """Convert RLE-decompressed pointcloud to standard format"""
        self._convert_to_standard(msg, 'rle')
    
    def convert_delta(self, msg):
        """Convert delta-decompressed pointcloud to standard format"""
        self._convert_to_standard(msg, 'delta')
    
    def convert_upsampled(self, msg):
        """Convert upsampled pointcloud to standard format"""
        self._convert_to_standard(msg, 'upsampled')
    
    def _convert_to_standard(self, msg, source_format):
        """Convert any pointcloud to standard format"""
        try:
            # Ensure proper field structure
            standardized_msg = self._standardize_pointcloud(msg)
            
            # Add metadata to header (optional)
            standardized_msg.header.frame_id = f"{msg.header.frame_id}_{source_format}"
            
            self.standard_pub.publish(standardized_msg)
            
            self.get_logger().debug(
                f'Converted {source_format}: {standardized_msg.width} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Format conversion error: {e}')
    
    def _standardize_pointcloud(self, msg):
        """Ensure pointcloud has standard field structure"""
        standardized = PointCloud2()
        standardized.header = msg.header
        standardized.header.stamp = self.get_clock().now().to_msg()
        
        # Standard fields: x, y, z
        standardized.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        standardized.is_bigendian = False
        standardized.point_step = 12
        standardized.is_dense = True
        
        # If the incoming message already has the right format, use it directly
        if (len(msg.fields) == 3 and 
            all(f.name in ['x', 'y', 'z'] for f in msg.fields) and
            msg.point_step == 12):
            
            standardized.data = msg.data
            standardized.width = msg.width
            standardized.height = msg.height
            standardized.row_step = msg.row_step
            
        else:
            # Convert to standard format
            standardized.data, point_count = self._convert_data_to_standard(msg)
            standardized.width = point_count
            standardized.height = 1
            standardized.row_step = len(standardized.data)
        
        return standardized
    
    def _convert_data_to_standard(self, msg):
        """Convert pointcloud data to standard x,y,z format"""
        points = []
        
        try:
            # Extract points based on field structure
            for i in range(0, len(msg.data), msg.point_step):
                point_data = msg.data[i:i+msg.point_step]
                
                x, y, z = self._extract_xyz_from_fields(point_data, msg.fields, msg.point_step)
                
                if x is not None and y is not None and z is not None:
                    points.append((x, y, z))
            
            # Convert to standard format
            standardized_data = bytearray()
            for x, y, z in points:
                standardized_data.extend(struct.pack('fff', x, y, z))
            
            return bytes(standardized_data), len(points)
            
        except Exception as e:
            self.get_logger().error(f'Data conversion error: {e}')
            return b'', 0
    
    def _extract_xyz_from_fields(self, point_data, fields, point_step):
        """Extract x, y, z coordinates from point data based on field definitions"""
        x = y = z = None
        
        for field in fields:
            if field.offset + field.count * 4 > point_step:
                continue
                
            if field.datatype != PointField.FLOAT32:
                continue
                
            # Extract value based on field offset
            value = struct.unpack('f', point_data[field.offset:field.offset+4])[0]
            
            if field.name == 'x':
                x = value
            elif field.name == 'y':
                y = value
            elif field.name == 'z':
                z = value
        
        return x, y, z

def main(args=None):
    rclpy.init(args=args)
    node = FormatConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
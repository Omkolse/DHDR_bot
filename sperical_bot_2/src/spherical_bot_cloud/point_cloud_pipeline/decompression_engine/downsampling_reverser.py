# spherical_bot_cloud/point_cloud_pipeline/decompression_engine/downsampling_reverser.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud
from sensor_msgs.msg import PointCloud2, PointField

class DownsamplingReverser(Node):
    def __init__(self):
        super().__init__('downsampling_reverser')
        
        self.subscription = self.create_subscription(
            CompressedPointCloud,
            'pointcloud/compressed',
            self.upsample_pointcloud,
            10
        )
        
        self.upsampled_pub = self.create_publisher(PointCloud2, 'pointcloud/upsampled', 10)
        
        # Upsampling parameters
        self.declare_parameter('upsampling_factor', 2.0)
        self.upsampling_factor = self.get_parameter('upsampling_factor').value
        
        self.get_logger().info(f'Downsampling Reverser started (factor: {self.upsampling_factor})')
    
    def upsample_pointcloud(self, msg):
        """Upsample downsampled pointcloud data"""
        if msg.compression_type != 'downsampled':
            return
        
        try:
            # Convert to numpy array
            points = self._bytes_to_points(bytes(msg.data))
            
            if len(points) == 0:
                return
            
            # Perform upsampling
            upsampled_points = self._upsample_points(points)
            
            # Create PointCloud2 message
            pointcloud_msg = self._create_pointcloud_msg(upsampled_points, msg)
            
            self.upsampled_pub.publish(pointcloud_msg)
            
            self.get_logger().debug(
                f'Upsampled: {len(points)} -> {len(upsampled_points)} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Upsampling error: {e}')
    
    def _bytes_to_points(self, data):
        """Convert byte data to numpy array of points"""
        if len(data) % 12 != 0:
            self.get_logger().warning('Invalid point data length')
            return np.array([])
        
        point_count = len(data) // 12
        points = np.frombuffer(data, dtype=np.float32).reshape(point_count, 3)
        return points
    
    def _upsample_points(self, points):
        """Upsample points using interpolation"""
        if self.upsampling_factor <= 1.0:
            return points
        
        if len(points) < 2:
            return points  # Not enough points to interpolate
        
        # Simple linear interpolation between points
        upsampled = []
        
        for i in range(len(points) - 1):
            # Add original point
            upsampled.append(points[i])
            
            # Add interpolated points
            for j in range(1, int(self.upsampling_factor)):
                alpha = j / self.upsampling_factor
                interpolated = points[i] + alpha * (points[i+1] - points[i])
                upsampled.append(interpolated)
        
        # Add last point
        upsampled.append(points[-1])
        
        return np.array(upsampled)
    
    def _create_pointcloud_msg(self, points, original_msg):
        """Create PointCloud2 message from numpy points"""
        msg = PointCloud2()
        msg.header = original_msg.header
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.data = points.astype(np.float32).tobytes()
        msg.is_dense = True
        
        msg.height = 1
        msg.width = len(points)
        msg.row_step = len(msg.data)
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = DownsamplingReverser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
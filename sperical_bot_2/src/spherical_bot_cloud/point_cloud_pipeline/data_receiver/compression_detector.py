# spherical_bot_cloud/point_cloud_pipeline/data_receiver/compression_detector.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud

class CompressionDetector(Node):
    def __init__(self):
        super().__init__('compression_detector')
        
        # Subscription to compressed pointcloud
        self.subscription = self.create_subscription(
            CompressedPointCloud,
            'pointcloud/compressed',
            self.detect_compression_type,
            10
        )
        
        # Publisher for detected compression info
        self.detection_pub = self.create_publisher(CompressedPointCloud, 'pointcloud/detected', 10)
        
        # Compression type statistics
        self.compression_stats = {}
        self.get_logger().info('Compression Detector started')
    
    def detect_compression_type(self, msg):
        """Detect and validate compression type"""
        detected_type = self._analyze_compression_type(msg)
        
        if detected_type != msg.compression_type:
            self.get_logger().warning(
                f'Compression type mismatch: claimed={msg.compression_type}, '
                f'detected={detected_type}'
            )
            msg.compression_type = detected_type
        
        # Update statistics
        self._update_statistics(detected_type, len(msg.data))
        
        # Publish with detected type
        self.detection_pub.publish(msg)
        
        self.get_logger().debug(
            f'Detected compression: {detected_type}, '
            f'points: {msg.point_count}, '
            f'ratio: {self._calculate_compression_ratio(msg):.2f}'
        )
    
    def _analyze_compression_type(self, msg):
        """Analyze data to detect compression type"""
        data = bytes(msg.data)
        
        # Check for RLE patterns (repeated bytes)
        if self._is_rle_compressed(data):
            return 'rle'
        
        # Check for delta encoding (small value differences)
        if self._is_delta_compressed(data):
            return 'delta'
        
        # Check for combined RLE + Delta
        if self._is_rle_delta_compressed(data):
            return 'rle_delta'
        
        # Check for simple downsampling
        if len(data) < msg.original_size * 0.3:
            return 'downsampled'
        
        return 'unknown'
    
    def _is_rle_compressed(self, data):
        """Check for Run-Length Encoding patterns"""
        if len(data) < 4:
            return False
        
        # RLE often has repeated byte sequences
        repeat_count = 0
        for i in range(1, len(data)):
            if data[i] == data[i-1]:
                repeat_count += 1
            else:
                repeat_count = 0
            
            if repeat_count > 10:  # Found significant repetition
                return True
        
        return False
    
    def _is_delta_compressed(self, data):
        """Check for Delta encoding"""
        if len(data) < 8:
            return False
        
        # Delta encoding typically has small value differences
        try:
            # Try to interpret as 32-bit floats
            import struct
            values = [struct.unpack('f', data[i:i+4])[0] for i in range(0, len(data)-3, 4)]
            
            if len(values) < 2:
                return False
            
            # Calculate differences
            diffs = [abs(values[i] - values[i-1]) for i in range(1, len(values))]
            avg_diff = sum(diffs) / len(diffs)
            
            # Delta encoding usually has small average differences
            return avg_diff < 1.0
            
        except:
            return False
    
    def _is_rle_delta_compressed(self, data):
        """Check for combined RLE + Delta compression"""
        # This would require more sophisticated analysis
        # For now, check if it has characteristics of both
        return self._is_rle_compressed(data) and self._is_delta_compressed(data)
    
    def _calculate_compression_ratio(self, msg):
        """Calculate compression ratio"""
        if msg.original_size > 0 and msg.compressed_size > 0:
            return msg.original_size / msg.compressed_size
        return 0.0
    
    def _update_statistics(self, compression_type, data_size):
        """Update compression statistics"""
        if compression_type not in self.compression_stats:
            self.compression_stats[compression_type] = {
                'count': 0,
                'total_size': 0
            }
        
        stats = self.compression_stats[compression_type]
        stats['count'] += 1
        stats['total_size'] += data_size
        
        # Log statistics periodically
        if stats['count'] % 100 == 0:
            avg_size = stats['total_size'] / stats['count']
            self.get_logger().info(
                f'Compression stats - {compression_type}: '
                f'count={stats["count"]}, avg_size={avg_size:.1f} bytes'
            )

def main(args=None):
    rclpy.init(args=args)
    node = CompressionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
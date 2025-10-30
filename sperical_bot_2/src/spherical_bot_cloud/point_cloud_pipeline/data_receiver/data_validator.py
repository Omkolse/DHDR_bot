# spherical_bot_cloud/point_cloud_pipeline/data_receiver/data_validator.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from spherical_bot_cloud_interfaces.msg import CompressedPointCloud

class DataValidator(Node):
    def __init__(self):
        super().__init__('data_validator')
        
        # Subscription
        self.subscription = self.create_subscription(
            CompressedPointCloud,
            'pointcloud/compressed',
            self.validate_pointcloud_data,
            10
        )
        
        # Publishers
        self.valid_pub = self.create_publisher(CompressedPointCloud, 'pointcloud/validated', 10)
        self.invalid_pub = self.create_publisher(CompressedPointCloud, 'pointcloud/invalid', 10)
        
        # Validation statistics
        self.validation_stats = {
            'total_received': 0,
            'valid': 0,
            'invalid': 0,
            'last_validation_time': self.get_clock().now()
        }
        
        self.get_logger().info('Data Validator started')
    
    def validate_pointcloud_data(self, msg):
        """Validate incoming pointcloud data"""
        self.validation_stats['total_received'] += 1
        
        validation_result = self._perform_validation(msg)
        
        if validation_result['is_valid']:
            self.validation_stats['valid'] += 1
            self.valid_pub.publish(msg)
            self.get_logger().debug(f'Valid pointcloud: {msg.point_count} points')
        else:
            self.validation_stats['invalid'] += 1
            self.invalid_pub.publish(msg)
            self.get_logger().warning(
                f'Invalid pointcloud: {validation_result["reason"]}'
            )
        
        # Log statistics periodically
        self._log_statistics()
    
    def _perform_validation(self, msg):
        """Perform comprehensive data validation"""
        checks = [
            self._check_data_size(msg),
            self._check_point_count(msg),
            self._check_timestamp(msg),
            self._check_compression_ratio(msg),
            self._check_data_integrity(msg)
        ]
        
        for check in checks:
            if not check['passed']:
                return {
                    'is_valid': False,
                    'reason': check['reason']
                }
        
        return {'is_valid': True, 'reason': 'All checks passed'}
    
    def _check_data_size(self, msg):
        """Check if data size is reasonable"""
        data_size = len(msg.data)
        
        if data_size == 0:
            return {'passed': False, 'reason': 'Empty data'}
        
        # Expected size range (adjust based on your sensor)
        min_expected = 100  # bytes
        max_expected = 10 * 1024 * 1024  # 10MB
        
        if data_size < min_expected:
            return {'passed': False, 'reason': f'Data too small: {data_size} bytes'}
        
        if data_size > max_expected:
            return {'passed': False, 'reason': f'Data too large: {data_size} bytes'}
        
        return {'passed': True, 'reason': 'Size OK'}
    
    def _check_point_count(self, msg):
        """Validate point count consistency"""
        if msg.point_count <= 0:
            return {'passed': False, 'reason': f'Invalid point count: {msg.point_count}'}
        
        # For typical ToF camera, expect reasonable point count
        max_expected_points = 50000  # Adjust based on your sensor
        
        if msg.point_count > max_expected_points:
            return {'passed': False, 'reason': f'Too many points: {msg.point_count}'}
        
        return {'passed': True, 'reason': 'Point count OK'}
    
    def _check_timestamp(self, msg):
        """Check timestamp validity"""
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        # Check if timestamp is in the future (with tolerance)
        if msg_time > current_time + rclpy.time.Duration(seconds=10):
            return {'passed': False, 'reason': 'Timestamp in future'}
        
        # Check if timestamp is too old (more than 1 minute)
        if msg_time < current_time - rclpy.time.Duration(seconds=60):
            return {'passed': False, 'reason': 'Timestamp too old'}
        
        return {'passed': True, 'reason': 'Timestamp OK'}
    
    def _check_compression_ratio(self, msg):
        """Validate compression ratio"""
        if msg.original_size <= 0 or msg.compressed_size <= 0:
            return {'passed': True, 'reason': 'Size info missing'}  # Not critical
        
        compression_ratio = msg.original_size / msg.compressed_size
        
        # Check for unreasonable compression ratios
        if compression_ratio > 1000:  # Too good to be true
            return {'passed': False, 'reason': f'Unreasonable compression ratio: {compression_ratio:.1f}'}
        
        if compression_ratio < 0.1:  # Compression made it larger?
            return {'passed': False, 'reason': f'Negative compression: {compression_ratio:.1f}'}
        
        return {'passed': True, 'reason': 'Compression ratio OK'}
    
    def _check_data_integrity(self, msg):
        """Basic data integrity checks"""
        try:
            data = bytes(msg.data)
            
            # Check for all zeros (suspicious)
            if all(b == 0 for b in data):
                return {'passed': False, 'reason': 'All zero data'}
            
            # Check for repeated patterns that might indicate errors
            if self._has_suspicious_patterns(data):
                return {'passed': False, 'reason': 'Suspicious data patterns'}
            
            return {'passed': True, 'reason': 'Data integrity OK'}
            
        except Exception as e:
            return {'passed': False, 'reason': f'Data processing error: {e}'}
    
    def _has_suspicious_patterns(self, data):
        """Detect suspicious data patterns"""
        if len(data) < 10:
            return False
        
        # Check for repeating byte sequences (might indicate stuck sensor)
        sample = data[:100]  # Check first 100 bytes
        unique_bytes = len(set(sample))
        
        if unique_bytes < 5:  # Very low diversity
            return True
        
        return False
    
    def _log_statistics(self):
        """Log validation statistics periodically"""
        current_time = self.get_clock().now()
        time_diff = current_time - self.validation_stats['last_validation_time']
        
        if time_diff.nanoseconds > 30 * 1e9:  # Every 30 seconds
            total = self.validation_stats['total_received']
            valid = self.validation_stats['valid']
            invalid = self.validation_stats['invalid']
            
            if total > 0:
                valid_percentage = (valid / total) * 100
                self.get_logger().info(
                    f'Validation stats: {valid}/{total} valid ({valid_percentage:.1f}%)'
                )
            
            self.validation_stats['last_validation_time'] = current_time

def main(args=None):
    rclpy.init(args=args)
    node = DataValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
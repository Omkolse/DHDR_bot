# spherical_bot_cloud/point_cloud_pipeline/noise_removal/temporal_consistency_filter.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class TemporalConsistencyFilter(Node):
    def __init__(self):
        super().__init__('temporal_consistency_filter')
        
        # Parameters
        self.declare_parameter('window_size', 5)
        self.declare_parameter('temporal_threshold', 0.02)
        
        self.window_size = self.get_parameter('window_size').value
        self.temporal_threshold = self.get_parameter('temporal_threshold').value
        
        # Temporal buffer for previous point clouds
        self.point_buffer = deque(maxlen=self.window_size)
        self.time_buffer = deque(maxlen=self.window_size)
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/standard',
            self.apply_temporal_filter,
            10
        )
        
        # Publisher
        self.filtered_pub = self.create_publisher(PointCloud2, 'pointcloud/filtered/temporal', 10)
        
        self.get_logger().info('Temporal Consistency Filter started')
    
    def apply_temporal_filter(self, msg):
        """Apply temporal consistency filtering"""
        try:
            current_points = self._pointcloud2_to_array(msg)
            current_time = self.get_clock().now()
            
            if len(current_points) == 0:
                return
            
            # Add current frame to buffer
            self.point_buffer.append(current_points)
            self.time_buffer.append(current_time)
            
            # Wait until we have enough frames
            if len(self.point_buffer) < self.window_size:
                filtered_msg = self._array_to_pointcloud2(current_points, msg.header)
                self.filtered_pub.publish(filtered_msg)
                return
            
            # Apply temporal filtering
            filtered_points = self._temporal_consistency_filter(current_points)
            filtered_msg = self._array_to_pointcloud2(filtered_points, msg.header)
            
            self.filtered_pub.publish(filtered_msg)
            
            self.get_logger().debug(
                f'Temporal filtering: {len(current_points)} -> {len(filtered_points)} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Temporal filtering error: {e}')
    
    def _temporal_consistency_filter(self, current_points):
        """Apply temporal consistency check"""
        filtered_points = []
        
        for i, point in enumerate(current_points):
            temporal_consistency = self._check_temporal_consistency(point, i)
            
            if temporal_consistency:
                filtered_points.append(point)
        
        return np.array(filtered_points)
    
    def _check_temporal_consistency(self, point, point_index):
        """Check if point is consistent across temporal window"""
        consistent_count = 0
        
        for frame_points in self.point_buffer:
            if len(frame_points) <= point_index:
                continue
            
            # Find closest point in the frame
            distances = np.linalg.norm(frame_points - point, axis=1)
            min_distance = np.min(distances)
            
            if min_distance < self.temporal_threshold:
                consistent_count += 1
        
        # Point is consistent if it appears in majority of frames
        return consistent_count >= (self.window_size // 2)
    
    def _pointcloud2_to_array(self, msg):
        """Convert PointCloud2 to numpy array"""
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points)
    
    def _array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2"""
        return pc2.create_cloud_xyz32(header, points)

def main(args=None):
    rclpy.init(args=args)
    node = TemporalConsistencyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# spherical_bot_cloud/point_cloud_pipeline/noise_removal/depth_edge_filter.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class DepthEdgeFilter(Node):
    def __init__(self):
        super().__init__('depth_edge_filter')
        
        # Parameters
        self.declare_parameter('edge_threshold', 0.05)
        self.declare_parameter('kernel_size', 3)
        
        self.edge_threshold = self.get_parameter('edge_threshold').value
        self.kernel_size = self.get_parameter('kernel_size').value
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/standard',
            self.filter_depth_edges,
            10
        )
        
        # Publisher
        self.filtered_pub = self.create_publisher(PointCloud2, 'pointcloud/filtered/edges', 10)
        
        self.get_logger().info('Depth Edge Filter started')
    
    def filter_depth_edges(self, msg):
        """Remove depth edge artifacts from point cloud"""
        try:
            points = self._pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            filtered_points = self._remove_depth_edges(points)
            filtered_msg = self._array_to_pointcloud2(filtered_points, msg.header)
            
            self.filtered_pub.publish(filtered_msg)
            
            self.get_logger().debug(
                f'Edge filtering: {len(points)} -> {len(filtered_points)} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Edge filtering error: {e}')
    
    def _remove_depth_edges(self, points):
        """Remove depth discontinuity edges"""
        if len(points) < self.kernel_size:
            return points
        
        # Calculate depth values (distance from origin)
        depths = np.linalg.norm(points, axis=1)
        
        # Create a grid-like structure for neighborhood analysis
        # This is simplified - in practice, you'd use the actual sensor pattern
        filtered_indices = []
        
        for i in range(len(points)):
            # Find nearest neighbors
            distances = np.linalg.norm(points - points[i], axis=1)
            neighbor_indices = np.argsort(distances)[:self.kernel_size]
            
            # Calculate depth variance in neighborhood
            neighbor_depths = depths[neighbor_indices]
            depth_variance = np.var(neighbor_depths)
            
            # Keep point if depth variance is below threshold
            if depth_variance < self.edge_threshold:
                filtered_indices.append(i)
        
        return points[filtered_indices]
    
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
    node = DepthEdgeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
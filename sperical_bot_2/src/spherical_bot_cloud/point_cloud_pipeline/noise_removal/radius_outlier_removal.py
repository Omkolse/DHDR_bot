# spherical_bot_cloud/point_cloud_pipeline/noise_removal/radius_outlier_removal.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.neighbors import NearestNeighbors

class RadiusOutlierRemoval(Node):
    def __init__(self):
        super().__init__('radius_outlier_removal')
        
        # Parameters
        self.declare_parameter('radius', 0.1)
        self.declare_parameter('min_neighbors', 5)
        
        self.radius = self.get_parameter('radius').value
        self.min_neighbors = self.get_parameter('min_neighbors').value
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/standard',
            self.remove_radius_outliers,
            10
        )
        
        # Publisher
        self.filtered_pub = self.create_publisher(PointCloud2, 'pointcloud/filtered/radius', 10)
        
        self.get_logger().info('Radius Outlier Removal started')
    
    def remove_radius_outliers(self, msg):
        """Remove outliers based on radius search"""
        try:
            points = self._pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            filtered_points = self._radius_outlier_removal(points)
            filtered_msg = self._array_to_pointcloud2(filtered_points, msg.header)
            
            self.filtered_pub.publish(filtered_msg)
            
            self.get_logger().debug(
                f'Radius filtering: {len(points)} -> {len(filtered_points)} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Radius filtering error: {e}')
    
    def _radius_outlier_removal(self, points):
        """Apply radius-based outlier removal"""
        if len(points) < 2:
            return points
        
        # Find neighbors within radius
        nbrs = NearestNeighbors(radius=self.radius, algorithm='ball_tree').fit(points)
        distances, indices = nbrs.radius_neighbors(points)
        
        # Count neighbors for each point
        neighbor_counts = np.array([len(n) for n in indices])
        
        # Keep points with sufficient neighbors
        inliers = neighbor_counts >= self.min_neighbors
        
        return points[inliers]
    
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
    node = RadiusOutlierRemoval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
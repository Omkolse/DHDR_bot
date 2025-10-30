# spherical_bot_cloud/point_cloud_pipeline/noise_removal/statistical_outlier_removal.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.neighbors import NearestNeighbors

class StatisticalOutlierRemoval(Node):
    def __init__(self):
        super().__init__('statistical_outlier_removal')
        
        # Parameters
        self.declare_parameter('mean_k', 50)
        self.declare_parameter('std_dev_threshold', 1.0)
        
        self.mean_k = self.get_parameter('mean_k').value
        self.std_dev_threshold = self.get_parameter('std_dev_threshold').value
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/standard',
            self.remove_outliers,
            10
        )
        
        # Publisher
        self.filtered_pub = self.create_publisher(PointCloud2, 'pointcloud/filtered/statistical', 10)
        
        self.get_logger().info('Statistical Outlier Removal started')
    
    def remove_outliers(self, msg):
        """Remove statistical outliers from point cloud"""
        try:
            # Convert PointCloud2 to numpy array
            points = self._pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            # Apply statistical outlier removal
            filtered_points = self._statistical_outlier_removal(points)
            
            # Convert back to PointCloud2
            filtered_msg = self._array_to_pointcloud2(filtered_points, msg.header)
            
            self.filtered_pub.publish(filtered_msg)
            
            self.get_logger().debug(
                f'Statistical filtering: {len(points)} -> {len(filtered_points)} points'
            )
            
        except Exception as e:
            self.get_logger().error(f'Statistical filtering error: {e}')
    
    def _statistical_outlier_removal(self, points):
        """Apply statistical outlier removal"""
        if len(points) < self.mean_k:
            return points
        
        # Find k nearest neighbors for each point
        nbrs = NearestNeighbors(n_neighbors=self.mean_k, algorithm='ball_tree').fit(points)
        distances, indices = nbrs.kneighbors(points)
        
        # Calculate mean distance for each point
        mean_distances = np.mean(distances, axis=1)
        
        # Calculate global mean and standard deviation
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        
        # Filter points based on standard deviation threshold
        threshold = global_mean + self.std_dev_threshold * global_std
        inliers = mean_distances < threshold
        
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
    node = StatisticalOutlierRemoval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
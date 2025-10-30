# spherical_bot_cloud/point_cloud_pipeline/noise_removal/adaptive_thresholding.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class AdaptiveThresholding(Node):
    def __init__(self):
        super().__init__('adaptive_thresholding')
        
        # Parameters
        self.declare_parameter('initial_threshold', 0.1)
        self.declare_parameter('learning_rate', 0.1)
        self.declare_parameter('min_points', 100)
        
        self.initial_threshold = self.get_parameter('initial_threshold').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.min_points = self.get_parameter('min_points').value
        
        # Adaptive threshold state
        self.current_threshold = self.initial_threshold
        self.filtering_stats = []
        
        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/standard',
            self.adaptive_filter,
            10
        )
        
        # Publisher
        self.filtered_pub = self.create_publisher(PointCloud2, 'pointcloud/filtered/adaptive', 10)
        
        self.get_logger().info('Adaptive Thresholding started')
    
    def adaptive_filter(self, msg):
        """Apply adaptive thresholding based on scene characteristics"""
        try:
            points = self._pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            # Apply adaptive filtering
            filtered_points, new_threshold = self._adaptive_distance_filter(points)
            filtered_msg = self._array_to_pointcloud2(filtered_points, msg.header)
            
            self.filtered_pub.publish(filtered_msg)
            
            # Update threshold
            self.current_threshold = new_threshold
            
            self.get_logger().debug(
                f'Adaptive filtering: {len(points)} -> {len(filtered_points)} points, '
                f'threshold: {self.current_threshold:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Adaptive filtering error: {e}')
    
    def _adaptive_distance_filter(self, points):
        """Apply adaptive distance-based filtering"""
        if len(points) < self.min_points:
            return points, self.current_threshold
        
        # Calculate distances from centroid
        centroid = np.mean(points, axis=0)
        distances = np.linalg.norm(points - centroid, axis=1)
        
        # Calculate statistics
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        
        # Adaptive threshold based on scene spread
        scene_spread = std_dist / (mean_dist + 1e-6)  # Normalized spread
        
        # Adjust threshold based on scene characteristics
        if scene_spread > 0.5:  # High variance scene
            adaptive_threshold = mean_dist + 2.0 * std_dist
        elif scene_spread < 0.1:  # Low variance scene
            adaptive_threshold = mean_dist + 1.0 * std_dist
        else:  # Medium variance
            adaptive_threshold = mean_dist + 1.5 * std_dist
        
        # Smooth threshold update
        new_threshold = (1 - self.learning_rate) * self.current_threshold + \
                       self.learning_rate * adaptive_threshold
        
        # Apply filtering
        inliers = distances < new_threshold
        filtered_points = points[inliers]
        
        return filtered_points, new_threshold
    
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
    node = AdaptiveThresholding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
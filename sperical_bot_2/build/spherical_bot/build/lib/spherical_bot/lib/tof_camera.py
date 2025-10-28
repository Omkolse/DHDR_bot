#!/usr/bin/env python3
"""
TOF Camera Library
Arducam TOF Camera Interface
Based on official documentation: 
https://docs.arducam.com/Raspberry-Pi-Camera/Tof-camera/ROS-With-Arducam-ToF-Camera/
"""

import numpy as np
import time
import json

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("WARNING: OpenCV not available - TOF camera running in simulation")

class TOFCamera:
    """Arducam TOF Camera Manager"""
    
    def __init__(self):
        self.connected = False
        self.camera = None
        self.frame_width = 240
        self.frame_height = 192
        self.frame_id = "tof_camera"
        
        # Camera parameters (from Arducam docs)
        self.focal_length = 240.0  # pixels
        self.principal_point = (120, 96)  # cx, cy
        
        # Compression settings
        self.compression_enabled = True
        self.downsample_factor = 2
        
    def initialize(self):
        """Initialize TOF camera"""
        if not CV2_AVAILABLE:
            print("TOFCamera: Running in simulation mode")
            self.connected = True
            return True
            
        try:
            # Try to open camera - Arducam TOF might be on a specific index
            for i in range(4):
                self.camera = cv2.VideoCapture(i)
                if self.camera.isOpened():
                    # Set camera properties for TOF
                    self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    self.camera.set(cv2.CAP_PROP_FPS, 10)
                    
                    # Verify camera opened correctly
                    ret, frame = self.camera.read()
                    if ret:
                        self.connected = True
                        print(f"TOFCamera: Initialized on device {i}")
                        return True
                    else:
                        self.camera.release()
            
            print("TOFCamera: No camera device found")
            return False
            
        except Exception as e:
            print(f"TOFCamera: Initialization failed - {e}")
            return False
    
    def get_depth_frame(self):
        """Get depth frame from camera"""
        if not self.connected or not CV2_AVAILABLE:
            return self._get_simulated_depth_frame()
        
        try:
            ret, frame = self.camera.read()
            if not ret:
                print("TOFCamera: Failed to read frame")
                return None
            
            # Convert to depth data (assuming 16-bit depth map)
            if len(frame.shape) == 2:
                depth_frame = frame.astype(np.float32) / 1000.0  # Convert mm to meters
            else:
                # Convert BGR to grayscale as fallback
                depth_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32) / 1000.0
            
            return depth_frame
            
        except Exception as e:
            print(f"TOFCamera: Frame capture error - {e}")
            return None
    
    def depth_to_pointcloud(self, depth_frame):
        """Convert depth frame to 3D point cloud"""
        if depth_frame is None:
            return None
        
        height, width = depth_frame.shape
        
        # Downsample for performance
        if self.downsample_factor > 1:
            depth_frame = depth_frame[::self.downsample_factor, ::self.downsample_factor]
            height = depth_frame.shape[0]
            width = depth_frame.shape[1]
        
        points = []
        
        for v in range(height):
            for u in range(width):
                z = depth_frame[v, u]
                
                # Skip invalid depth
                if z <= 0 or z > 10.0:  # 10 meter max range
                    continue
                
                # Convert to 3D coordinates (pinhole camera model)
                x = (u * self.downsample_factor - self.principal_point[0]) * z / self.focal_length
                y = (v * self.downsample_factor - self.principal_point[1]) * z / self.focal_length
                
                points.append([x, y, z])
        
        return np.array(points)
    
    def compress_pointcloud(self, points):
        """Compress point cloud data for transmission"""
        if points is None or len(points) == 0:
            return None
        
        compressed_data = {
            'type': 'compressed_pointcloud',
            'timestamp': time.time(),
            'point_count': len(points),
            'data': points.tobytes(),
            'shape': points.shape,
            'dtype': str(points.dtype)
        }
        
        return compressed_data
    
    def detect_obstacles(self, depth_frame, min_distance=0.1, max_distance=3.0):
        """Simple obstacle detection from depth frame"""
        if depth_frame is None:
            return None
        
        # Create obstacle map
        obstacle_mask = (depth_frame >= min_distance) & (depth_frame <= max_distance)
        
        # Find closest obstacle in each region (left, center, right)
        height, width = depth_frame.shape
        region_width = width // 3
        
        obstacles = {
            'left': float('inf'),
            'center': float('inf'), 
            'right': float('inf'),
            'min_distance': float('inf')
        }
        
        # Left region
        left_region = depth_frame[:, :region_width]
        if np.any(obstacle_mask[:, :region_width]):
            obstacles['left'] = np.min(left_region[obstacle_mask[:, :region_width]])
        
        # Center region  
        center_region = depth_frame[:, region_width:2*region_width]
        if np.any(obstacle_mask[:, region_width:2*region_width]):
            obstacles['center'] = np.min(center_region[obstacle_mask[:, region_width:2*region_width]])
        
        # Right region
        right_region = depth_frame[:, 2*region_width:]
        if np.any(obstacle_mask[:, 2*region_width:]):
            obstacles['right'] = np.min(right_region[obstacle_mask[:, 2*region_width:]])
        
        obstacles['min_distance'] = min(obstacles['left'], obstacles['center'], obstacles['right'])
        
        return obstacles
    
    def _get_simulated_depth_frame(self):
        """Generate simulated depth data for testing"""
        # Create a simulated depth frame with some obstacles
        depth_frame = np.ones((self.frame_height, self.frame_width), dtype=np.float32) * 5.0
        
        # Add some simulated obstacles
        center_x, center_y = self.frame_width // 2, self.frame_height // 2
        
        # Wall at 2 meters
        depth_frame[center_y-20:center_y+20, :] = 2.0
        
        # Pillar at 1 meter
        depth_frame[center_y-10:center_y+10, center_x-10:center_x+10] = 1.0
        
        # Add some noise
        noise = np.random.normal(0, 0.1, depth_frame.shape)
        depth_frame += noise
        
        return depth_frame
    
    def cleanup(self):
        """Release camera resources"""
        if self.camera:
            self.camera.release()
        self.connected = False
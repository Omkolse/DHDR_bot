from setuptools import setup
import os
from glob import glob

package_name = 'spherical_bot_cloud'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        f'{package_name}.point_cloud_pipeline',
        f'{package_name}.point_cloud_pipeline.data_receiver',
        f'{package_name}.point_cloud_pipeline.decompression_engine',
        f'{package_name}.point_cloud_pipeline.noise_removal',
        f'{package_name}.point_cloud_pipeline.frame_stitching',
        f'{package_name}.point_cloud_pipeline.live_visualization',
        f'{package_name}.point_cloud_pipeline.storage_management',
        f'{package_name}.navigation_engine',
        f'{package_name}.navigation_engine.slam_processor',
        f'{package_name}.navigation_engine.path_planner',
        f'{package_name}.navigation_engine.obstacle_processor',
        f'{package_name}.ai_processing',
        f'{package_name}.ai_processing.object_detection',
        f'{package_name}.ai_processing.scene_understanding',
        f'{package_name}.ai_processing.behavioral_analysis',
        f'{package_name}.performance_optimization',
        f'{package_name}.performance_optimization.load_balancer',
        f'{package_name}.performance_optimization.data_optimization',
        f'{package_name}.performance_optimization.monitoring_analytics',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Spherical Bot Admin',
    maintainer_email='admin@sphericalbot.com',
    description='Cloud services for Spherical Bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Point Cloud Pipeline
            'data_receiver = spherical_bot_cloud.point_cloud_pipeline.data_receiver.mqtt_pointcloud_subscriber:main',
            'decompression_engine = spherical_bot_cloud.point_cloud_pipeline.decompression_engine.run_length_decoder:main',
            'noise_removal = spherical_bot_cloud.point_cloud_pipeline.noise_removal.statistical_outlier_removal:main',
            'frame_stitching = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.icp_registrar:main',
            'live_visualization = spherical_bot_cloud.point_cloud_pipeline.live_visualization.websocket_pointcloud_stream:main',
            
            # Navigation Engine
            'slam_processor = spherical_bot_cloud.navigation_engine.slam_processor.occupancy_grid_mapper:main',
            'path_planner = spherical_bot_cloud.navigation_engine.path_planner.global_planner:main',
            'obstacle_processor = spherical_bot_cloud.navigation_engine.obstacle_processor.dynamic_obstacle_tracker:main',
            
            # AI Processing
            'object_detection = spherical_bot_cloud.ai_processing.object_detection.yolo_pointcloud_detector:main',
            
            # Performance Optimization
            'load_balancer = spherical_bot_cloud.performance_optimization.load_balancer.request_distributor:main',
            'monitoring_analytics = spherical_bot_cloud.performance_optimization.monitoring_analytics.performance_tracker:main',
            
            # Web Dashboard (Flask app)
            'web_dashboard = spherical_bot_cloud.web_dashboard.app:main',
        ],
    },
)
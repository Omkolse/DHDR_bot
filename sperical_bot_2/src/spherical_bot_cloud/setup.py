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
            'websocket_handler = spherical_bot_cloud.point_cloud_pipeline.data_receiver.websocket_data_handler:main',
            'compression_detector = spherical_bot_cloud.point_cloud_pipeline.data_receiver.compression_detector:main',
            'data_validator = spherical_bot_cloud.point_cloud_pipeline.data_receiver.data_validator:main',
            
            'rle_decoder = spherical_bot_cloud.point_cloud_pipeline.decompression_engine.run_length_decoder:main',
            'delta_decoder = spherical_bot_cloud.point_cloud_pipeline.decompression_engine.delta_decoder:main',
            'downsampling_reverser = spherical_bot_cloud.point_cloud_pipeline.decompression_engine.downsampling_reverser:main',
            'format_converter = spherical_bot_cloud.point_cloud_pipeline.decompression_engine.format_converter:main',
            
            # Noise Removal
            'statistical_outlier_removal = spherical_bot_cloud.point_cloud_pipeline.noise_removal.statistical_outlier_removal:main',
            'radius_outlier_removal = spherical_bot_cloud.point_cloud_pipeline.noise_removal.radius_outlier_removal:main',
            'depth_edge_filter = spherical_bot_cloud.point_cloud_pipeline.noise_removal.depth_edge_filter:main',
            'temporal_consistency_filter = spherical_bot_cloud.point_cloud_pipeline.noise_removal.temporal_consistency_filter:main',
            'adaptive_thresholding = spherical_bot_cloud.point_cloud_pipeline.noise_removal.adaptive_thresholding:main',
            
            # Frame Stitching
            'icp_registrar = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.icp_registrar:main',
            'feature_based_aligner = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.feature_based_aligner:main',
            'pose_graph_optimizer = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.pose_graph_optimizer:main',
            'loop_closure_detector = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.loop_closure_detector:main',
            'global_consistency_checker = spherical_bot_cloud.point_cloud_pipeline.frame_stitching.global_consistency_checker:main',
            
            # Live Visualization
            'websocket_pointcloud_stream = spherical_bot_cloud.point_cloud_pipeline.live_visualization.websocket_pointcloud_stream:main',
            
            # Storage Management
            'realtime_buffer = spherical_bot_cloud.point_cloud_pipeline.storage_management.realtime_buffer:main',
            'map_database_writer = spherical_bot_cloud.point_cloud_pipeline.storage_management.map_database_writer:main',
        ],
    },
)
from setuptools import setup, find_packages
import os
from glob import glob

# Read package version
def get_package_version():
    try:
        with open('version.txt', 'r') as f:
            return f.read().strip()
    except:
        return "1.0.0"

# Read long description from README
def get_long_description():
    try:
        with open('README.md', 'r') as f:
            return f.read()
    except:
        return "Self-balancing spherical robot with real-time control and cloud navigation"

package_name = 'spherical_bot'
version = get_package_version()

setup(
    name=package_name,
    version=version,
    packages=find_packages(),
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install script files (make them executable)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        
        # Install documentation
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        
        # Install test files
        (os.path.join('share', package_name, 'test'), glob('test/*.py')),
    ],
    install_requires=[
        'setuptools>=58.2.0',
        'wheel>=0.37.0',
    ],
    zip_safe=True,
    maintainer='Spherical Bot Team',
    maintainer_email='spherical-bot@example.com',
    description='Self-balancing spherical robot with real-time control and cloud navigation',
    long_description=get_long_description(),
    long_description_content_type='text/markdown',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    # Entry points for ROS 2 nodes
    entry_points={
        'console_scripts': [
            # Core nodes
            'core_controller = spherical_bot.nodes.core_controller:main',
            'sensor_manager = spherical_bot.nodes.sensor_manager:main',
            'cloud_bridge = spherical_bot.nodes.cloud_bridge:main',
            'system_guardian = spherical_bot.nodes.system_guardian:main',
            
            # Utility scripts
            'spherical_bot_calibration = spherical_bot.scripts.calibration:main',
            'spherical_bot_diagnostics = spherical_bot.scripts.diagnostics:main',
            'spherical_bot_emergency = spherical_bot.scripts.emergency_stop:main',
            
            # Test nodes
            'test_sensors = spherical_bot.test.test_sensors:main',
            'test_motors = spherical_bot.test.test_motors:main',
            'test_balance = spherical_bot.test.test_balance:main',
        ],
        
        # ROS 2 launch file entry points
        'ros2cli.command': [
            'spherical_bot = spherical_bot.cli:SphericalBotCommand',
        ],
    },
    
    # Python requirements
    python_requires='>=3.8',
    
    # Classifiers for PyPI
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
    
    # Keywords
    keywords=['ros2', 'robotics', 'self-balancing', 'autonomous', 'navigation'],
    
    # Project URLs
    project_urls={
        'Documentation': 'https://github.com/your-username/spherical_bot/docs',
        'Source': 'https://github.com/your-username/spherical_bot',
        'Tracker': 'https://github.com/your-username/spherical_bot/issues',
    },
)
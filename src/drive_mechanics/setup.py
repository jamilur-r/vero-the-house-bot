from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drive_mechanics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        # Include URDF files if any
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.urdf*'))),
        # Include meshes if any
        (os.path.join('share', package_name, 'meshes'), 
            glob(os.path.join('meshes', '*'))),
        # Include documentation
        (os.path.join('share', package_name, 'docs'), 
            glob(os.path.join('docs', '*.md'))),
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), 
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=[
        'setuptools',
        'smbus2',  # For I2C communication
        'pyyaml',  # For YAML configuration files
    ],
    zip_safe=True,
    maintainer='xlab',
    maintainer_email='abir2jamilur30@gmail.com',
    description='ROS2 package for controlling differential drive robots using Waveshare Motor Driver Hat with PCA9685',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Pi Zero nodes (lightweight, hardware interface)
            'drive_node = drive_mechanics.drive_node:main',
            'motor_test = drive_mechanics.motor_test:main',
            
            # PC nodes (computational, simulation, teleop)
            'teleop_node = drive_mechanics.teleop_node:main',
            'robot_simulator = drive_mechanics.robot_simulator:main',
        ],
    },
)

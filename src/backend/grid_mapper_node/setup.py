from setuptools import find_packages, setup
import os
from glob import glob

# Package name must exactly match the directory name and package.xml <name>
package_name = 'grid_mapper_node'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers grid_mapper_node/ (the inner Python package)
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament_index registration — required for `ros2 run` to discover the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml into the share directory
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR Team',
    maintainer_email='team@nidar.dev',
    description='Grid mapper node: SLAM position → arena grid box ID, '
                'survivor deduplication, occupancy grid overlay',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run grid_mapper_node grid_mapper
            # → calls grid_mapper_node.grid_mapper_node:main()
            'grid_mapper = grid_mapper_node.grid_mapper_node:main',
        ],
    },
)

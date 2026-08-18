from setuptools import find_packages, setup
import os
from glob import glob

# Package name must exactly match the directory name and package.xml <name>
package_name = 'exploration_node'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers the inner exploration_node/ Python package
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament_index registration — required for ros2 run to discover the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml into share directory
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR Team',
    maintainer_email='team@nidar.dev',
    description=(
        'Exploration Node for NIDAR AirMouse — frontier-based autonomous '
        'exploration with A* path planning and battery-aware replanning.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run exploration_node exploration_node
            # -> calls exploration_node.exploration_node:main()
            'exploration_node = exploration_node.exploration_node:main',
        ],
    },
)

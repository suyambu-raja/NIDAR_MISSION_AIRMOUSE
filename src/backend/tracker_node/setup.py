from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tracker_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR Team',
    maintainer_email='team@nidar.dev',
    description='ByteTrack survivor tracking node for NIDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker = tracker_node.tracker_node:main'
        ],
    },
)

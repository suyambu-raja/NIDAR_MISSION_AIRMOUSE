from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'detection_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR Team',
    maintainer_email='dev@nidar.org',
    description='NIDAR AirMouse RGB and Thermal Human/Survivor Detection Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = detection_node.detection_node:main',
        ],
    },
)

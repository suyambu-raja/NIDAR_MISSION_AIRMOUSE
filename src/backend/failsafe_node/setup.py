from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'failsafe_node'

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
    description=(
        'Failsafe Safety Supervisor Node for NIDAR AirMouse — monitors '
        'battery, telemetry link loss, geofence, and triggers obstacle-aware A* RTL.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'failsafe_node = failsafe_node.failsafe_node:main',
        ],
    },
)

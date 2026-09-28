from setuptools import setup, find_packages
import os

package_name = 'mission_manager'

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
    description='NIDAR AirMouse Rescue Mission State Machine Supervisor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = mission_manager.mission_manager_node:main',
        ],
    },
)

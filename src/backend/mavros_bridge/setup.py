from setuptools import find_packages, setup

package_name = 'mavros_bridge'

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
        'MAVROS Bridge Node for NIDAR AirMouse — high-level interface '
        'between ROS 2 and ArduPilot/Pixhawk with SITL and hardware support.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavros_bridge_node = mavros_bridge.mavros_bridge_node:main',
        ],
    },
)

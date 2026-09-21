from setuptools import find_packages, setup

package_name = 'corridor_classifier'

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
        'Corridor and Room Semantic Classifier Node for NIDAR AirMouse — '
        'geometric segmentation and labeling of SLAM occupancy grid maps.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'corridor_classifier_node = corridor_classifier.corridor_classifier_node:main',
        ],
    },
)

from setuptools import find_packages, setup

# Package name must exactly match the directory name and package.xml <name>
package_name = 'fusion_node'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers fusion_node/ (the inner Python package)
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
    description='Sensor fusion node: RGB tracker + thermal detector → '
                'confirmed survivors with weighted evidence fusion + EMA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run fusion_node fusion
            # → calls fusion_node.fusion_node:main()
            'fusion = fusion_node.fusion_node:main',
        ],
    },
)

"""
setup.py — slam_node ROS 2 Python Package Setup
================================================
Registers the slam_node package with colcon / ament_python.

What this installs:
  - slam_node Python module (the executable node)
  - rplidar_launch.py → share/slam_node/launch/
  - slam_params.yaml  → share/slam_node/config/

Build and install:
    cd /path/to/ros2_ws
    colcon build --packages-select slam_node --symlink-install
    source install/setup.bash

After install, the following commands are available:
    ros2 run slam_node slam_node
    ros2 launch slam_node rplidar_launch.py
"""

import os
from glob import glob
from setuptools import find_packages, setup

# Package name — must match the <name> tag in package.xml exactly
PACKAGE_NAME = "slam_node"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    # Automatically discover sub-packages (none currently, but ready for expansion)
    packages=find_packages(exclude=["test"]),
    # Non-Python data files to install alongside the package
    data_files=[
        # Required by ament: register this package in the ROS index
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        # Package manifest — colcon reads this for dependency resolution
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        # ------------------------------------------------------------------ #
        # Launch files — installed to share/slam_node/launch/                #
        # Allows: ros2 launch slam_node rplidar_launch.py                     #
        # ------------------------------------------------------------------ #
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("*.py"),   # rplidar_launch.py (and any future launch files)
        ),
        # ------------------------------------------------------------------ #
        # Config files — installed to share/slam_node/config/                #
        # slam_params.yaml is referenced by rplidar_launch.py via             #
        # get_package_share_directory("slam_node") + "/config/slam_params.yaml"
        # ------------------------------------------------------------------ #
        (
            os.path.join("share", PACKAGE_NAME, "config"),
            glob("*.yaml"),  # slam_params.yaml
        ),
    ],
    # Runtime Python dependencies (ROS 2 packages are managed separately by apt)
    install_requires=["setuptools"],
    zip_safe=True,
    # Package metadata
    maintainer="NIDAR Team",
    maintainer_email="team@nidar.dev",
    description=(
        "SLAM relay node for NIDAR AirMouse — wraps slam_toolbox to publish "
        "/map and /drone_pose from RPLIDAR A2M8 input on Raspberry Pi 4."
    ),
    license="MIT",
    # ament will run these tests when `colcon test` is invoked
    tests_require=["pytest"],
    # -----------------------------------------------------------------------  #
    # Entry points — defines `ros2 run slam_node slam_node` console script      #
    # The key format is: "executable_name = package.module:function"            #
    # -----------------------------------------------------------------------  #
    entry_points={
        "console_scripts": [
            # ros2 run slam_node slam_node → calls slam_node.slam_node.main()
            "slam_node = slam_node.slam_node:main",
        ],
    },
)

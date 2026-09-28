import os
from glob import glob
from setuptools import setup

package_name = "nidar_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NIDAR Autonomy Team",
    maintainer_email="info@nidar.aero",
    description="Bringup launch files for NIDAR UAV autonomy system",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)

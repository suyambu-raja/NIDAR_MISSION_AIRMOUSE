# slam_node/__init__.py
# Makes slam_node/ a Python package so setup.py entry point
# `slam_node = slam_node:main` resolves correctly.
#
# Re-export main so `from slam_node import main` works from setup.py's
# console_scripts entry point.
from slam_node.slam_node import main  # noqa: F401

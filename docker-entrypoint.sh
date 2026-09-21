#!/bin/bash
# =============================================================================
# NIDAR AirMouse — Docker Entrypoint Script
# Sources ROS 2 underlay + workspace overlay, then executes the given command
# =============================================================================

set -e

# Source the ROS 2 Humble base installation
source /opt/ros/humble/setup.bash

# If the workspace has been built (install/ directory exists), source it too
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo "[NIDAR] Workspace overlay sourced."
else
    echo "[NIDAR] No workspace overlay found. Run 'colcon build' first."
fi

# Execute whatever command was passed to the container
exec "$@"

#!/bin/bash
# demo_grid_mapper.sh — full live demo inside nidar-dev container
# Starts: mock SLAM + grid_mapper_node + web dashboard
# Open http://localhost:8080 in your Windows browser to see the live grid

source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "======================================================"
echo " NIDAR Grid Mapper — Live Demo"
echo " Open http://localhost:8080 in your browser!"
echo "======================================================"

# 1. Start mock SLAM
echo "[1/3] Starting mock SLAM (/map + /drone_pose)..."
python3 /workspace/tools/mock_publishers/mock_slam.py &
sleep 2

# 2. Start grid_mapper_node
echo "[2/3] Starting grid_mapper_node..."
ros2 run grid_mapper_node grid_mapper &
sleep 2

# 3. Start web dashboard (blocks — serves until Ctrl+C)
echo "[3/3] Starting web dashboard at http://localhost:8080 ..."
echo ""
echo "  -> Open http://localhost:8080 in your browser NOW"
echo "  -> To inject a survivor, open a new terminal and run:"
echo ""
echo "  docker run --rm -v \"\${PWD}:/workspace\" nidar-dev bash -c \\"
echo "    \"source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \\"
echo "     ros2 topic pub --once /tracked_survivors nidar_msgs/SurvivorArray \\"
echo "     '{header: {frame_id: map}, detections: [{survivor_id: 1, position: {x: 6.3, y: 4.7, z: 0.0}, confidence: 0.92, is_confirmed: true, detection_source: rgb, bbox: [0.0, 0.0, 0.0, 0.0]}]}'\""
echo ""

python3 /workspace/tools/mock_publishers/grid_mapper_visualizer.py

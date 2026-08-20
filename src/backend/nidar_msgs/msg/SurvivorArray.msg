# =============================================================================
# SurvivorArray.msg — Array of survivor detections (batch publish)
# Published by: detection_node (raw), tracker_node (tracked), fusion_node (fused)
# =============================================================================

# Standard ROS 2 header (contains timestamp and frame_id for coordinate transforms)
std_msgs/Header header

# Array of individual survivor detections
SurvivorDetection[] detections

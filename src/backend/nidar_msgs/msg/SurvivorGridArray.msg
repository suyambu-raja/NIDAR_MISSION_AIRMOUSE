# =============================================================================
# SurvivorGridArray.msg — Array of confirmed survivor grid locations
# Published by: grid_mapper_node  (~2 Hz)
# Consumed by:  fusion_node, GCS dashboard
# =============================================================================

# Standard ROS 2 header (timestamp + frame_id = "map")
std_msgs/Header header

# All confirmed unique survivors found so far during the mission
SurvivorGridLocation[] locations

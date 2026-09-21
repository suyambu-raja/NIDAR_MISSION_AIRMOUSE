# =============================================================================
# MissionStatus.msg — Overall mission status
# Published by: mission_manager_node
# =============================================================================

# Standard ROS 2 header
std_msgs/Header header

# Drone battery status
float32 battery_percentage

# Time elapsed since mission start in seconds
int32 time_elapsed_sec

# Total number of unique survivors found and tracked
int32 survivors_found

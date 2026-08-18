# =============================================================================
# SpaceClassification.msg — Room vs corridor classification result
# Published by: space_classifier_node
# =============================================================================

# Standard ROS 2 header
std_msgs/Header header

# Classified space type: "room" or "corridor"
string space_type

# Classification confidence score (0.0 to 1.0)
float32 confidence

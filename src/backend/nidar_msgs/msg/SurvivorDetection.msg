# =============================================================================
# SurvivorDetection.msg — Single survivor detection result
# Published by: fusion_node (after combining RGB + thermal evidence)
# Consumed by:  tracker_node, grid_mapper_node, GCS dashboard
# =============================================================================

# Unique survivor ID assigned by the tracker (-1 if not yet tracked)
int32 survivor_id

# 3D position of the detected survivor in the map frame
geometry_msgs/Point position

# Detection confidence score (0.0 to 1.0) — from fusion algorithm
float32 confidence

# Source of the detection: "rgb", "thermal", "fused"
string detection_source

# Bounding box in image coordinates (for GCS overlay)
# [x_min, y_min, width, height]
float32[4] bbox

# Timestamp of the original sensor reading
builtin_interfaces/Time detection_stamp

# Is this a confirmed survivor (confidence above threshold + persistence)?
bool is_confirmed

# =============================================================================
# SurvivorGridLocation.msg — A single confirmed survivor's grid location
# Published by: grid_mapper_node
# Consumed by:  fusion_node, GCS dashboard
# =============================================================================

# Unique survivor ID (assigned by tracker, preserved through grid mapping)
int32 survivor_id

# Grid box identifier, e.g. "F4" (column letter + row number)
# Column: A–O (left→right, X axis), Row: 1–15 (bottom→top, Y axis)
string grid_box

# World-frame position in metres (map frame, from SLAM)
float32 world_x
float32 world_y

# Detection confidence score (0.0 to 1.0), averaged from all matched detections
float32 confidence

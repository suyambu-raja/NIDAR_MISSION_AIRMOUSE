cmake_minimum_required(VERSION 3.8)
project(nidar_msgs)

# Find build dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# =============================================================================
# Generate message interfaces from .msg files
# These become importable as: from nidar_msgs.msg import SurvivorDetection
# =============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  # Single survivor detection result
  "msg/SurvivorDetection.msg"
  # Array of survivor detections (batch publish)
  "msg/SurvivorArray.msg"
  # Overall mission status (battery, time, survivors found)
  "msg/MissionStatus.msg"
  # Room vs corridor classification result
  "msg/SpaceClassification.msg"
  # Grid mapper outputs: one survivor's grid location
  "msg/SurvivorGridLocation.msg"
  # Grid mapper outputs: all confirmed survivor grid locations
  "msg/SurvivorGridArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()

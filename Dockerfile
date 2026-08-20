# =============================================================================
# NIDAR AirMouse — Multi-Stage Dockerfile
# Target: NVIDIA Jetson Nano Super / Jetson (JetPack 6.0 / L4T r36.2.0, ARM64) with ROS 2 Humble
# =============================================================================
# Stage 1: Base image with all ROS 2 + system dependencies
# Stage 2: Development image with colcon build tools
# =============================================================================

# ---- Stage 1: Base Runtime ----
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0 AS base

# Prevent interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Use CycloneDDS — more stable than FastDDS in Docker and on ARM64 hardware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set ROS domain ID (isolates this system's DDS traffic)
ENV ROS_DOMAIN_ID=0

# --------------------------------------------------------------------------
# System dependencies: ROS 2 packages + hardware drivers + ML libraries
# All apt installs in a single layer to minimize image size
# --------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- DDS middleware ---
    ros-humble-rmw-cyclonedds-cpp \
    # --- MAVROS (Pixhawk bridge) ---
    ros-humble-mavros \
    ros-humble-mavros-extras \
    # --- Navigation2 (path planning, costmaps, obstacle avoidance) ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # --- SLAM (2D occupancy grid mapping) ---
    ros-humble-slam-toolbox \
    # --- RPLIDAR driver ---
    ros-humble-rplidar-ros \
    # --- rosbridge (WebSocket bridge for GCS dashboard) ---
    ros-humble-rosbridge-suite \
    # --- TF2 (coordinate frame transforms) ---
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # --- Image transport and CV bridge ---
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    # --- Vision messages (for detection results) ---
    ros-humble-vision-msgs \
    # --- Diagnostic tools ---
    ros-humble-diagnostic-msgs \
    # --- Build essentials for custom messages ---
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    # --- PyTorch & OpenBLAS dependencies for Jetson ---
    libopenblas-base \
    libopenmpi-dev \
    libomp-dev \
    # --- General system tools ---
    python3-pip \
    python3-opencv \
    git \
    wget \
    usbutils \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------
# Install MAVROS geographic lib datasets (required for GPS-denied operation)
# This downloads geoid data needed by MAVROS even in GPS-denied mode
# --------------------------------------------------------------------------
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# --------------------------------------------------------------------------
# Python dependencies: Jetson CUDA PyTorch, DepthAI SDK, thermal processing, tracking
# Pinned versions for reproducibility
# --------------------------------------------------------------------------
RUN pip3 install --no-cache-dir \
    https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.2.0a0+6a974be.nv23.11-cp310-cp310-linux_aarch64.whl \
    torchvision==0.17.0 \
    depthai==2.24.0.0 \
    numpy>=1.24.0 \
    scipy>=1.10.0 \
    filterpy>=1.4.5 \
    lap>=0.4.0 \
    Pillow>=9.0.0 \
    ultralytics>=8.0.0

# Set up the workspace directory
WORKDIR /workspace

# ---- Stage 2: Development ----
FROM base AS dev

# Additional development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- Colcon build system ---
    python3-colcon-common-extensions \
    python3-rosdep \
    # --- Debugging tools ---
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    htop \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint script
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

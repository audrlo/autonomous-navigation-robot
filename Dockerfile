# Multi-stage build for autonomous navigation robot
# Supports both AMD64 and ARM64 (for Raspberry Pi 5)
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_WS=/workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    software-properties-common \
    # ROS2 dependencies
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-colcon-common-extensions \
    python3-vcstool \
    # Intel RealSense dependencies
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    # Additional dependencies for SLAM and navigation
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    # Serial communication for RoboClaw
    python3-serial \
    # Network tools
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK
RUN mkdir -p /tmp/realsense && cd /tmp/realsense \
    && wget https://github.com/IntelRealSense/librealsense/releases/download/v2.54.1/librealsense2-dkms_1.3.18-0ubuntu1_amd64.deb \
    && wget https://github.com/IntelRealSense/librealsense/releases/download/v2.54.1/librealsense2-utils_2.54.1-0~realsense0.8431_amd64.deb \
    && wget https://github.com/IntelRealSense/librealsense/releases/download/v2.54.1/librealsense2-dev_2.54.1-0~realsense0.8431_amd64.deb \
    && dpkg -i *.deb || apt-get install -f -y \
    && rm -rf /tmp/realsense

# Create workspace
WORKDIR ${ROS_WS}

# Copy source files
COPY src/ src/

# Install ROS dependencies
RUN apt-get update && rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . ${ROS_ROOT}/setup.sh \
    && colcon build --symlink-install \
    && rm -rf build log

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up the environment
RUN echo "source ${ROS_ROOT}/setup.bash" >> ~/.bashrc \
    && echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"] 
#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if it exists
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source ${ROS_WS}/install/setup.bash
fi

# Set up device permissions for RealSense camera
if [ -c /dev/video0 ]; then
    echo "Setting up RealSense camera permissions..."
    chmod 666 /dev/video*
fi

# Set up serial port permissions for RoboClaw
if [ -c /dev/ttyACM0 ]; then
    echo "Setting up RoboClaw serial permissions..."
    chmod 666 /dev/ttyACM*
fi

# Execute the command
exec "$@"
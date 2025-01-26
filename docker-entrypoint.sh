#!/usr/bin/env bash
set -e  # Exit immediately if a command exits with a non-zero status

# Source ROS2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Check if the workspace is built
if [ ! -f "/app/install/setup.bash" ]; then
    echo "Workspace not built. Building now..."
    colcon build --base-paths /app/src/fg_srv --install-base /app/install
    echo "Workspace built successfully."
fi

# Source the local workspace setup
source /app/install/setup.bash

# Check if TEST_MODE is set
if [ "$TEST_MODE" == "true" ]; then
    echo "Running tests..."
    pytest src/fg_srv/test/ --ros-domain-id="${ROS_DOMAIN_ID}"
else
    echo "Starting ROS2 mission system..."
    # Run the launch file
    exec ros2 launch fg_srv mission_system.launch.py
fi

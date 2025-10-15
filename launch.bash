#!/bin/bash

# --- NEW LINE ADDED TO SOURCE ROS 2 ENVIRONMENT ---
source /opt/ros/jazzy/setup.bash

# Build the package if it hasn't been built yet
colcon build --packages-select rpi_cpp_hardware || exit 1

# Source the package (this is needed to find the local launch file)
source install/setup.bash

# Launch the node
ros2 launch rpi_cpp_hardware rpi_cpp_hardware_launch.py

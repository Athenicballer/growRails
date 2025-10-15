#!/bin/bash

# Ensure the script exits immediately if any command fails (e.g., if the build fails)
set -e 

# --- 1. SOURCE ROS 2 ENVIRONMENT ---
echo "--- Sourcing ROS 2 environment (Jazzy) ---"
source /opt/ros/jazzy/setup.bash

# --- 2. BUILD THE PACKAGE ---
# Compiles the C++ nodes and installs the launch file
echo "--- Building rpi_cpp_hardware package ---"
colcon build --packages-select rpi_cpp_hardware 

# --- 3. SOURCE THE LOCAL WORKSPACE ---
# This step makes the newly built C++ executables and the launch file available
echo "--- Sourcing local workspace setup ---"
source install/setup.bash

# --- 4. LAUNCH THE SYSTEM ---
# Executes the rpi_system_launch.py file, which starts all three C++ nodes
echo "--- Launching RPi C++ Hardware System ---"
ros2 launch rpi_cpp_hardware rpi_system_launch.py
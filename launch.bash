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

# Check the build result
if [ $? -ne 0 ]; then
    echo "--- ERROR: Build failed. Aborting launch. ---"
    exit 1
fi

# --- 3. SOURCE THE LOCAL WORKSPACE ---
# This step makes the newly built C++ executables and the launch file available
echo "--- Sourcing local workspace setup ---"
source install/setup.bash

# --- 4. PiGPIO DAEMON CHECK (CRITICAL FOR HARDWARE) ---
# Check if the pigpiod daemon is running.
if ! pgrep "pigpiod" > /dev/null; then
    echo "--- PiGPIO Daemon is NOT running. Attempting to start with sudo... ---"
    # The daemon must be run with sudo for proper GPIO access.
    # If the main script is run without sudo, this line will prompt for a password.
    sudo pigpiod
    if [ $? -eq 0 ]; then
        echo "--- PiGPIO Daemon successfully started. Waiting for initialization... ---"
        # Wait a moment for the daemon to initialize the connection sockets
        sleep 3
    else
        echo "--- ERROR: Failed to start pigpiod (sudo pigpiod failed). Check daemon installation and permissions. ---"
        exit 1
    fi
else
    echo "--- PiGPIO Daemon is already running. ---"
fi

# --- 5. LAUNCH THE SYSTEM ---
echo "--- Launching RPi C++ Hardware System ---"
ros2 launch rpi_cpp_hardware rpi_system_launch.py
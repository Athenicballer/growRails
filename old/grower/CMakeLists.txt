cmake_minimum_required(VERSION 3.10)
project(robot_control_package)

# 1. --- ROS 2 Package Configuration ---
# Set package name (required for ament_cmake)
set(PACKAGE_NAME robot_control_package)

# Find ROS 2 build system
find_package(ament_cmake REQUIRED)

# Find required ROS 2 dependencies (Messages and Core)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. --- Pigpio Integration (As Subdirectory) ---
# Assuming pigpio source is located in the sibling directory '../pigpio'
set(PIGPIO_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../pigpio")
set(PIGPIO_INCLUDE_DIR "${PIGPIO_SOURCE_DIR}")

# Build the PIGPIO library first
add_subdirectory(${PIGPIO_SOURCE_DIR} pigpio_build)

# 3. --- Define Executable and Link Dependencies ---
# Add your main application source file
add_executable(robot_node main.cpp)

# Link all dependencies to the executable
target_link_libraries(robot_node
    # Link to the pigpio library built by the subdirectory
    pigpio
    # Link to ROS 2 libraries
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    # Link to required Linux libraries
    Threads::Threads
)

# 4. --- Compiler Include Paths ---
# Add the pigpio include path for the compiler
target_include_directories(robot_node PRIVATE 
    ${PIGPIO_INCLUDE_DIR}
)

# 5. --- Install Rules (Required for ROS 2) ---
install(TARGETS robot_node
    DESTINATION lib/${PACKAGE_NAME}
)

# Export the build information (Ament required)
ament_package()
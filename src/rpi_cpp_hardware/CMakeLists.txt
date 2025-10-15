cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- NEW: Find the pigpio library ---
# This looks for the pigpio library file (libpigpio.so or libpigpio.a)
find_library(PIGPIO_LIBRARIES pigpio)

if(NOT PIGPIO_LIBRARIES)
  message(FATAL_ERROR "Could not find pigpio library. Ensure 'sudo apt install libpigpio-dev' has been run.")
endif()

# --- C++ Node Definitions ---

# Sensor Publisher Node
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs)
# NEW: Link the executable to the pigpio library
target_link_libraries(sensor_publisher_node ${PIGPIO_LIBRARIES})

# Motor Driver Node
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs)
# NEW: Link the executable to the pigpio library
target_link_libraries(motor_driver_node ${PIGPIO_LIBRARIES})

# Hardware System Node (if you have one)
# Assuming a third node is for the rclcpp_components hardware implementation (if you plan to use ros2_control)
# If this is a placeholder, feel free to uncomment/rename as needed
# add_executable(rpi_hardware_system_node src/rpi_hardware_system.cpp)
# ament_target_dependencies(rpi_hardware_system_node rclcpp)
# target_link_libraries(rpi_hardware_system_node ${PIGPIO_LIBRARIES})

# Install targets
install(TARGETS
  sensor_publisher_node
  motor_driver_node
  # rpi_hardware_system_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

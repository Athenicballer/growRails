cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 and external packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) # For sensor publisher message

# --- Motor Driver Node Configuration ---
add_executable(motor_driver_node
  src/motor_driver.cpp
)
# Define ROS 2 dependencies
ament_target_dependencies(motor_driver_node
  rclcpp
  geometry_msgs
)
# CRUCIAL FIX: Link the external pigpio client library
target_link_libraries(motor_driver_node
  pigpiod_if_c
)
install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Sensor Publisher Node Configuration ---
add_executable(sensor_publisher_node
  src/sensor_publisher.cpp
)
# Define ROS 2 dependencies
ament_target_dependencies(sensor_publisher_node
  rclcpp
  std_msgs
)
# CRUCIAL FIX: Link the external pigpio client library
target_link_libraries(sensor_publisher_node
  pigpiod_if_c
)
install(TARGETS
  sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
)

ament_package()

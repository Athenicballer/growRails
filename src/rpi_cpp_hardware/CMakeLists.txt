cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find the external pigpio library
# This assumes the pigpio library is installed on the system (e.g., via apt install pigpio)
find_library(PIGPIO_LIBRARY pigpio)
if(NOT PIGPIO_LIBRARY)
    message(FATAL_ERROR "Could not find pigpio library. Please ensure 'sudo apt install pigpio' has been run.")
endif()


# --- 1. Motor Driver Node ---
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node
  rclcpp
  geometry_msgs
)
# Link the motor driver node against the system pigpio library
target_link_libraries(motor_driver_node ${PIGPIO_LIBRARY})

# --- 2. Sensor Publisher Node ---
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node
  rclcpp
  std_msgs
)
# Link the sensor publisher node against the system pigpio library
target_link_libraries(sensor_publisher_node ${PIGPIO_LIBRARY})

# --- 3. Main Control Logic Node (No direct pigpio, only ROS) ---
add_executable(main_control_node src/main_control_logic.cpp)
ament_target_dependencies(main_control_node
  rclcpp
  std_msgs
  geometry_msgs
)


# --- Installation (Install executables into lib/${PROJECT_NAME}) ---
install(TARGETS
  motor_driver_node
  sensor_publisher_node
  main_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs geometry_msgs)
ament_package()

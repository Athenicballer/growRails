cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

# Set C++ standard to 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNU_CXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # For cmd_vel

# --- Find PiGPIO dependencies ---
# The target_link_libraries uses these names.
find_library(PIGPIO_LIBRARIES pigpio)
find_library(PIGPIOD_IF_LIBRARIES pigpiod_if)

if(NOT PIGPIO_LIBRARIES OR NOT PIGPIOD_IF_LIBRARIES)
  message(FATAL_ERROR "Could not find required pigpio or pigpiod_if libraries. Please install them.")
endif()

# --- 1. Custom Message Generation ---
# Find required packages for message generation
find_package(rosidl_default_generators REQUIRED)

# List custom messages here
rosidl_generate_interfaces(rpi_cpp_hardware
  MSG
  msg/RpiSensorData.msg
  DEPENDENCIES
  std_msgs
)

# --- Node for Main Control Logic ---
add_executable(main_control_logic_node src/main_control_logic.cpp)
ament_target_dependencies(main_control_logic_node rclcpp std_msgs geometry_msgs ${PROJECT_NAME} ) # Added ${PROJECT_NAME} for custom message headers
# Link against pigpiod_if only if needed for client mode, or skip for pure logic
install(TARGETS
  main_control_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Node for Motor Control (Direct PiGPIO Access) ---
add_executable(motor_driver_node src/motor_driver.cpp)
# Added ${PROJECT_NAME} for custom message headers, even if not strictly needed now
ament_target_dependencies(motor_driver_node rclcpp std_msgs geometry_msgs ${PROJECT_NAME})
# Link against both the client interface and the core pigpio library (for direct access)
target_link_libraries(motor_driver_node pigpiod_if pigpio)
install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Node for Sensor Reading (PiGPIO Client Access) ---
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
# IMPORTANT: Added ${PROJECT_NAME} to link the generated RpiSensorData.msg header
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs ${PROJECT_NAME})
# Link against the client interface (pigpiod_if) and the core pigpio library
target_link_libraries(sensor_publisher_node pigpiod_if pigpio)
install(TARGETS
  sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install the launch files ---
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install message files
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_dirs(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime
)

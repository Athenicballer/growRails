cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# REQUIRED for custom message generation
find_package(rosidl_default_generators REQUIRED) 
find_package(hardware_interface REQUIRED)
find_package(rclcpp_components REQUIRED)

# Find pigpio library for GPIO access
# Find pigpio library for GPIO access (installed in /usr/local/lib)
find_library(PIGPIO_LIBRARY pigpio PATHS /usr/local/lib REQUIRED)
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 PATHS /usr/local/lib REQUIRED)

# Print the current working directory (where this CMakeLists.txt is located)
message("Current Source Directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# Print the top-level project source directory
message("Top-Level Source Directory: ${CMAKE_SOURCE_DIR}")

# Print the current directory for build files
# message(STATUS "Current Binary Directory: ${CMAKE_CURRENT_BINARY_DIR}")
message("Current Binary Directory: ${CMAKE_CURRENT_BINARY_DIR}")


# --- Custom Message Configuration Start ---
set(
  MSG_FILES
  "msg/RpiSensorData.msg" # <-- THIS LINE IS CRITICAL: Ensures the correct path is used
)

# Generates the C++ (and other language) interfaces from the .msg file
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs # Depends on std_msgs/Header
)

# Export the runtime dependency so other packages can use the generated message
ament_export_dependencies(rosidl_default_runtime)

install(
  FILES ${MSG_FILES}
  DESTINATION share/${PROJECT_NAME}/msg
)




add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs)

# Link the generated message interfaces to the executable
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target_link_libraries(rpi_publisher 
#   "${cpp_typesupport_target}"
#   ${PIGPIO_LIBRARY}
#   ${PIGPIO_IF_LIBRARY}
# )

target_link_libraries(sensor_publisher_node
  "${cpp_typesupport_target}"
  ${PIGPIO_LIBRARY}
  ${PIGPIOD_IF2_LIBRARY}
  pthread
  rt
)

install(TARGETS
  sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)


# --- Motor Driver Executable ---
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs)
target_link_libraries(motor_driver_node
  ${PIGPIO_LIBRARY}
  ${PIGPIOD_IF2_LIBRARY}
  pthread
  rt
)

install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)


# --- Main Control Logic Executable (Needs Custom Message) ---
add_executable(main_control_logic_node src/main_control_logic.cpp)
ament_target_dependencies(main_control_logic_node rclcpp std_msgs) # Add custom message dependency if needed later

# Link the generated message interfaces to the executable
rosidl_get_typesupport_target(main_control_ts_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(main_control_logic_node
  "${main_control_ts_target}"
  ${PIGPIO_LIBRARY}
  ${PIGPIOD_IF2_LIBRARY}
  pthread
  rt
)

install(TARGETS
  main_control_logic_node
  DESTINATION lib/${PROJECT_NAME}
)


# --- Install Launch Files ---
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  # Include patterns for Python and XML launch files
  PATTERN "*.py"
  PATTERN "*.xml"
)

ament_package()

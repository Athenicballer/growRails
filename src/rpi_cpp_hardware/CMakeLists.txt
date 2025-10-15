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

# --- Find the pigpio library ---
# This looks for libpigpio.so in standard system paths, including /usr/local/lib
find_library(PIGPIO_LIBRARIES pigpio)

if(NOT PIGPIO_LIBRARIES)
  # Fallback for manual installs in /usr/local/
  message(STATUS "pigpio library not found in standard paths. Assuming /usr/local/lib.")
  include_directories(/usr/local/include)
  link_directories(/usr/local/lib)
  set(PIGPIO_LIBRARIES "-lpigpio") # Use -l flag for linker
endif()


# --- C++ Node Definitions ---
# The names used here MUST match the 'executable' argument in the launch file.


# 1. Motor Driver Node
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs)
# CRITICAL FIX: Add include directories for headers used in C++ files
target_include_directories(motor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} # <-- Required for geometry_msgs headers
)
target_link_libraries(motor_driver_node ${PIGPIO_LIBRARIES})

# 2. Sensor Publisher Node
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs)
# CRITICAL FIX: Add include directories for headers used in C++ files
target_include_directories(sensor_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} # <-- Required for std_msgs headers
)
target_link_libraries(sensor_publisher_node ${PIGPIO_LIBRARIES})

# 3. Main Control Logic Node (The one currently failing)
add_executable(main_control_logic_node src/main_control_logic.cpp)
ament_target_dependencies(main_control_logic_node rclcpp std_msgs geometry_msgs)
# CRITICAL FIX: Add include directories for headers used in C++ files
target_include_directories(main_control_logic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}     # <-- Required for std_msgs headers (your current error)
  ${geometry_msgs_INCLUDE_DIRS} # <-- Required if it also uses geometry_msgs (good practice)
)
target_link_libraries(main_control_logic_node ${PIGPIO_LIBRARIES})


# Install targets
install(TARGETS
  motor_driver_node
  sensor_publisher_node
  main_control_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

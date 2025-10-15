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

# --- CRITICAL: HARDCODED PIGPIO PATHS ---
# Since apt failed and pigpio was installed manually, we set the paths directly
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Use the pigpio library name for linking (it will now look in /usr/local/lib)
set(PIGPIO_LIBRARY "pigpio")

# --- C++ Node Definitions ---

# 1. Sensor Publisher Node
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs)
# Link the executable to the pigpio library
target_link_libraries(sensor_publisher_node ${PIGPIO_LIBRARY})

# 2. Motor Driver Node
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node rclcpp geometry_msgs)
# Link the executable to the pigpio library
target_link_libraries(motor_driver_node ${PIGPIO_LIBRARY})

# Install targets
install(TARGETS
  sensor_publisher_node
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

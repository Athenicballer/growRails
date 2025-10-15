cmake_minimum_required(VERSION 3.16)
project(rpi_cpp_hardware)

# Enforce good C++ standards
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17) # Added -std=c++17
endif()

# Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_components REQUIRED)
# You may need to add find_package(PIGPIO) here later if you use it directly
# but for a standard ROS node, this is usually enough.

# --- 1. DEFINE C++ EXECUTABLES ---

# List all C++ source files
set(RPI_NODE_SOURCES
  src/main_control_logic.cpp
  src/motor_driver.cpp
  src/sensor_publisher.cpp
)

# Macro to simplify adding and installing executables
function(add_rpi_executable TARGET_NAME SOURCE_FILE)
    # Add the C++ executable
    add_executable(${TARGET_NAME} ${SOURCE_FILE})

    # Link against required ROS 2 libraries
    ament_target_dependencies(${TARGET_NAME} 
      rclcpp 
      std_msgs 
      hardware_interface
      controller_interface
      rclcpp_components
      # Add -lpthread if you need it for complex timing/threading
    )

    # Install the C++ executable into the 'lib' directory
    install(TARGETS ${TARGET_NAME}
      DESTINATION lib/${PROJECT_NAME}
    )
endfunction()


# Create the three executables
add_rpi_executable(main_control_logic_node src/main_control_logic.cpp)
add_rpi_executable(motor_driver_node src/motor_driver.cpp)
add_rpi_executable(sensor_publisher_node src/sensor_publisher.cpp)


# --- 2. PYTHON LAUNCH FILE INSTALLATION ---

# Install the Python launch file into the 'share/package_name/launch' directory.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"
)


# --- 3. GENERAL PACKAGE INSTALLATION ---

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})


# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  hardware_interface
  controller_interface
  rclcpp_components
)

# Export targets so other packages can depend on them
ament_export_targets(
  main_control_logic_node 
  motor_driver_node
  sensor_publisher_node
  HAS_LIBRARY_TARGET
)

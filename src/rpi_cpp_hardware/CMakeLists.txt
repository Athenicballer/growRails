cmake_minimum_required(VERSION 3.8)
project(rpi_cpp_hardware)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Node for Main Control Logic (New Addition) ---
add_executable(main_control_logic_node src/main_control_logic.cpp)
# Assuming it uses rclcpp, std_msgs, and maybe geometry_msgs if it commands movement.
ament_target_dependencies(main_control_logic_node rclcpp std_msgs geometry_msgs) 
# Note: Since the core logic may not use pigpio, we skip linking pigpiod_if/pigpio here, 
# but if main_control_logic.cpp needs direct GPIO access, add it back.
install(TARGETS
  main_control_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Node for Motor Control ---
add_executable(motor_driver_node src/motor_driver.cpp)
ament_target_dependencies(motor_driver_node rclcpp std_msgs geometry_msgs)
# Link against both the client interface and the core pigpio library
target_link_libraries(motor_driver_node pigpiod_if pigpio)
install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Node for Sensor Reading ---
add_executable(sensor_publisher_node src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher_node rclcpp std_msgs)
# Link against both the client interface and the core pigpio library
target_link_libraries(sensor_publisher_node pigpiod_if pigpio)
install(TARGETS
  sensor_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install the launch files ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

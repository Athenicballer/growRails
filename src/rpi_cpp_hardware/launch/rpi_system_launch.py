import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches all three C++ nodes for the RPi robot system."""

    # Get the package directory
    pkg_name = 'rpi_cpp_hardware'
    pkg_dir = get_package_share_directory(pkg_name)

    # 1. Motor Driver Node (Subscribes to /cmd_vel)
    motor_node = Node(
        package=pkg_name,
        executable='motor_driver_node',
        name='motor_driver',
        output='screen',
        emulate_tty=True
    )

    # 2. Sensor Publisher Node (Publishes sensor readings)
    sensor_node = Node(
        package=pkg_name,
        executable='sensor_publisher_node',
        name='sensor_publisher',
        output='screen',
        emulate_tty=True
    )

    # 3. Main Control Logic Node (Reads sensors, publishes /cmd_vel)
    control_node = Node(
        package=pkg_name,
        executable='main_control_node',
        name='main_controller',
        output='screen',
        emulate_tty=True
    )

    # Return the aggregated launch description
    return LaunchDescription([
        motor_node,
        sensor_node,
        control_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the three C++ executables for the RPi robot system.

    The 'executable' arguments MUST match the target names 
    defined in CMakeLists.txt (e.g., 'motor_driver_node').
    """
    
    pkg_name = 'rpi_cpp_hardware'

    # 1. Motor Driver Node
    motor_node = Node(
        package=pkg_name,
        executable='motor_driver_node', 
        name='motor_driver',
        output='screen',
        emulate_tty=True, 
    )

    # 2. Sensor Publisher Node
    sensor_node = Node(
        package=pkg_name,
        executable='sensor_publisher_node',
        name='sensor_publisher',
        output='screen',
        emulate_tty=True,
    )

    # 3. Main Control Logic Node
    control_node = Node(
        package=pkg_name,
        executable='main_control_logic_node',
        name='main_control_logic',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        motor_node,
        sensor_node,
        control_node,
    ])

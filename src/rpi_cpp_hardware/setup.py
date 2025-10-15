import os
from glob import glob
from setuptools import setup

package_name = 'rpi_cpp_hardware'

setup(
    name=package_name,
    version='0.0.0',
    # We leave packages=[] empty because this package installs C++ executables, not Python modules.
    packages=[],
    
    # --- CRITICAL SECTION: Installing the Launch File ---
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs your launch file into the package's 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'rpi_system_launch.py'))),
    ],
    # ----------------------------------------------------

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grower',
    maintainer_email='user@raspberrypi.local',
    description='ROS 2 hardware package for RPi robot control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # No console scripts needed here since the C++ executables are handled by CMake
    entry_points={
        'console_scripts': [],
    },
)

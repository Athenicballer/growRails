import os
from glob import glob
from setuptools import setup

package_name = 'rpi_cpp_hardware'
launch_files = glob(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- CRITICAL SECTION: Explicitly installing launch files ---
        # The file is in launch/rpi_system_launch.py, and it must be installed
        # directly into share/rpi_cpp_hardware/
        (os.path.join('share', package_name), launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grower',
    maintainer_email='user@raspberrypi.local',
    description='ROS 2 hardware package for RPi robot control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

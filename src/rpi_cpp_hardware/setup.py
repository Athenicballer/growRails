import os
from glob import glob
from setuptools import setup

package_name = 'rpi_cpp_hardware'

setup(
    name=package_name,
    version='0.0.0',
    # We leave packages=[] empty because this package installs C++ executables
    packages=[],
    data_files=[
        # Install resources (package.xml and resource index)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- CRITICAL SECTION: Installing the Launch File ---
        # Install the Python launch file directly into the share/package_name directory.
        # This is the correct location for Python launch files to be found by `ros2 launch`.
        (os.path.join('share', package_name), glob(os.path.join('launch', '*.launch.py'))),
        
        # This line was unnecessary and has been removed:
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

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

import os
from setuptools import setup
from glob import glob

package_name = 'rpi_cpp_hardware'


setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- CRITICAL FIX: Using the fixed, more generic glob pattern ---
        # This will match 'rpi_system_launch.py' as requested,
        # assuming it is the only file ending in 'launch.py'.
        (os.path.join('share', package_name), glob('launch/*launch.py')),
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

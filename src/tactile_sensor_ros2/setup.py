from setuptools import setup
import os
from glob import glob

package_name = 'tactile_sensor_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS 2 package for tactile sensor data acquisition using CH341',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cap_read_node = tactile_sensor_ros2.cap_read_node:main',
            'data_subscriber = tactile_sensor_ros2.data_subscriber_node:main',
        ],
    },
)
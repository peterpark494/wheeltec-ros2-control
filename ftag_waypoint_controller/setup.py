from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ftag_waypoint_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='FT-AG kinematic waypoint guidance for WHEELTEC ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = ftag_waypoint_controller.controller_node:main',
            'map_line_node = ftag_waypoint_controller.map_line_node:main',
            'odom_line_node = ftag_waypoint_controller.odom_line_node:main',
            'odom_curve_node = ftag_waypoint_controller.odom_curve_node:main',
            'front_left_fault_node = '
            'ftag_waypoint_controller.front_left_fault_node:main',
        ],
    },
)

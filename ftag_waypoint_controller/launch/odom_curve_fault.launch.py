"""Launch quadratic curve tracking through the front-left fault injector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ftag_waypoint_controller')
    return LaunchDescription([
        Node(
            package='ftag_waypoint_controller',
            executable='odom_curve_node',
            name='odom_curve_controller',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'odom_curve.yaml'),
                {'cmd_vel_topic': '/cmd_vel_nominal'},
            ],
        ),
        Node(
            package='ftag_waypoint_controller',
            executable='front_left_fault_node',
            name='front_left_fault_injector',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'front_left_fault.yaml'),
            ],
        ),
    ])

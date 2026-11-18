"""Launch the waypoint controller with motion disabled by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ftag_waypoint_controller')
    return LaunchDescription([
        Node(
            package='ftag_waypoint_controller',
            executable='controller_node',
            name='ftag_waypoint_controller',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'controller.yaml'),
                os.path.join(share, 'config', 'waypoints.yaml'),
            ],
        ),
    ])

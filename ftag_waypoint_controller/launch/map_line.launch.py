"""Launch the fixed map-frame line controller with motion disabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ftag_waypoint_controller')
    return LaunchDescription([
        Node(
            package='ftag_waypoint_controller',
            executable='map_line_node',
            name='map_line_controller',
            output='screen',
            parameters=[os.path.join(share, 'config', 'map_line.yaml')],
        ),
    ])

"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
For now this launch file starts only the MLX90640 thermal node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mlx90640_node')
    params_file = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='mlx90640_node',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[params_file],
        ),
    ])

"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
The sensor packages stay focused on their own drivers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mlx90640_share = get_package_share_directory('mlx90640_node')
    mlx90640_params = os.path.join(mlx90640_share, 'config', 'params.yaml')
    start_rosbridge = LaunchConfiguration('start_rosbridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='Start rosbridge websocket on port 9090 for the frontend.',
        ),
        Node(
            package='mlx90640_node',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[mlx90640_params],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='thermal_rosbridge',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])

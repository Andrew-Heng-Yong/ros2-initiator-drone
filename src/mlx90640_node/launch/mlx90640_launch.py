import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('mlx90640_node')
    params_file = os.path.join(package_share, 'config', 'params.yaml')
    start_rosbridge = LaunchConfiguration('start_rosbridge')

    return LaunchDescription([
        # The dashboard sets this to true. Keeping it optional lets the driver still
        # be launched by itself on systems that do not have rosbridge installed.
        DeclareLaunchArgument('start_rosbridge', default_value='false'),
        Node(package='mlx90640_node', executable='mlx90640_node',
             name='mlx90640_node', output='screen', parameters=[params_file]),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='thermal_rosbridge', output='screen',
             condition=IfCondition(start_rosbridge)),
    ])

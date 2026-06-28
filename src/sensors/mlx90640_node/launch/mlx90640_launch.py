import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('mlx90640_node')
    params_file = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(package='mlx90640_node', executable='mlx90640_node',
             name='mlx90640_node', output='screen', parameters=[params_file]),
    ])

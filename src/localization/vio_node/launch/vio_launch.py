import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vio_node')
    default_params = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='VIO parameter YAML file.',
        ),
        Node(
            package='vio_node',
            executable='vio_node',
            name='vio_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])

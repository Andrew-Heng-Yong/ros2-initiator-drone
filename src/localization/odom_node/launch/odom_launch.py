import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('odom_node')
    default_params = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Odometry parameter YAML file.',
        ),
        Node(
            package='odom_node',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mi0802_senxor_driver')
    params_file = os.path.join(package_share, 'config', 'params.yaml')
    device = LaunchConfiguration('device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='MI0802 USB CDC ACM device path.',
        ),
        Node(
            package='mi0802_senxor_driver',
            executable='mi0802_senxor_node',
            name='mi0802_senxor_node',
            output='screen',
            parameters=[params_file, {'device': device}],
        ),
    ])

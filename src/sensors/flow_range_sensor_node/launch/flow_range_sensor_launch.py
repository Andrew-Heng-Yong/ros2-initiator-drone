import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('flow_range_sensor_node')
    default_params = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('spi_device', default_value='/dev/spidev0.1'),
        DeclareLaunchArgument('i2c_device', default_value='/dev/i2c-1'),
        DeclareLaunchArgument('flow_rotation', default_value='0'),
        Node(
            package='flow_range_sensor_node',
            executable='flow_range_sensor_node',
            name='flow_range_sensor_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'spi_device': LaunchConfiguration('spi_device'),
                'i2c_device': LaunchConfiguration('i2c_device'),
                'flow_rotation': ParameterValue(
                    LaunchConfiguration('flow_rotation'), value_type=int),
            }],
        ),
    ])

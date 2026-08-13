import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('odom_node')
    default_params = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Odometry parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'static_override',
            default_value='false',
            description='Publish a fixed calibrated origin pose for stationary bench testing.',
        ),
        DeclareLaunchArgument(
            'quality_override',
            default_value='false',
            description='Report gyro-only translation as observed for visualization clients.',
        ),
        Node(
            package='odom_node',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'static_override': ParameterValue(
                    LaunchConfiguration('static_override'), value_type=bool),
                'quality_override': ParameterValue(
                    LaunchConfiguration('quality_override'), value_type=bool),
            }],
        ),
    ])

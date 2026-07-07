import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('human_pose_detection')
    default_params_file = os.path.join(share_dir, 'config', 'human_box_tracker.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='ROS parameter YAML for the human box tracker node.',
        ),
        Node(
            package='human_pose_detection',
            executable='human_box_tracker_node',
            name='human_box_tracker_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])

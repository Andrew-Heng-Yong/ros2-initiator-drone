"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
The sensor packages stay focused on their own drivers.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def load_settings():
    share_dir = get_package_share_directory('drone_control')
    settings_path = os.environ.get(
        'DRONE_SETTINGS_FILE',
        os.path.join(share_dir, 'config', 'drone_settings.yaml'),
    )
    with open(settings_path, 'r', encoding='utf-8') as settings_file:
        return yaml.safe_load(settings_file) or {}


def generate_launch_description():
    settings = load_settings()
    human_tracker_share = get_package_share_directory('human_pose_detection')
    default_human_tracking_params_file = os.path.join(
        human_tracker_share,
        'config',
        'human_box_tracker.yaml',
    )
    camera = settings.get('camera', {})

    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_camera = LaunchConfiguration('start_camera')
    start_pose = LaunchConfiguration('start_pose')
    human_tracking_params_file = LaunchConfiguration('human_tracking_params_file')
    orbbec_setup = LaunchConfiguration('orbbec_setup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='Start rosbridge websocket on port 9090 for the frontend.',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='false',
            description='Start the Orbbec RGB camera driver.',
        ),
        DeclareLaunchArgument(
            'start_pose',
            default_value='true',
            description='Start the RGB-only TensorFlow Lite human box tracker.',
        ),
        DeclareLaunchArgument(
            'human_tracking_params_file',
            default_value=default_human_tracking_params_file,
            description='ROS parameter YAML for the human box tracker node.',
        ),
        DeclareLaunchArgument(
            'orbbec_setup',
            default_value=EnvironmentVariable(
                'ORBBEC_SETUP',
                default_value='/home/andrew/orbbec_ws/install/setup.bash',
            ),
            description='Path to the Orbbec driver workspace setup.bash.',
        ),
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                [
                    'if [ ! -f "',
                    orbbec_setup,
                    '" ]; then echo "Missing Orbbec setup file: ',
                    orbbec_setup,
                    '"; exit 1; fi; '
                    'source "',
                    orbbec_setup,
                    '"; '
                    f'exec ros2 launch orbbec_camera gemini_e.launch.py '
                    f'color_width:={int(camera.get("color_width", 640))} '
                    f'color_height:={int(camera.get("color_height", 360))} '
                    f'color_fps:={int(camera.get("color_fps", 5))} '
                    f'enable_depth:={str(camera.get("enable_depth", False)).lower()} '
                    f'depth_width:={int(camera.get("depth_width", 640))} '
                    f'depth_height:={int(camera.get("depth_height", 480))} '
                    f'depth_fps:={int(camera.get("depth_fps", 10))} '
                    f'enable_ir:={str(camera.get("enable_ir", False)).lower()}',
                ],
            ],
            output='screen',
            condition=IfCondition(start_camera),
        ),
        Node(
            package='human_pose_detection',
            executable='human_box_tracker_node',
            name='human_box_tracker_node',
            output='screen',
            condition=IfCondition(start_pose),
            parameters=[human_tracking_params_file],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='camera_rosbridge',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])

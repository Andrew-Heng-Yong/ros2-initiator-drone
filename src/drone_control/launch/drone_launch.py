"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
The sensor packages stay focused on their own drivers.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
    camera = settings.get('camera', {})
    thermal_camera = settings.get('thermal_camera', {})

    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_camera = LaunchConfiguration('start_camera')
    start_thermal_camera = LaunchConfiguration('start_thermal_camera')
    orbbec_setup = LaunchConfiguration('orbbec_setup')
    thermal_device = LaunchConfiguration('thermal_device')
    thermal_width = LaunchConfiguration('thermal_width')
    thermal_height = LaunchConfiguration('thermal_height')
    thermal_fps = LaunchConfiguration('thermal_fps')
    thermal_pixel_format = LaunchConfiguration('thermal_pixel_format')
    thermal_output_encoding = LaunchConfiguration('thermal_output_encoding')
    thermal_frame_id = LaunchConfiguration('thermal_frame_id')

    def create_thermal_camera_node(context):
        return [Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='thermal_camera',
            namespace='thermal',
            output='screen',
            condition=IfCondition(start_thermal_camera),
            parameters=[{
                'video_device': thermal_device.perform(context),
                'image_size': [
                    int(thermal_width.perform(context)),
                    int(thermal_height.perform(context)),
                ],
                'time_per_frame': [
                    1,
                    int(thermal_fps.perform(context)),
                ],
                'pixel_format': thermal_pixel_format.perform(context),
                'output_encoding': thermal_output_encoding.perform(context),
                'camera_frame_id': thermal_frame_id.perform(context),
            }],
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='Start rosbridge websocket on port 9090 for the frontend.',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='false',
            description='Start the Orbbec RGB/depth camera driver.',
        ),
        DeclareLaunchArgument(
            'start_thermal_camera',
            default_value='false',
            description='Start the V4L2 thermal camera driver.',
        ),
        DeclareLaunchArgument(
            'orbbec_setup',
            default_value=EnvironmentVariable(
                'ORBBEC_SETUP',
                default_value='/home/andrew/orbbec_ws/install/setup.bash',
            ),
            description='Path to the Orbbec driver workspace setup.bash.',
        ),
        DeclareLaunchArgument(
            'thermal_device',
            default_value=str(thermal_camera.get('device', '/dev/video0')),
            description='V4L2 device path for the thermal camera.',
        ),
        DeclareLaunchArgument(
            'thermal_width',
            default_value=str(thermal_camera.get('width', 256)),
            description='Thermal camera image width.',
        ),
        DeclareLaunchArgument(
            'thermal_height',
            default_value=str(thermal_camera.get('height', 192)),
            description='Thermal camera image height.',
        ),
        DeclareLaunchArgument(
            'thermal_fps',
            default_value=str(thermal_camera.get('fps', 25)),
            description='Thermal camera frames per second.',
        ),
        DeclareLaunchArgument(
            'thermal_pixel_format',
            default_value=str(thermal_camera.get('pixel_format', 'YUYV')),
            description='V4L2 thermal camera pixel format.',
        ),
        DeclareLaunchArgument(
            'thermal_output_encoding',
            default_value=str(thermal_camera.get('output_encoding', 'yuv422_yuy2')),
            description='ROS image encoding published by v4l2_camera.',
        ),
        DeclareLaunchArgument(
            'thermal_frame_id',
            default_value=str(thermal_camera.get('frame_id', 'thermal_camera_frame')),
            description='Frame id for thermal camera images.',
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
                    f'color_height:={int(camera.get("color_height", 480))} '
                    f'color_fps:={int(camera.get("color_fps", 5))} '
                    f'enable_depth:={str(camera.get("enable_depth", True)).lower()} '
                    f'depth_width:={int(camera.get("depth_width", 640))} '
                    f'depth_height:={int(camera.get("depth_height", 480))} '
                    f'depth_fps:={int(camera.get("depth_fps", 10))} '
                    f'enable_ir:={str(camera.get("enable_ir", False)).lower()}',
                ],
            ],
            output='screen',
            condition=IfCondition(start_camera),
        ),
        OpaqueFunction(function=create_thermal_camera_node),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='camera_rosbridge',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])

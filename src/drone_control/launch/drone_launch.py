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
from launch_ros.parameter_descriptions import ParameterValue


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
    pose = settings.get('pose', {})

    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_camera = LaunchConfiguration('start_camera')
    start_thermal_camera = LaunchConfiguration('start_thermal_camera')
    start_pose = LaunchConfiguration('start_pose')
    pose_model_path = LaunchConfiguration('pose_model_path')
    pose_image_topic = LaunchConfiguration('pose_image_topic')
    pose_confidence_threshold = LaunchConfiguration('pose_confidence_threshold')
    pose_min_confident_keypoints = LaunchConfiguration('pose_min_confident_keypoints')
    pose_max_inference_fps = LaunchConfiguration('pose_max_inference_fps')
    pose_publish_debug_image = LaunchConfiguration('pose_publish_debug_image')
    orbbec_setup = LaunchConfiguration('orbbec_setup')
    thermal_device = LaunchConfiguration('thermal_device')
    thermal_width = LaunchConfiguration('thermal_width')
    thermal_height = LaunchConfiguration('thermal_height')
    thermal_fps = LaunchConfiguration('thermal_fps')
    thermal_pixel_format = LaunchConfiguration('thermal_pixel_format')
    thermal_output_encoding = LaunchConfiguration('thermal_output_encoding')
    thermal_frame_id = LaunchConfiguration('thermal_frame_id')

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
            'start_thermal_camera',
            default_value='false',
            description='Start the V4L2 thermal camera driver.',
        ),
        DeclareLaunchArgument(
            'start_pose',
            default_value='true',
            description='Start the RGB-only MoveNet human pose detection node.',
        ),
        DeclareLaunchArgument(
            'pose_model_path',
            default_value='',
            description='Path to the MoveNet Lightning INT8 TensorFlow Lite model.',
        ),
        DeclareLaunchArgument(
            'pose_image_topic',
            default_value=str(pose.get('image_topic', '/camera/color/image_raw')),
            description='RGB image topic consumed by the pose detector.',
        ),
        DeclareLaunchArgument(
            'pose_confidence_threshold',
            default_value=str(pose.get('confidence_threshold', 0.2)),
            description='Minimum keypoint confidence used for person detection.',
        ),
        DeclareLaunchArgument(
            'pose_min_confident_keypoints',
            default_value=str(pose.get('min_confident_keypoints', 5)),
            description='Minimum keypoints above threshold required to call a person detected.',
        ),
        DeclareLaunchArgument(
            'pose_max_inference_fps',
            default_value=str(pose.get('max_inference_fps', 5.0)),
            description='Maximum MoveNet inference rate. Use 0 to process every camera frame.',
        ),
        DeclareLaunchArgument(
            'pose_publish_debug_image',
            default_value=str(pose.get('publish_debug_image', True)).lower(),
            description='Publish annotated pose debug image.',
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
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='thermal_camera',
            namespace='thermal',
            output='screen',
            condition=IfCondition(start_thermal_camera),
            parameters=[{
                'video_device': thermal_device,
                'image_size': [
                    ParameterValue(thermal_width, value_type=int),
                    ParameterValue(thermal_height, value_type=int),
                ],
                'time_per_frame': [
                    1,
                    ParameterValue(thermal_fps, value_type=int),
                ],
                'pixel_format': thermal_pixel_format,
                'output_encoding': thermal_output_encoding,
                'camera_frame_id': thermal_frame_id,
            }],
        ),
        Node(
            package='human_pose_detection',
            executable='movenet_pose_node',
            name='movenet_pose_node',
            output='screen',
            condition=IfCondition(start_pose),
            parameters=[{
                'model_path': pose_model_path,
                'image_topic': pose_image_topic,
                'confidence_threshold': ParameterValue(pose_confidence_threshold, value_type=float),
                'min_confident_keypoints': ParameterValue(pose_min_confident_keypoints, value_type=int),
                'max_inference_fps': ParameterValue(pose_max_inference_fps, value_type=float),
                'publish_debug_image': ParameterValue(pose_publish_debug_image, value_type=bool),
            }],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='camera_rosbridge',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])

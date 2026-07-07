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
    thermal_camera = settings.get('archived_thermal_camera', settings.get('thermal_camera', {}))
    human_tracking = settings.get('human_tracking', settings.get('pose', {}))

    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_camera = LaunchConfiguration('start_camera')
    start_thermal_camera = LaunchConfiguration('start_thermal_camera')
    start_pose = LaunchConfiguration('start_pose')
    human_tracking_model_path = LaunchConfiguration('human_tracking_model_path')
    human_tracking_model_name = LaunchConfiguration('human_tracking_model_name')
    human_tracking_image_topic = LaunchConfiguration('human_tracking_image_topic')
    human_tracking_confidence_threshold = LaunchConfiguration('human_tracking_confidence_threshold')
    human_tracking_max_detections = LaunchConfiguration('human_tracking_max_detections')
    human_tracking_track_iou_threshold = LaunchConfiguration('human_tracking_track_iou_threshold')
    human_tracking_max_track_missed_frames = LaunchConfiguration('human_tracking_max_track_missed_frames')
    human_tracking_max_inference_fps = LaunchConfiguration('human_tracking_max_inference_fps')
    human_tracking_publish_debug_image = LaunchConfiguration('human_tracking_publish_debug_image')
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
            description='Start the Orbbec RGB camera driver.',
        ),
        DeclareLaunchArgument(
            'start_thermal_camera',
            default_value='false',
            description='Start the archived V4L2 thermal camera driver.',
        ),
        DeclareLaunchArgument(
            'start_pose',
            default_value='true',
            description='Start the RGB-only TensorFlow Lite human box tracker.',
        ),
        DeclareLaunchArgument(
            'human_tracking_model_path',
            default_value=str(human_tracking.get('model_path', '/home/andrew/models/efficientdet_lite0.tflite')),
            description='Hardcoded path to the ultra-light TensorFlow Lite person-box model.',
        ),
        DeclareLaunchArgument(
            'human_tracking_model_name',
            default_value=str(human_tracking.get('model_name', 'efficientdet_lite0_person_boxes')),
            description='Hardcoded model selection for the TensorFlow Lite human box tracker.',
        ),
        DeclareLaunchArgument(
            'human_tracking_image_topic',
            default_value=str(human_tracking.get('image_topic', '/camera/color/image_raw')),
            description='RGB image topic consumed by the human box tracker.',
        ),
        DeclareLaunchArgument(
            'human_tracking_confidence_threshold',
            default_value=str(human_tracking.get('confidence_threshold', 0.35)),
            description='Minimum person box confidence used for detection.',
        ),
        DeclareLaunchArgument(
            'human_tracking_max_detections',
            default_value=str(human_tracking.get('max_detections', 8)),
            description='Maximum human boxes to publish per frame.',
        ),
        DeclareLaunchArgument(
            'human_tracking_track_iou_threshold',
            default_value=str(human_tracking.get('track_iou_threshold', 0.3)),
            description='Minimum box overlap used to keep a tracked human id.',
        ),
        DeclareLaunchArgument(
            'human_tracking_max_track_missed_frames',
            default_value=str(human_tracking.get('max_track_missed_frames', 5)),
            description='Frames a tracked human can be missing before its id is retired.',
        ),
        DeclareLaunchArgument(
            'human_tracking_max_inference_fps',
            default_value=str(human_tracking.get('max_inference_fps', 5.0)),
            description='Maximum box-tracker inference rate. Use 0 to process every camera frame.',
        ),
        DeclareLaunchArgument(
            'human_tracking_publish_debug_image',
            default_value=str(human_tracking.get('publish_debug_image', True)).lower(),
            description='Publish annotated human box debug image.',
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
        OpaqueFunction(function=create_thermal_camera_node),
        Node(
            package='human_pose_detection',
            executable='human_box_tracker_node',
            name='human_box_tracker_node',
            output='screen',
            condition=IfCondition(start_pose),
            parameters=[{
                'model_path': human_tracking_model_path,
                'model_name': human_tracking_model_name,
                'image_topic': human_tracking_image_topic,
                'confidence_threshold': ParameterValue(human_tracking_confidence_threshold, value_type=float),
                'max_detections': ParameterValue(human_tracking_max_detections, value_type=int),
                'track_iou_threshold': ParameterValue(human_tracking_track_iou_threshold, value_type=float),
                'max_track_missed_frames': ParameterValue(human_tracking_max_track_missed_frames, value_type=int),
                'max_inference_fps': ParameterValue(human_tracking_max_inference_fps, value_type=float),
                'publish_debug_image': ParameterValue(human_tracking_publish_debug_image, value_type=bool),
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

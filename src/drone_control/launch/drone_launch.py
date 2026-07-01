"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
The sensor packages stay focused on their own drivers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_camera = LaunchConfiguration('start_camera')
    start_pose = LaunchConfiguration('start_pose')
    pose_model_path = LaunchConfiguration('pose_model_path')
    pose_image_topic = LaunchConfiguration('pose_image_topic')
    pose_confidence_threshold = LaunchConfiguration('pose_confidence_threshold')
    pose_max_inference_fps = LaunchConfiguration('pose_max_inference_fps')
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
            description='Start the RGB-only MoveNet human pose detection node.',
        ),
        DeclareLaunchArgument(
            'pose_model_path',
            default_value='',
            description='Path to the MoveNet Lightning INT8 TensorFlow Lite model.',
        ),
        DeclareLaunchArgument(
            'pose_image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by the pose detector.',
        ),
        DeclareLaunchArgument(
            'pose_confidence_threshold',
            default_value='0.2',
            description='Minimum keypoint confidence used for person detection.',
        ),
        DeclareLaunchArgument(
            'pose_max_inference_fps',
            default_value='5.0',
            description='Maximum MoveNet inference rate. Use 0 to process every camera frame.',
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
                    'exec ros2 launch orbbec_camera gemini_e.launch.py '
                    'color_width:=640 '
                    'color_height:=480 '
                    'color_fps:=5 '
                    'enable_depth:=false '
                    'depth_width:=640 '
                    'depth_height:=480 '
                    'depth_fps:=10 '
                    'enable_ir:=false',
                ],
            ],
            output='screen',
            condition=IfCondition(start_camera),
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
                'max_inference_fps': ParameterValue(pose_max_inference_fps, value_type=float),
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

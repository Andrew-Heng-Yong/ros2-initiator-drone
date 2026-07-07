from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/andrew/models/efficientdet_lite0.tflite',
            description='Path to the ultra-light TensorFlow Lite person-box model.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='efficientdet_lite0_person_boxes',
            description='Hardcoded model selection for the human box tracker.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='RGB camera image topic.',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.35',
            description='Minimum person box score used for detection.',
        ),
        DeclareLaunchArgument(
            'max_detections',
            default_value='8',
            description='Maximum human boxes to publish per frame.',
        ),
        DeclareLaunchArgument(
            'track_iou_threshold',
            default_value='0.3',
            description='Minimum box overlap used to keep a tracked human id.',
        ),
        Node(
            package='human_pose_detection',
            executable='human_box_tracker_node',
            name='human_box_tracker_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'model_name': LaunchConfiguration('model_name'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'),
                    value_type=float,
                ),
                'max_detections': ParameterValue(
                    LaunchConfiguration('max_detections'),
                    value_type=int,
                ),
                'track_iou_threshold': ParameterValue(
                    LaunchConfiguration('track_iou_threshold'),
                    value_type=float,
                ),
            }],
        ),
    ])

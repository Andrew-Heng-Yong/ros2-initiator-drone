from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Path to the MoveNet Lightning INT8 .tflite model.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='RGB camera image topic.',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
            description='Minimum keypoint score used for detection.',
        ),
        Node(
            package='human_pose_detection',
            executable='movenet_pose_node',
            name='movenet_pose_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'),
                    value_type=float,
                ),
            }],
        ),
    ])

"""Top-level launch file for the drone's ROS 2 software.

Add future drone nodes here (flight controller, camera, telemetry, and so on).
The sensor packages stay focused on their own drivers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mlx90640_share = get_package_share_directory('mlx90640_node')
    mlx90640_params = os.path.join(mlx90640_share, 'config', 'params.yaml')
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_depth_camera = LaunchConfiguration('start_depth_camera')
    start_thermal_overlay = LaunchConfiguration('start_thermal_overlay')
    overlay_alpha = LaunchConfiguration('overlay_alpha')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='Start rosbridge websocket on port 9090 for the frontend.',
        ),
        DeclareLaunchArgument(
            'start_depth_camera',
            default_value='false',
            description='Start the Orbbec depth/color camera driver.',
        ),
        DeclareLaunchArgument(
            'start_thermal_overlay',
            default_value='false',
            description='Start the RGB camera thermal overlay node.',
        ),
        DeclareLaunchArgument(
            'overlay_alpha',
            default_value='0.45',
            description='Thermal overlay opacity, from 0.0 to 1.0.',
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py',
                'color_width:=640',
                'color_height:=480',
                'color_fps:=10',
                'enable_depth:=true',
                'depth_width:=640',
                'depth_height:=480',
                'depth_fps:=10',
                'enable_ir:=false',
            ],
            output='screen',
            condition=IfCondition(start_depth_camera),
        ),
        Node(
            package='mlx90640_node',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[mlx90640_params],
        ),
        Node(
            package='mlx90640_node',
            executable='thermal_overlay_node',
            name='thermal_overlay_node',
            output='screen',
            condition=IfCondition(start_thermal_overlay),
            parameters=[{
                'alpha': ParameterValue(overlay_alpha, value_type=float),
                'camera_topic': '/camera/color/image_raw',
                'thermal_topic': '/thermal/image_raw',
                'output_topic': '/camera/thermal_overlay/image_raw',
                'camera_hfov_deg': 67.0,
                'camera_vfov_deg': 53.6,
                'thermal_hfov_deg': 55.0,
                'thermal_vfov_deg': 35.0,
            }],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='thermal_rosbridge',
            output='screen',
            condition=IfCondition(start_rosbridge),
        ),
    ])

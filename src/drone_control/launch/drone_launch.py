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
    mi0802_share = get_package_share_directory('mi0802_senxor_driver')
    mi0802_params = os.path.join(mi0802_share, 'config', 'params.yaml')
    mlx90640_share = get_package_share_directory('mlx90640_node')
    mlx90640_params = os.path.join(mlx90640_share, 'config', 'params.yaml')
    mpu6050_share = get_package_share_directory('mpu6050_node')
    mpu6050_params = os.path.join(mpu6050_share, 'config', 'params.yaml')
    vio_share = get_package_share_directory('vio_node')
    vio_params = os.path.join(vio_share, 'config', 'params.yaml')
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    start_depth_camera = LaunchConfiguration('start_depth_camera')
    start_imu = LaunchConfiguration('start_imu')
    start_vio = LaunchConfiguration('start_vio')
    enable_color_camera = LaunchConfiguration('enable_color_camera')
    color_fps = LaunchConfiguration('color_fps')
    depth_fps = LaunchConfiguration('depth_fps')
    vio_static_override = LaunchConfiguration('vio_static_override')
    start_thermal_overlay = LaunchConfiguration('start_thermal_overlay')
    start_thermal_cropper = LaunchConfiguration('start_thermal_cropper')
    thermal_device = LaunchConfiguration('thermal_device')
    thermal_cropper_enabled = LaunchConfiguration('thermal_cropper_enabled')
    passthrough_when_no_region = LaunchConfiguration('passthrough_when_no_region')
    depth_output_decimation = LaunchConfiguration('depth_output_decimation')
    overlay_alpha = LaunchConfiguration('overlay_alpha')
    crop_unit_thermal_pixels = LaunchConfiguration('crop_unit_thermal_pixels')
    min_region_size = LaunchConfiguration('min_region_size')
    inflation_radius_thermal_pixels = LaunchConfiguration('inflation_radius_thermal_pixels')
    highlight_min_temp = LaunchConfiguration('highlight_min_temp')
    highlight_max_temp = LaunchConfiguration('highlight_max_temp')
    highlight_min_delta_from_frame_low = LaunchConfiguration('highlight_min_delta_from_frame_low')
    highlight_max_delta_from_frame_high = LaunchConfiguration('highlight_max_delta_from_frame_high')
    depth_fov_horizontal = LaunchConfiguration('depth_fov_horizontal')
    depth_fov_vertical = LaunchConfiguration('depth_fov_vertical')
    thermal_fov_horizontal = LaunchConfiguration('thermal_fov_horizontal')
    thermal_fov_vertical = LaunchConfiguration('thermal_fov_vertical')
    thermal_offset_x = LaunchConfiguration('thermal_offset_x')
    thermal_offset_y = LaunchConfiguration('thermal_offset_y')
    thermal_scale = LaunchConfiguration('thermal_scale')
    thermal_barrel_distortion = LaunchConfiguration('thermal_barrel_distortion')
    thermal_stretch_x = LaunchConfiguration('thermal_stretch_x')
    thermal_stretch_y = LaunchConfiguration('thermal_stretch_y')
    flip_thermal_x = LaunchConfiguration('flip_thermal_x')
    flip_thermal_y = LaunchConfiguration('flip_thermal_y')

    depth_camera_process = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py',
            ['enable_color:=', enable_color_camera],
            ['color_fps:=', color_fps],
            'enable_depth:=true',
            'depth_width:=640',
            'depth_height:=480',
            ['depth_fps:=', depth_fps],
            'enable_ir:=false',
        ],
        output='screen',
        condition=IfCondition(start_depth_camera),
    )
    thermal_driver_process = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            [
                'for attempt in $(seq 1 30); do '
                'ros2 topic info /camera/depth/image_raw > /dev/null 2>&1 && break; '
                'echo "Waiting for depth topic /camera/depth/image_raw..."; '
                'sleep 1; '
                'done; '
                'ros2 topic info /camera/depth/image_raw > /dev/null 2>&1 || '
                '(echo "Timed out waiting for depth topic /camera/depth/image_raw"; exit 1); '
                f'exec ros2 run mi0802_senxor_driver mi0802_senxor_node --ros-args '
                f'--params-file "{mi0802_params}" -p device:=',
                thermal_device,
            ],
        ],
        output='screen',
        condition=IfCondition(start_depth_camera),
    )
    thermal_cropper_node = Node(
        package='mlx90640_node',
        executable='thermal_cropper_node',
        name='thermal_cropper_node',
        output='screen',
        condition=IfCondition(start_thermal_cropper),
        parameters=[mlx90640_params, {
            'enabled': ParameterValue(thermal_cropper_enabled, value_type=bool),
            'passthrough_when_no_region': ParameterValue(passthrough_when_no_region, value_type=bool),
            'output_decimation': ParameterValue(depth_output_decimation, value_type=int),
            'crop_unit_thermal_pixels': ParameterValue(crop_unit_thermal_pixels, value_type=int),
            'min_region_size': ParameterValue(min_region_size, value_type=int),
            'inflation_radius_thermal_pixels': ParameterValue(inflation_radius_thermal_pixels, value_type=int),
            'highlight_min_temp': ParameterValue(highlight_min_temp, value_type=float),
            'highlight_max_temp': ParameterValue(highlight_max_temp, value_type=float),
            'highlight_min_delta_from_frame_low': ParameterValue(highlight_min_delta_from_frame_low, value_type=float),
            'highlight_max_delta_from_frame_high': ParameterValue(highlight_max_delta_from_frame_high, value_type=float),
            'depth_fov_horizontal': ParameterValue(depth_fov_horizontal, value_type=float),
            'depth_fov_vertical': ParameterValue(depth_fov_vertical, value_type=float),
            'thermal_fov_horizontal': ParameterValue(thermal_fov_horizontal, value_type=float),
            'thermal_fov_vertical': ParameterValue(thermal_fov_vertical, value_type=float),
            'thermal_offset_x': ParameterValue(thermal_offset_x, value_type=float),
            'thermal_offset_y': ParameterValue(thermal_offset_y, value_type=float),
            'thermal_scale': ParameterValue(thermal_scale, value_type=float),
            'thermal_barrel_distortion': ParameterValue(
                thermal_barrel_distortion, value_type=float),
            'thermal_stretch_x': ParameterValue(thermal_stretch_x, value_type=float),
            'thermal_stretch_y': ParameterValue(thermal_stretch_y, value_type=float),
            'flip_thermal_x': ParameterValue(flip_thermal_x, value_type=bool),
            'flip_thermal_y': ParameterValue(flip_thermal_y, value_type=bool),
        }],
    )
    thermal_overlay_node = Node(
        package='mlx90640_node',
        executable='thermal_overlay_node',
        name='thermal_overlay_node',
        output='screen',
        condition=IfCondition(start_thermal_overlay),
        parameters=[{
            'alpha': ParameterValue(overlay_alpha, value_type=float),
            'camera_topic': '/camera/depth/image_raw',
            'thermal_topic': '/thermal/image_raw',
            'output_topic': '/camera/thermal_overlay/image_raw',
            'camera_hfov_deg': 67.0,
            'camera_vfov_deg': 53.6,
            'thermal_hfov_deg': 55.0,
            'thermal_vfov_deg': 35.0,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='false',
            description='Start rosbridge websocket on port 9090 for the frontend.',
        ),
        DeclareLaunchArgument(
            'start_depth_camera',
            default_value='false',
            description='Start the Orbbec depth camera driver.',
        ),
        DeclareLaunchArgument(
            'start_vio',
            default_value='false',
            description='Start VIO and enable its IMU/RGB inputs by default.',
        ),
        DeclareLaunchArgument(
            'start_imu',
            default_value=start_vio,
            description='Start the MPU6050 IMU driver (defaults to start_vio).',
        ),
        DeclareLaunchArgument(
            'enable_color_camera',
            default_value=start_vio,
            description='Enable Orbbec RGB on /camera/color/image_raw (defaults to start_vio).',
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='5',
            description='Orbbec RGB stream frame rate used by VIO.',
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value='10',
            description='Orbbec depth frame rate; the phone stream is cropped and decimated later.',
        ),
        DeclareLaunchArgument(
            'vio_static_override',
            default_value='false',
            description='Skip VIO alignment and publish a fixed zero-motion estimate.',
        ),
        DeclareLaunchArgument(
            'start_thermal_overlay',
            default_value='false',
            description='Start the camera thermal overlay node.',
        ),
        DeclareLaunchArgument(
            'start_thermal_cropper',
            default_value='false',
            description='Start the thermal-guided depth cropper node.',
        ),
        DeclareLaunchArgument(
            'thermal_device',
            default_value='/dev/ttyACM0',
            description='MI0802 USB CDC ACM device path.',
        ),
        DeclareLaunchArgument(
            'thermal_cropper_enabled',
            default_value='true',
            description='Enable thermal-guided masking in the cropper node at startup.',
        ),
        DeclareLaunchArgument(
            'passthrough_when_no_region',
            default_value='true',
            description='Publish uncropped frames while no thermal crop region is available.',
        ),
        DeclareLaunchArgument(
            'depth_output_decimation',
            default_value='2',
            description='Keep every Nth depth pixel per axis on the cropped rosbridge stream.',
        ),
        DeclareLaunchArgument(
            'overlay_alpha',
            default_value='0.45',
            description='Thermal overlay opacity, from 0.0 to 1.0.',
        ),
        DeclareLaunchArgument(
            'crop_unit_thermal_pixels',
            default_value='2',
            description='Thermal-pixel square size for one cropper analysis unit.',
        ),
        DeclareLaunchArgument(
            'min_region_size',
            default_value='4',
            description='Minimum neighboring highlighted thermal-unit cluster size.',
        ),
        DeclareLaunchArgument(
            'inflation_radius_thermal_pixels',
            default_value='0',
            description='Cropper mask inflation radius in thermal pixels.',
        ),
        DeclareLaunchArgument(
            'highlight_min_temp',
            default_value='25.0',
            description='Minimum absolute thermal value for a highlighted pixel.',
        ),
        DeclareLaunchArgument(
            'highlight_max_temp',
            default_value='40.0',
            description='Maximum absolute thermal value for a highlighted pixel.',
        ),
        DeclareLaunchArgument(
            'highlight_min_delta_from_frame_low',
            default_value='3.0',
            description='Minimum delta above the current frame low for a highlighted pixel.',
        ),
        DeclareLaunchArgument(
            'highlight_max_delta_from_frame_high',
            default_value='1000.0',
            description='Maximum delta below the current frame high for a highlighted pixel.',
        ),
        DeclareLaunchArgument(
            'depth_fov_horizontal',
            default_value='79.0',
            description='Depth camera horizontal field of view in degrees.',
        ),
        DeclareLaunchArgument(
            'depth_fov_vertical',
            default_value='62.0',
            description='Depth camera vertical field of view in degrees.',
        ),
        DeclareLaunchArgument(
            'thermal_fov_horizontal',
            default_value='90.0',
            description='Thermal camera horizontal field of view in degrees.',
        ),
        DeclareLaunchArgument(
            'thermal_fov_vertical',
            default_value='68.0',
            description='Thermal camera vertical field of view in degrees.',
        ),
        DeclareLaunchArgument(
            'thermal_offset_x',
            default_value='0.0',
            description='Thermal-to-depth horizontal offset in depth pixels.',
        ),
        DeclareLaunchArgument(
            'thermal_offset_y',
            default_value='0.0',
            description='Thermal-to-depth vertical offset in depth pixels.',
        ),
        DeclareLaunchArgument(
            'thermal_scale',
            default_value='1.0',
            description='Uniform thermal-to-depth overlay scale.',
        ),
        DeclareLaunchArgument(
            'thermal_barrel_distortion',
            default_value='0.0',
            description='Signed radial distortion used to align thermal and depth image edges.',
        ),
        DeclareLaunchArgument(
            'thermal_stretch_x',
            default_value='0.8',
            description='Horizontal thermal-to-depth overlay stretch.',
        ),
        DeclareLaunchArgument(
            'thermal_stretch_y',
            default_value='0.9',
            description='Vertical thermal-to-depth overlay stretch.',
        ),
        DeclareLaunchArgument(
            'flip_thermal_x',
            default_value='true',
            description='Flip thermal coordinates horizontally.',
        ),
        DeclareLaunchArgument(
            'flip_thermal_y',
            default_value='false',
            description='Flip thermal coordinates vertically.',
        ),
        depth_camera_process,
        thermal_driver_process,
        thermal_cropper_node,
        thermal_overlay_node,
        Node(
            package='mpu6050_node',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            condition=IfCondition(start_imu),
            parameters=[mpu6050_params],
        ),
        Node(
            package='vio_node',
            executable='vio_node',
            name='vio_node',
            output='screen',
            condition=IfCondition(start_vio),
            parameters=[vio_params, {
                'static_override': ParameterValue(vio_static_override, value_type=bool),
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

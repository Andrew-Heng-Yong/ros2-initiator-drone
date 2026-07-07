from setuptools import find_packages, setup

package_name = 'human_pose_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', ['launch/movenet_pose_launch.py']),
        (f'share/{package_name}/config', ['config/human_box_tracker.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Workspace maintainer',
    maintainer_email='user@example.com',
    description='RGB-only TensorFlow Lite human tracking nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'movenet_pose_node = human_pose_detection.movenet_pose_node:main',
            'human_box_tracker_node = human_pose_detection.human_box_tracker_node:main',
            'check_runtime = human_pose_detection.check_runtime:main',
        ],
    },
)

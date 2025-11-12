from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            remappings=[
                ('image_rect', '/rgb_camera/image'),
                ('camera_info', '/rgb_camera/camera_info'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.05,   # tag size in meters
                'max_hamming': 0,
                'detector.threads': 4,
                'detector.decimate': 1.0,
                'detector.blur': 0.0,
                'detector.refine': True,
                'detector.sharpening': 0.25,
                'detector.debug': False,
            }]
        )
    ])

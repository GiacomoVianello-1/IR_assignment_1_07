from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('assignment_1_07'),
        'config',
        'apriltag_params.yaml'
    )

    # path to another launch file to include
    other_launch = os.path.join(
        get_package_share_directory('ir_launch'),
        'launch',
        'assignment_1.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch),
        ),
        # AprilTag detection node      
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            remappings=[
                ('image_rect', '/rgb_camera/image'),
                ('camera_info', '/rgb_camera/camera_info'),
            ],
            parameters=[cfg]
        ),

        # Goal selector node
        Node(
            package='assignment_1_07',
            executable='goal_selector',
            name='goal_selector',
            output='screen',
            parameters=[{
                'target_frame': 'odom',            # change in 'map' if using SLAM
                'tag_frame_prefix': 'tag36h11:',   # must match the prefix in apriltag_params.yaml
                'tf_timeout_sec': 0.3
            }]
        ),
    ])

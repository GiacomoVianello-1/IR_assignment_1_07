from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
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

        # Navigation2 orchestrator node: activate nav2 stack
        Node(
            package='assignment_1_07',
            executable='nav2_orchestrator',
            name='nav2_orchestrator',
            output='screen',
            parameters=[{
                'initial_x': 0.0,
                'initial_y': 0.0,
                'initial_yaw': 0.0,
                'covariance_x': 0.5,
                'covariance_y': 0.5,
                'covariance_yaw': 0.1,
                'service_wait_timeout_sec': 10.0,
                'call_timeout_sec': 10.0,
                'amcl_pose_wait_sec': 10.0
            }]
        ),

        # Goal selector node
        Node(
            package='assignment_1_07',
            executable='goal_selector',
            name='goal_selector',
            output='screen',
            parameters=[{
                'target_frame': 'map',            
                'tag_frame_prefix': 'tag36h11:',   # must match the prefix in apriltag_params.yaml
                'tf_timeout_sec': 0.3
            }]
        ),

        # Goal sender node: Delayed start to ensure everything else is up and running
        
        Node(
            package='assignment_1_07',
            executable='goal_sender',
            name='goal_sender',
            output='screen',
            parameters=[{
                'tag_id_1': 1,          # first tag
                'tag_id_2': 10          # second tag
            }]  
        )

    ])

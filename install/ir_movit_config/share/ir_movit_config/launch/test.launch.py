from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur_gripper", package_name="ir_movit_config")
        .robot_description(file_path="urdf/ur_gripper.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path="config/ur_gripper.srdf")
        .planning_pipelines()  # Automatically discovers the planning pipeline config
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # RViz config file path
    rviz_config_file = os.path.join(get_package_share_directory("ir_movit_config"), "config", "moveit.rviz")

    # Create RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()]
    )

    # Return launch description with both move_group and rviz nodes
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        ),
        rviz_node,
    ])


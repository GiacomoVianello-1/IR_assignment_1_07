from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # Build the MoveIt configuration
    # Note: We're avoiding the unsupported 'robot_description_ros2_control' method
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur_gripper", package_name="ir_movit_config")
        .robot_description(file_path="urdf/ur_gripper.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path="config/ur_gripper.srdf")
        .planning_pipelines()
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # File paths for configuration files
    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("ir_movit_config"),
        "config",
        "ros2_controllers.yaml"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory("ir_movit_config"),
        "config",
        "moveit.rviz"
    )

    # Get the robot description content from the MoveIt config
    robot_description_content = moveit_config.robot_description

    # --------------------------------------------------------------------------------------
    # Core ROS 2 Nodes
    # --------------------------------------------------------------------------------------

    # 1. Publishes the robot description as a string on the /robot_description topic
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    # 2. Publishes joint states from the robot model
    # This is essential for RViz to show the robot in a consistent state
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # 3. The ros2_control fake hardware interface
    # This node provides the hardware abstraction and runs the controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_yaml],
        output="screen",
        respawn=True,  # Automatically restart if it crashes
    )

    # --------------------------------------------------------------------------------------
    # Controller Spawners
    # --------------------------------------------------------------------------------------

    # Spawners load and activate the controllers defined in ros2_controllers.yaml
    controller_names = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "ir_gripper_controller"
    ]

    spawner_nodes = []
    for controller in controller_names:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    # --------------------------------------------------------------------------------------
    # MoveIt and Visualization Nodes
    # --------------------------------------------------------------------------------------

    # 4. The main MoveIt node for planning and trajectory execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        # Note: In some setups, you might need remapping here if
        # ros2_control publishes to a different topic, but for
        # this basic case, the default is often sufficient.
    )

    # 5. The RViz2 visualization node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # Return all nodes in the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        ros2_control_node,
        *spawner_nodes,
        move_group_node,
        rviz_node,
    ])
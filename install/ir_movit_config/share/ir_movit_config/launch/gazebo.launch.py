import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Get configuration from launch arguments
    ur_type = LaunchConfiguration("ur_type").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    # Set the Gazebo resource path to find meshes and models
    ur_description_share_dir = get_package_share_directory('ir_desription')
    
    # Correctly set the path to the parent of the package folder.
    # This allows Gazebo to find 'ir_desription' as a model.
    parent_dir = os.path.dirname(ur_description_share_dir)

    #set_gz_resource_path = SetEnvironmentVariable(
    #    name='GZ_SIM_RESOURCE_PATH',
    #    value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', parent_dir]
    #)

    # Use xacro to generate the robot description
    robot_description_content = {
        'robot_description': PathJoinSubstitution([
            FindPackageShare('xacro'), 'xacro', description_file,
            'ur_type:=', ur_type,
            'simulation_controllers:=', controllers_file
        ])
    }

    # Start the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, robot_description_content],
    )

    # Launch Gazebo simulation
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": f'-r -v 4 {world_file}'}.items(),
    )

    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content['robot_description'],
            "-name", "ur",
            "-allow_renaming", "true",
            "-topic", "/robot_description"
        ],
    )

    # Bridge ROS 2 and Gazebo topics
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/ur/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        output="screen",
    )

    return [
        #set_gz_resource_path,
        robot_state_publisher_node,
        gz_launch_description,
        gz_spawn_entity,
        gz_sim_bridge,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type", default_value="ur5", description="Type of UR robot."
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ir_movit_config"), "config", "ros2_controllers.yaml"
            ]),
            description="Path to YAML file with controllers configuration."
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ir_desription"), "urdf", "ur_gripper.urdf.xacro"
            ]),
            description="URDF/XACRO description file with the robot."
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ir_base"), "worlds", "ir_arena.sdf"
            ]),
            description="Gazebo world file to load."
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Launch Gazebo with GUI."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time."
        )
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
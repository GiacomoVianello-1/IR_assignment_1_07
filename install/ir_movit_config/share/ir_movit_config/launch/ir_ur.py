import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    arguments = declare_argumentes()

    # The actual value of the argument used in launch_arguments
    world_file         = LaunchConfiguration("world_file")
    ur_type            = LaunchConfiguration("ur_type")
    safety_limits      = LaunchConfiguration("safety_limits")
    description_file   = LaunchConfiguration("description_file")
    controllers_file   = LaunchConfiguration("controllers_file")
    #moveit_launch_file = LaunchConfiguration("moveit_launch_file")

    # Setup the resource for the map
    package_dir = get_package_share_directory('ir_arena')
    model_dir   = os.path.join(package_dir, 'models')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_dir
    )


    return LaunchDescription([

        *arguments,  # Declare the argument first

        set_gz_resource_path,

        # Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ir_arena"), "launch", "movit_ur.py"
                ])
            ),
            launch_arguments={
                "world_file": world_file,
                # Add others if needed
                "ur_type": ur_type,
                "safety_limits": safety_limits,
                "controllers_file": controllers_file,
                "description_file": description_file,
                # "moveit_launch_file": moveit_launch_file
            }.items(),
        ),

        # ADD YOUR NODES HERE!!
        

    ])

def declare_argumentes():
    arguments = []

    # Declare 'world_file' argument
    arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ir_arena"), "worlds", "ir_arena.sdf"]
            ),
            description="Absolute path to world file.",
        )
    )

    arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5",
        )
    )

    arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="false",
            description="Enables the safety limits controller if true.",
        )
    )

    arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur_gripper.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )

    # General arguments
    # arguments.append(
    #     DeclareLaunchArgument(
    #         "controllers_file",
    #         default_value=PathJoinSubstitution(
    #             [FindPackageShare("ir_arena"), "config", "controllers.yaml"]
    #         ),
    #         description="Absolute path to YAML file with the controllers configuration.",
    #     )
    # )
    # arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value=PathJoinSubstitution(
    #             #[FindPackageShare("ir_arena"), "urdf", "tmp.xacro"]
    #             [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
    #         ),
    #         description="URDF/XACRO description file (absolute path) with the robot.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "moveit_launch_file",
    #         default_value=PathJoinSubstitution(
    #             [
    #                 FindPackageShare("ur_moveit_config"),
    #                 "launch",
    #                 "ur_moveit.launch.py",
    #             ]
    #         ),
    #         description="Absolute path for MoveIt launch file, part of a config package with robot SRDF/XACRO files. Usually the argument "
    #         "is not set, it enables use of a custom moveit config.",
    #     )
    # )

   

    return arguments
    

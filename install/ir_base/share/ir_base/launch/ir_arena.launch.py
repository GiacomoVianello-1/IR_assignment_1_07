#!/usr/bin/env python

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#sdf_file = 'ir_arena.sdf'
sdf_file = 'empty_world.sdf'

def generate_launch_description():
    package_dir = get_package_share_directory('ir_base')
    model_dir = os.path.join(package_dir, 'models')
    world_path = os.path.join(package_dir, 'worlds', sdf_file)
    

    return LaunchDescription([
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=model_dir,
            ),
            ExecuteProcess(
                cmd=['gz', 'sim', world_path],
                output='screen',
            ),
        ])
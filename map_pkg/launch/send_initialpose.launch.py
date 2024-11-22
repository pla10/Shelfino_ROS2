#!/usr/bin/env python3
#
# Authors: 
#     Enrico Saccon     enrico.saccon [at] unitn.it

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument, GroupAction
from launch import LaunchDescription
from launch_ros.actions import Node

import launch.logging
import logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    map_env_pkg = get_package_share_directory('map_pkg')

    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    if (not os.path.exists(map_env_params_file_path)):
        raise Exception("[{}] Map config file `{}` does not exist".format(__file__, map_env_params_file_path))

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map_env_params_file',
            default_value=map_env_params_file_path,
            description='Full path to the map_pkg params file to use'
        )
    ]

    
    send_initial_poses_node = GroupAction([
        Node(
            package='map_pkg',
            executable='send_initialpose',
            name='send_initialpose',
            output='screen',
            namespace='shelfino1',
            parameters=[map_env_params_file],
        )
    ])

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))
    ld.add_action(send_initial_poses_node)

    return ld

#!/usr/bin/env python3
#
# Authors: 
#     Enrico Saccon     enrico.saccon [at] unitn.it
#     Placido Falqueto  placido.falqueto [at] unitn.it

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch import LaunchDescription
from launch_ros.actions import Node


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
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    victims_activated   = LaunchConfiguration('victims_activated', default='false')

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        DeclareLaunchArgument(
            'map_env_params_file',
            default_value=map_env_params_file_path,
            description='Full path to the map_pkg params file to use'
        ),
        DeclareLaunchArgument(
            'victims_activated',
            default_value='false',
            description='Activate the victims'
        )
    ]

    # List of nodes to launch
    nodes = GroupAction(
        actions = [
            # Node ( 
            #     package='map_pkg',
            #     executable='orchestrator',
            #     name='orchestrator',
            #     output='screen',
            #     # parameters=[configured_params]
            #     parameters= [map_env_params_file] + [
            #         {
            #             'victims_activated' : victims_activated,
            #             # 'init_names' : init_names,
            #             # 'init_x' : init_x,
            #             # 'init_y' : init_y,
            #             # 'init_yaw' : init_yaw
            #         }
            #     ]
            # ),
            Node (
                package='map_pkg',
                executable='send_gates',
                name='send_gates',
                output='screen',
                parameters=[map_env_params_file]
            ),
            Node (
                package='map_pkg',
                executable='send_borders',
                name='send_borders',
                output='screen',
                parameters=[map_env_params_file]
            ),
            Node (
                package='map_pkg',
                executable='send_victims',
                name='send_victims',
                output='screen',
                parameters=[map_env_params_file]
            ),
            Node (
                package='map_pkg',
                executable='send_obstacles',
                name='send_obstacles',
                output='screen',
                parameters=[map_env_params_file]
            ),
        ],
        # period = PythonExpression(["'", '1.0', "'"])
    )

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)
    
    ld.add_action(nodes)    

    return ld

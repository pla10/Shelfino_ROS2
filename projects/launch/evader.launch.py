#!/usr/bin/env python3
#
# Author: Enrico Saccon  enrico.saccon [at] unitn.it

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


import launch.logging
import logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def check_map(context):
    from pathlib import Path
    map_name = Path(context.launch_configurations['map_file'])
    world_name = Path(context.launch_configurations['gazebo_world_file'])
    if map_name.stem != world_name.stem:
        raise Exception("[{}] Map `{}` does not match world `{}`".format(__file__, map_name.stem, world_name.stem))
    return 

def get_map_name(context):
    map_name = Path(context.launch_configurations['map_file']).stem
    context.launch_configurations['map_name'] = map_name
    return 

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    shelfino_desc_pkg  = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg  = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg  = get_package_share_directory('shelfino_gazebo')
    map_env_pkg        = get_package_share_directory('map_pkg')

    nav2_params_file_path    = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    shelfino_id  = LaunchConfiguration('shelfino_id', default='0')

    # Gazebo simulation arguments
    use_gui           = LaunchConfiguration('use_gui', default='true')
    use_rviz          = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file  = LaunchConfiguration('rviz_config_file', default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon.world'))
    robot_model_file  = LaunchConfiguration('robot_model_file', default=os.path.join(shelfino_desc_pkg, 'models', 'shelfino', 'model.sdf.xacro'))

    # Navigation arguments
    map_file = LaunchConfiguration('map_file', default=os.path.join(shelfino_nav2_pkg, 'maps', 'hexagon.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))

    # Map package arguments
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    shelfino_name = PythonExpression(["'", 'shelfino', shelfino_id, "'"])

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        # General arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'shelfino_id',
            default_value=shelfino_id,
            description='ID of the robot'
        ),

        # Gazebo simulation arguments
        DeclareLaunchArgument(
            'use_gui',
            default_value=use_gui,
            choices=['true', 'false'],
            description='Flag to enable gazebo visualization'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=use_rviz,
            choices=['true', 'false'],
            description='Flag to enable rviz visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            # choices=['empty', 'povo', 'hexagon'],
            description='World used in the gazebo simulation'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value=gazebo_world_file,
            # choices=['empty', 'povo', 'hexagon'],
            description='World used in the gazebo simulation'
        ),
        DeclareLaunchArgument(
            'robot_model_file',
            default_value=robot_model_file,
            description='Model used in the gazebo simulation'
        ),

        # Navigation arguments
        DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=nav2_params_file_path,
            description='Full path to the nav2 params file to use'
        ),
        DeclareLaunchArgument(
            'nav2_rviz_config_file',
            default_value=nav2_rviz_config_file,
            description='Full path to the nav2 rviz config file to use'
        ),

        # Map package arguments
        DeclareLaunchArgument(
            'map_env_params_file',
            default_value=map_env_params_file_path,
            description='Full path to the map_pkg params file to use'
        ),
    ]

    # Rewrite the map file parameter substituting n_victims with 0
    mav_env_conf_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={'n_victims' : '0'},
        convert_types=True
    )

    # List of nodes to launch
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_desc_pkg, 'launch'),
                '/rsp.launch.py']
            ),
            launch_arguments= {
                'use_sim_time': use_sim_time,
                'robot_id': shelfino_id,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_gaze_pkg, 'launch'),
                '/shelfino.launch.py']
            ),
            launch_arguments= {
                'use_sim_time': use_sim_time,
                'robot_id': shelfino_id,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'rviz_config_file': rviz_config_file,
                'gazebo_world_file': gazebo_world_file,
                'robot_model_file': robot_model_file,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_nav2_pkg, 'launch'),
                '/shelfino_nav.launch.py']
            ),
            launch_arguments= {
                'use_sim_time': use_sim_time,
                'robot_id': shelfino_id,
                'map_file' : map_file,
                'nav2_params_file' : nav2_params_file,
                'rviz_config_file': nav2_rviz_config_file,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(map_env_pkg, 'launch'),
                '/map_env.launch.py']
            ),
            launch_arguments= {
                'map_env_params_file': mav_env_conf_params,
            }.items()
        ),
    ]

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=check_map))
    ld.add_action(OpaqueFunction(function=get_map_name))
    ld.add_action(OpaqueFunction(function=print_env))
    
    for node in nodes:
        ld.add_action(node)

    return ld

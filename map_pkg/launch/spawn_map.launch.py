#!/usr/bin/env python3
#
# Authors: 
#     Enrico Saccon     enrico.saccon [at] unitn.it
#     Placido Falqueto  placido.falqueto [at] unitn.it

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch import LaunchDescription, LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
import launch

from launch.events.process import ProcessExited
from launch.event_handlers import OnProcessExit

try:
    from mapscripts import spawn_borders, spawn_gates, spawn_obstacles
except Exception as e:
    import sys
    sys.path.append(os.path.join(get_package_share_directory('map_pkg'), 'launch', 'mapscripts'))
    sys.path.append(os.path.join(get_package_share_directory('map_pkg'), 'launch'))
    from mapscripts import spawn_gates, spawn_borders, spawn_obstacles


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
    gen_map_params_file_path = os.path.join(map_env_pkg, 'config', 'full_config.yaml')

    # General arguments
    gen_map_params_file  = LaunchConfiguration('gen_map_params_file', default=gen_map_params_file_path)

    victims_activated    = LaunchConfiguration('victims_activated', default='false')

    elements_models_path = LaunchConfiguration('elements_models_path', default=PythonExpression(["'", os.path.join(get_package_share_directory('map_pkg'), 'models'), "'"]))
    gazebo_models_path   = LaunchConfiguration('gazebo_models_path', default=PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_gazebo'), 'worlds'), "'"]))

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        DeclareLaunchArgument(
            'gen_map_params_file',
            default_value=gen_map_params_file_path,
            description='FULL path to the final configuration file'
        ),
        DeclareLaunchArgument(
            'victims_activated',
            default_value=victims_activated,
            description='Activate the victims'
        ),
        DeclareLaunchArgument(
            'elements_models_path',
            default_value=elements_models_path,
            description='Path to the models'
        ),
        DeclareLaunchArgument(
            'gazebo_models_path',
            default_value=gazebo_models_path,
            description='Path to the gazebo models'
        )
    ]

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))

    functions = [
        OpaqueFunction(function=spawn_borders.spawn_borders),
        OpaqueFunction(function=spawn_gates.spawn_gates),
        OpaqueFunction(function=spawn_obstacles.spawn_obstacles)
        # OpaqueFunction(function=spawn_victims.spawn_victims)
    ]

    nodes = [
        Node(
            package='map_pkg',
            executable='send_borders',
            name='send_borders',
            output='screen',
            parameters=[gen_map_params_file]
        ),
        Node(
            package='map_pkg',
            executable='send_gates',
            name='send_gates',
            output='screen',
            parameters=[gen_map_params_file]
        ),
        Node(
            package='map_pkg',
            executable='send_obstacles',
            name='send_obstacles',
            output='screen',
            parameters=[gen_map_params_file]
        ),
        Node(
            package='map_pkg',
            executable='send_victims',
            name='send_victims',
            output='screen',
            parameters=[gen_map_params_file],
            condition=IfCondition(victims_activated)
        ),
    ]

    for action in nodes+functions:
        ld.add_action(action)

    return ld
#!/usr/bin/env python3
#
# Author: Enrico Saccon  enrico.saccon [at] unitn.it

import os, sys
from pathlib import Path
import random
from math import cos, pi, sin, sqrt

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription, LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.events.process import ProcessExited
from launch.event_handlers import OnProcessExit

from launch.actions import  GroupAction

import logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def get_map_name(context):
    map_name = Path(context.launch_configurations['map_file']).stem
    context.launch_configurations['map_name'] = map_name
    return 

shelfino_pos_ready = False

def read_shelfino_pose(context):
    import yaml

    shelfino_name = context.launch_configurations['shelfino_name']
    gen_config_file = context.launch_configurations['gen_map_params_file']

    print(f"Reading {gen_config_file} to get the pose of {shelfino_name}")
    
    with open(gen_config_file, 'r') as f:
        map_config = yaml.safe_load(f)

        shelfino_names = map_config['/**']['ros__parameters']['init_names']
        if shelfino_name not in shelfino_names:
            raise Exception(f"Shelfino {shelfino_name} not found in the map configuration")
        
        array_pos = shelfino_names.index(shelfino_name)

        shelfino_pose_rand = map_config["/**"]['ros__parameters']['init_rand'][array_pos]

        if shelfino_pose_rand:
            raise Exception(f"Shelfino {shelfino_name} is set to random pose, but should have been taken care of by the map generator")
        else: 
            shelfino_pose_x = map_config["/**"]['ros__parameters']['init_x'][array_pos]
            shelfino_pose_y = map_config["/**"]['ros__parameters']['init_y'][array_pos]
            shelfino_pose_yaw = map_config["/**"]['ros__parameters']['init_yaw'][array_pos]

        context.launch_configurations['shelfino_init_x']   = str(shelfino_pose_x)
        context.launch_configurations['shelfino_init_y']   = str(shelfino_pose_y)
        context.launch_configurations['shelfino_init_yaw'] = str(shelfino_pose_yaw)


def evaluate_rviz(context):
    """
    This function allows for launching just one Rviz instance for all the robots.
    It takes the rviz config file and creates a new one with the correct items
    multiplied for all the robots. 
    :param context: The context of the launch including the launch config.
    """
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    
    rviz_path = context.launch_configurations['nav2_rviz_config_file']
    shelfino_name = context.launch_configurations['shelfino_name']
    cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', f"{shelfino_name}_nav.rviz")

    with open(rviz_path,'r') as f_in:
        filedata = f_in.read()
        filedata = filedata.replace("shelfinoX", shelfino_name)
    with open (cr_path, 'w') as f_out:
        f_out.write(filedata)
    context.launch_configurations['nav2_rviz_config_file'] = cr_path

    return [Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', cr_path],
        parameters=[
            {'use_sim_time': True if context.launch_configurations['use_sim_time'] == "true" else False}
        ],
        output='screen'
    )]


def get_shelfino_pose(context):
    global shelfino_pos_ready

    import threading
    t = threading.Thread(target=read_shelfino_pose, args=(context,))
    t.start()
    t.join()
    logging.info(f"Shelfino pose ready {context.launch_configurations['shelfino_init_x']} {context.launch_configurations['shelfino_init_y']} {context.launch_configurations['shelfino_init_yaw']}")
    shelfino_pos_ready = True
    return


def nav2_func(context, nav2_node):
    global shelfino_pos_ready
    while(not shelfino_pos_ready):
        pass

    return [nav2_node]

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    shelfino_desc_pkg  = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg  = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg  = get_package_share_directory('shelfino_gazebo')
    map_env_pkg        = get_package_share_directory('map_pkg')

    nav2_params_file_path    = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    gen_map_params_file_path = os.path.join(map_env_pkg, 'config', 'full_config.yaml')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    shelfino_name  = LaunchConfiguration('shelfino_name', default='shelfino')

    # Gazebo simulation arguments
    use_gui           = LaunchConfiguration('use_gui', default='true')
    use_rviz          = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file  = LaunchConfiguration('rviz_config_file', default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'empty.world'))

    # Shelfino related arguments
    spawn_shelfino    = LaunchConfiguration('spawn_shelfino', default='false')
    shelfino_init_x   = LaunchConfiguration('shelfino_init_x', default='0.0')
    shelfino_init_y   = LaunchConfiguration('shelfino_init_y', default='0.0')
    shelfino_init_yaw = LaunchConfiguration('shelfino_init_yaw', default='0.0')

    # Navigation arguments
    map_file = LaunchConfiguration('map_file', default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))

    # Map package arguments
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)
    gen_map_params_file = LaunchConfiguration('gen_map_params_file', default=gen_map_params_file_path)
    victims_activated = LaunchConfiguration('victims_activated', default='true')
    generate_new_map_config = LaunchConfiguration('generate_new_map_config', default='true')

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        # General arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'shelfino_name',
            default_value=shelfino_name,
            description='Shelfino\'s name'
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
            description='Configuration file for rviz'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value=gazebo_world_file,
            # choices=['empty', 'povo', 'hexagon'],
            description='World used in the gazebo simulation'
        ),

        # Shelfino related arguments
        DeclareLaunchArgument(
            'spawn_shelfino',
            default_value=spawn_shelfino,
            choices=['true', 'false'],
            description='Whether to spawn the robot or not'
        ),
        DeclareLaunchArgument(
            'shelfino_init_x',
            default_value=shelfino_init_x,
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'shelfino_init_y',
            default_value=shelfino_init_y,
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'shelfino_init_yaw',
            default_value=shelfino_init_yaw,
            description='Initial yaw of the robot'
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
            description='Full path to the map_pkg template file to use for params'
        ),
        DeclareLaunchArgument(
            'gen_map_params_file',
            default_value=gen_map_params_file_path,
            description='Full path to the final params file file to use'
        ),
        DeclareLaunchArgument(
            'victims_activated',
            default_value=victims_activated,
            choices=['true'],
            description='Whether to activate the victims or not'
        ),
        DeclareLaunchArgument(
            'generate_new_map_config',
            default_value=generate_new_map_config,
            choices=['true', 'false'],
            description='Whether to regenerate the gen_map_params_file or not'
        ),
    ]

    # List of nodes to launch

    # This is the first node that must be called as it generates the configuration file
    gen_config_node = Node ( 
        package='map_pkg',
        executable='generate_config_file.py',
        name='generate_config_file',
        output='screen',
        parameters= [map_env_params_file] + [
            {
                'generate_new_config' : generate_new_map_config,
                'map_env_params_file' : map_env_params_file,
                'gen_map_params_file' : gen_map_params_file,
                'victims_activated' : victims_activated,
            }
        ],
    )
    
    # This node is used to create the map in PGM format, which is then used by the map server
    create_map_node = Node (
        package='map_pkg',
        executable='create_map_pgm.py',
        name='create_map_pgm',
        output='screen',
        parameters=[map_env_params_file]
    )

    # This node spawns the objects in Gazebo and publishes the data to the topics
    spawn_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(map_env_pkg, 'launch', 'spawn_map.launch.py')]
        ),
        launch_arguments= {
            'victims_activated': victims_activated,
            'gen_map_params_file': gen_map_params_file,
        }.items()
    )

    # This group of nodes is used to spawn the robot in Gazebo and to publish 
    # the description of the robot
    sim_nodes = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_gaze_pkg, 'launch', 'shelfino.launch.py')
            ]),
            launch_arguments= {
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'gazebo_world_file': gazebo_world_file,
                'spawn_shelfino' : spawn_shelfino,
                'shelfino_name': shelfino_name,
                'rviz_config_file': rviz_config_file,
                'shelfino_init_x' : shelfino_init_x,
                'shelfino_init_y' : shelfino_init_y,
                'shelfino_init_yaw' : shelfino_init_yaw,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_desc_pkg, 'launch'),
                '/rsp.launch.py']
            ),
            launch_arguments= {
                'use_sim_time': use_sim_time,
                'shelfino_name': shelfino_name,
            }.items()
        )
    ])
    
    # This node is used to launch the navigation stack
    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_nav2_pkg, 'launch', 'shelfino_nav.launch.py')]
        ),
        launch_arguments= {
            'use_sim_time': use_sim_time,
            'robot_name': shelfino_name,
            'map_file' : map_file,
            'nav2_params_file' : nav2_params_file,
            # 'rviz_config_file': nav2_rviz_config_file,
            'set_initial_pose' : 'true',
            'initial_x' : shelfino_init_x,
            'initial_y' : shelfino_init_y,
            'initial_yaw' : shelfino_init_yaw,
        }.items()
    )

    kill_shelfino_node = Node( 
        package='shelfino_gazebo',
        executable='destroy_shelfino',
        name='destroy_shelfino',
        output='screen',
        namespace=shelfino_name
    )

    # # Event-handlers

    nav2_opaque_func = OpaqueFunction(function=nav2_func, args=[nav2_launch_file])

    nodes = [
        OpaqueFunction(function=get_shelfino_pose), 
        create_map_node, 
        spawn_map_launch, 
        sim_nodes, 
        kill_shelfino_node,
    ]

    def launch_nodes(event : ProcessExited, context : LaunchContext):
        return nodes 

    gen_config_eh = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=gen_config_node,
            on_exit=launch_nodes
        )
    )

    def launch_nav2(event : ProcessExited, context : LaunchContext):
        return [nav2_opaque_func] 

    map_spawn_eh = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=create_map_node,
            on_exit=launch_nav2
        )
    )

    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))
    ld.add_action(OpaqueFunction(function=get_map_name))

    ld.add_action(gen_config_node)
    ld.add_action(gen_config_eh)
    ld.add_action(map_spawn_eh)

    ld.add_action(OpaqueFunction(function=evaluate_rviz))

    return ld

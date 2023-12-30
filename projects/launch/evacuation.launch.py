#!/usr/bin/env python3
#
# Author: Enrico Saccon  enrico.saccon [at] unitn.it

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
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
    # world_name = Path(context.launch_configurations['gazebo_world_file'])
    # if map_name.stem != world_name.stem:
    #     raise Exception("[{}] Map `{}` does not match world `{}`".format(__file__, map_name.stem, world_name.stem))
    return 

def get_map_name(context):
    map_name = Path(context.launch_configurations['map_file']).stem
    context.launch_configurations['map_name'] = map_name
    return 

def evaluate_rviz(context, *args, **kwargs):
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    
    rviz_path = context.launch_configurations['nav2_rviz_config_file']
    cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino') + \
        'evacuation_'+context.launch_configurations['n_shelfini']+'_nav.rviz'
    
    output_config = {}
    with open(rviz_path, 'r') as f_in:
        rviz_config= yaml.load(f_in, Loader=yaml.FullLoader)
        for key in rviz_config.keys():
            if key != 'Visualization Manager':
                output_config[key] = rviz_config[key]

        # Add everything that is not displays or tools
        output_config['Visualization Manager'] = {}
        for key in rviz_config['Visualization Manager'].keys():
            if key != 'Displays' and key != 'Tools':
                output_config['Visualization Manager'][key] = rviz_config['Visualization Manager'][key]

        # Configure displays for Rviz
        displays = rviz_config['Visualization Manager']['Displays']
        output_config['Visualization Manager']['Displays'] = []
        for display in displays:
            if type(display) is not dict:
                raise Exception("[{}] Display `{}` is not a dictionary".format(__file__, display))
            if "shelfinoX" in str(display):
                display_str = str(display)
                for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
                    output_config['Visualization Manager']['Displays'].append(
                        yaml.load(display_str.replace("shelfinoX", "shelfino"+str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Displays'].append(display)

        # Configure tools for Rviz
        tools = rviz_config['Visualization Manager']['Tools']
        output_config['Visualization Manager']['Tools'] = []
        for tool in tools:
            if type(tool) is not dict:
                raise Exception("[{}] Tool `{}` is not a dictionary".format(__file__, tool))
            if "shelfinoX" in str(tool):
                tool_str = str(tool)
                for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
                    output_config['Visualization Manager']['Tools'].append(
                        yaml.load(tool_str.replace("shelfinoX", "shelfino"+str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Tools'].append(tool)

        with open(cr_path, 'w+') as f_out:
            yaml.dump(output_config, f_out, default_flow_style=False)

    context.launch_configurations['rviz_config_file'] = cr_path
    
    return
    

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    shelfino_desc_pkg  = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg  = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg  = get_package_share_directory('shelfino_gazebo')
    map_env_pkg        = get_package_share_directory('map_pkg')

    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')

    # General arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_shelfini   = LaunchConfiguration('n_shelfini', default='3')

    # Gazebo simulation arguments
    use_gui           = LaunchConfiguration('use_gui', default='true')
    use_rviz          = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file  = LaunchConfiguration('rviz_config_file', default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'empty.world'))

    # Navigation arguments
    map_file              = LaunchConfiguration('map_file', default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_params_file      = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))

    # Map package arguments
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        # General arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'n_shelfini',
            default_value=n_shelfini,
            description='The number of Shelfini to spawn'
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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_gaze_pkg, 'launch'),
            '/shelfino.launch.py']
        ),
        launch_arguments= {
            'use_sim_time': use_sim_time,
            'spawn_shlefino': 'false',
            'use_gui': use_gui,
            'use_rviz': use_rviz,
            'gazebo_world_file': gazebo_world_file
        }.items()
    )

    # Rewrite the map file parameter substituting n_victims with 0
    map_env_conf_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={'n_victims' : '0'},
        convert_types=True
    )

    map_pkg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(map_env_pkg, 'launch'),
            '/map_env.launch.py']
        ),
        launch_arguments= {
            'map_env_params_file': map_env_conf_params,
        }.items()
    )

    rviz2_node = Node (
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    def launch_rsp(context):
        nodes = []
        for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
            
            shelfino_name = PythonExpression(["'", "shelfino", str(shelfino_id), "'"])
    
            rsp_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(shelfino_desc_pkg, 'launch'),
                    '/rsp.launch.py']
                ),
                launch_arguments= {
                    'use_sim_time': use_sim_time,
                    'shelfino_id': str(shelfino_id),
                }.items()
            )
            nodes.append(rsp_node)

        return nodes
    
    def spawn_shelfini(context):
        nodes = []
        for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
            
            shelfino_name = PythonExpression(["'", "shelfino", str(shelfino_id), "'"])
    
            spawn_node = GroupAction([
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                            '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                            '-entity', shelfino_name,
                            '-robot_namespace', shelfino_name,
                            '-x', str(shelfino_id*2.0-2.0),
                            '-y', str(shelfino_id*2.0-2.0),
                            '-z', '0.0',
                            '-Y', '0.0',
                    ]
                ),
                Node(
                    package='shelfino_gazebo',
                    executable='destroy_shelfino',
                    name='destroy_shelfino',
                    output='screen',
                    namespace=shelfino_name,
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ])
            nodes.append(spawn_node)

        return nodes

    def launch_navigation(context):
        nodes = []
        for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
            
            shelfino_name = PythonExpression(["'", "shelfino", str(shelfino_id), "'"])
    
            model_output = PythonExpression(["'", os.path.join(shelfino_desc_pkg, 'models', 'shelfino'), str(shelfino_id), ".sdf'"])
            sapf_node = GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(shelfino_nav2_pkg, 'launch'),
                        '/shelfino_nav.launch.py']
                    ),
                    launch_arguments= {
                        'use_sim_time': use_sim_time,
                        'robot_id': str(shelfino_id),
                        'map_file' : map_file,
                        'nav2_params_file' : nav2_params_file,
                        'rviz_config_file': nav2_rviz_config_file,
                        'initial_x': str(shelfino_id*2.0-2.0),
                        'initial_y': str(shelfino_id*2.0-2.0),
                        'headless' : 'true',
                    }.items()
                ),
            ])

            nodes.append(sapf_node)

        return nodes


    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=get_map_name))
    ld.add_action(OpaqueFunction(function=print_env))
    ld.add_action(OpaqueFunction(function=evaluate_rviz))
    
    create_map_node = Node (
        package='map_pkg',
        executable='create_map_pgm.py',
        name='create_map_pgm',
        output='screen',
        parameters=[map_env_conf_params]
    )

    ld.add_action(gazebo_launch)
    ld.add_action(map_pkg_node)
    
    ld.add_action(TimerAction(
        period='2',
        actions = [create_map_node]
    ))
    
    ld.add_action(TimerAction(
        period='5',
        actions = [OpaqueFunction(function=launch_rsp)]
    ))
    ld.add_action(TimerAction(
        period='10',
        actions = [OpaqueFunction(function=spawn_shelfini)]
    ))

    ld.add_action(TimerAction(
        period='15',
        actions = [rviz2_node]
    ))

    ld.add_action(TimerAction(
        period='20',
        actions = [OpaqueFunction(function=launch_navigation)]
    ))



    # ld.add_action(map_pkg_node)
    # nodes = OpaqueFunction(function=populate)
    # ld.add_action(nodes)
    # ld.add_action(rviz2_node)

    return ld

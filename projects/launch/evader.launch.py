#!/usr/bin/env python3
#
# Author: Enrico Saccon  enrico.saccon [at] unitn.it

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription, LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch.events.process import ProcessExited
from launch.event_handlers import OnProcessExit

def print_env(context):
    """
    Prints the environment variables.
    :param context: The context of the launch including the launch config.
    """
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return


def start_shelfini(context, shelfino_desc_pkg, shelfino_nav2_pkg):
    import yaml, re

    nodes = []
    shelfini_names = []

    with open (context.launch_configurations['gen_map_params_file'], 'r') as f:
        shelfino_config_yaml = yaml.load(f, Loader=yaml.FullLoader)
        shelfino_config_yaml = shelfino_config_yaml["/**"]["ros__parameters"]

        # Check that evader and shelfino are present
        if "evader" not in shelfino_config_yaml['init_names']:
            raise Exception("Evader is not present in the map config file")
        if "pursuer" not in shelfino_config_yaml['init_names']:
            raise Exception(f"Shelfino pursuer is not present in the map config file")
         
        # Get indexes of evader and shelfino
        evader_index = shelfino_config_yaml['init_names'].index("evader")
        shelfino_index = shelfino_config_yaml['init_names'].index("pursuer")

        print(f"Indexes {shelfino_index}")

        indexes = [evader_index, shelfino_index]

        for index in indexes:
            if shelfino_config_yaml['init_rand'][index]:
                raise Exception(f"{shelfino_config_yaml['init_names'][index]} is set to random pose, but should have been taken care of by the map generator")

            shelfino_name = shelfino_config_yaml['init_names'][index]
            shelfini_names.append(shelfino_name)
            shelfino_pose_x = shelfino_config_yaml['init_x'][index]
            shelfino_pose_y = shelfino_config_yaml['init_y'][index]
            shelfino_pose_yaw = shelfino_config_yaml['init_yaw'][index]

            print(f"Spawning {shelfino_name} at ({shelfino_pose_x}, {shelfino_pose_y}, {shelfino_pose_yaw})")

            rsp_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(shelfino_desc_pkg, 'launch', 'rsp.launch.py')
                ]),
                launch_arguments= {
                    'use_sim_time': context.launch_configurations['use_sim_time'],
                    'shelfino_name': shelfino_name,
                }.items()
            )

            nav2_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(shelfino_nav2_pkg, 'launch', 'shelfino_nav.launch.py')]
                ),
                launch_arguments= {
                    'use_sim_time': context.launch_configurations['use_sim_time'],
                    'robot_name': shelfino_name,
                    'map_file' : context.launch_configurations['map_file'],
                    'nav2_params_file' : context.launch_configurations['nav2_params_file'],
                    'rviz_config_file': context.launch_configurations['nav2_rviz_config_file'],
                    'initial_x': str(shelfino_pose_x),
                    'initial_y': str(shelfino_pose_y),
                    'initial_yaw': str(shelfino_pose_yaw),
                    'set_initial_pose': 'true',
                    'headless' : 'true',
                }.items()
            )

            print(f"[{index}] Spawning {shelfino_name} with at ({shelfino_pose_x}, {shelfino_pose_y}, {shelfino_pose_yaw})")

            spawn_shelfino_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                        '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                        '-entity', shelfino_name,
                        '-robot_namespace', shelfino_name,
                        '-x', PythonExpression(["'", str(shelfino_pose_x), "'"]),
                        '-y', PythonExpression(["'", str(shelfino_pose_y), "'"]),
                        '-z', '0.0',
                        '-Y', PythonExpression(["'", str(shelfino_pose_yaw), "'"]),
                ]
            )

            destroy_shelfino_node = Node(
                package='shelfino_gazebo',
                executable='destroy_shelfino',
                name='destroy_shelfino',
                output='screen',
                namespace=shelfino_name
            )

            nodes += [
                rsp_launch_file, 
                spawn_shelfino_node,
                nav2_launch_file, 
                destroy_shelfino_node
            ]

    return nodes + [OpaqueFunction(function=evaluate_rviz, args=[shelfini_names])]

def evaluate_rviz(context, shelfini_names):
    """
    This function allows for launching just one Rviz instance for all the robots.
    It takes the rviz config file and creates a new one with the correct items
    multiplied for all the robots. 
    :param context: The context of the launch including the launch config.
    """
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    
    rviz_path = context.launch_configurations['nav2_rviz_config_file']
    cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', f"evader_{len(shelfini_names)}_nav.rviz")
    
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
                for shelfino_name in shelfini_names:
                    output_config['Visualization Manager']['Displays'].append(
                        yaml.load(display_str.replace("shelfinoX", shelfino_name), Loader=yaml.FullLoader))
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
                for shelfino_name in shelfini_names:
                    output_config['Visualization Manager']['Tools'].append(
                        yaml.load(tool_str.replace("shelfinoX", shelfino_name), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Tools'].append(tool)
        
        print(output_config)
        print("Writing to", cr_path)

        with open(cr_path, 'w+') as f_out:
            yaml.dump(output_config, f_out, default_flow_style=False)

    context.launch_configurations['rviz_config_file'] = cr_path

    nodes = [ Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', context.launch_configurations['rviz_config_file']],
        parameters=[
            {'use_sim_time': True if context.launch_configurations['use_sim_time'] == 'true' else False}
        ],
    )]
    
    return nodes
    

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    shelfino_desc_pkg  = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg  = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg  = get_package_share_directory('shelfino_gazebo')
    map_env_pkg        = get_package_share_directory('map_pkg')

    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    gen_map_params_file_path = os.path.join(map_env_pkg, 'config', 'full_config.yaml')

    # Evader arguments
    difficulty = LaunchConfiguration('difficulty', default=1)

    # General arguments
    use_sim_time  = LaunchConfiguration('use_sim_time', default='true')

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
    gen_map_params_file = LaunchConfiguration('gen_map_params_file', default=gen_map_params_file_path)
    victims_activated = LaunchConfiguration('victims_activated', default='true')
    generate_new_map_config = LaunchConfiguration('generate_new_map_config', default='true')
    

    # Declare LaunchArguments for exposing launching arguments
    launch_args = [
        # Evader arguments
        DeclareLaunchArgument(
            'difficulty',
            default_value=difficulty,
            description='The difficulty of the evader'
        ),

        # General arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
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

    # Launch Gazebo but do not spawn the robots
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_gaze_pkg, 'launch', 'shelfino.launch.py')]
        ),
        launch_arguments= {
            # 'use_sim_time': use_sim_time,
            'use_gui': use_gui,
            'use_rviz': use_rviz,
            'gazebo_world_file': gazebo_world_file,
            'spawn_shelfino': 'false',
        }.items()
    )

    # Rewrite the map file parameter substituting n_victims with 0
    gen_map_params_file_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={'send_victims' : {'victims_activated' : 'false'}},
        convert_types=True
    )

    # This node creates the map in pgm format
    create_map_node = Node (
        package='map_pkg',
        executable='create_map_pgm.py',
        name='create_map_pgm',
        output='screen',
        parameters=[gen_map_params_file_params]
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
    

    # Event handlers
    # The nodes to launch after the configuration node has finished
    after_config_nodes = [
        create_map_node,
        spawn_map_launch
    ]

    def launch_after_config_nodes(event : ProcessExited, context : LaunchContext):
        return after_config_nodes 

    gen_config_eh = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=gen_config_node,
            on_exit=launch_after_config_nodes
        )
    )

    # Spawn shelini and start the navigation
    def launch_after_spawn_map_nodes(event : ProcessExited, context : LaunchContext):
        return OpaqueFunction(function=start_shelfini, args=[shelfino_desc_pkg, shelfino_nav2_pkg])
    
    spawn_map_eh = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=create_map_node,
            on_exit=launch_after_spawn_map_nodes
        )
    )

    # LaunchDescription 
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))

    ld.add_action(gazebo_launch)

    ld.add_action(gen_config_node)

    ld.add_action(gen_config_eh)

    ld.add_action(spawn_map_eh)

    return ld

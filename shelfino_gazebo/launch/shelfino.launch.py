#!/usr/bin/env python3
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import logging, launch.logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def check_exists(context):
    if not os.path.exists(context.launch_configurations['gazebo_world_file']):
        raise Exception("[{}] Gazebo world file `{}` does not exist".format(__file__, context.launch_configurations['gazebo_world_file']))
        
    if context.launch_configurations['use_rviz']=="true" and \
        not os.path.exists(context.launch_configurations['rviz_config_file']):
        raise Exception("[{}] Rviz configuration `{}` does not exist".format(__file__, context.launch_configurations['use_rviz']))

    return

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    gazebo_ros_pkg    = get_package_share_directory('gazebo_ros')
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')

    use_sim_time      = LaunchConfiguration('use_sim_time', default='true')
    shelfino_id       = LaunchConfiguration('shelfino_id', default='G')
    use_gui           = LaunchConfiguration('use_gui')
    use_rviz          = LaunchConfiguration('use_rviz')
    spawn_shelfino    = LaunchConfiguration('spawn_shelfino', default='false')
    rviz_config_file  = LaunchConfiguration('rviz_config_file', 
        default=PythonExpression(["'", os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino'), shelfino_id, ".rviz'"]) 
    )
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', 
        default=os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon.world')
    )
    
    shelfino_name = PythonExpression(["'", 'shelfino', shelfino_id, "'"])
    model_output = PythonExpression(["'", os.path.join(shelfino_desc_pkg, 'models', 'shelfino'), shelfino_id, ".sdf'"])
    
    # TODO move models to shelfino_gazebo
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(shelfino_desc_pkg, 'models')

    # rviz_config = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino'), shelfino_id, '.rviz', "'"])

    def evaluate_rviz(context, *args, **kwargs):
        """
        Replace shelfinoX with the robot name in the rviz configuration file and 
        sets the rviz_config_file to the new file
        """
        if context.launch_configurations['use_rviz'] == 'true':
            rn = 'shelfino' + context.launch_configurations['shelfino_id']
            rviz_path = context.launch_configurations['rviz_config_file']
            cr_path = os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino') + context.launch_configurations['shelfino_id'] + '.rviz'
            
            print("[{}] Replacing shelfinoX with {} from file {} to file {}".format(__file__, rn, rviz_path, cr_path))

            with open(rviz_path,'r') as f_in:
                filedata = f_in.read()
                newdata = filedata.replace("shelfinoX", rn)
                with open(cr_path,'w+') as f_out:
                    f_out.write(newdata)

            context.launch_configurations['rviz_config_file'] = cr_path

        return

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value=use_sim_time,
            description='Flag to enable use of simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            name='shelfino_id',
            default_value=shelfino_id,
            description='ID of the robot'
        ),
        DeclareLaunchArgument(
            name='use_gui',
            default_value=use_gui, 
            choices=['true', 'false'],
            description='Flag to enable gazebo visualization'
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value=use_rviz, 
            choices=['true', 'false'],
            description='Flag to enable rviz visualization'
        ),
        DeclareLaunchArgument(
            name='spawn_shelfino',
            default_value=spawn_shelfino,
            choices=['true', 'false'],
            description='Flag to enable spawning of the robot'
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=rviz_config_file,
            description='Path to the rviz configuration file, used only if use_rviz=true'
        ),
        DeclareLaunchArgument(
            name='gazebo_world_file',
            default_value=gazebo_world_file, # choices=['empty', 'povo', 'hexagon'],
            description='World used in the gazebo simulation'
        ),

        OpaqueFunction(function=print_env),
        OpaqueFunction(function=check_exists),
        OpaqueFunction(function=evaluate_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': gazebo_world_file,
                'verbose': 'true'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={
                'verbose': 'true'
            }.items(), 
            condition=IfCondition(use_gui)
        ),
    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                        '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                        '-entity', shelfino_name,
                        '-robot_namespace', shelfino_name],
            condition=IfCondition(spawn_shelfino),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])

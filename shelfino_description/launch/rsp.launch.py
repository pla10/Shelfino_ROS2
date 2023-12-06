#!/usr/bin/env python3
#
# Author: Placido Falqueto placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

#TODO change robot_id to shelfino_id

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_id = LaunchConfiguration('robot_id', default='')

    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])

    xacro_model = os.path.join(
        get_package_share_directory('shelfino_description'),
        'models','shelfino','model.urdf.xacro')
    
    robot_desc = Command(['xacro ', xacro_model, ' robot_id:=', robot_id])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Shelfino ID'),

        OpaqueFunction(function=print_env),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'robot_description': robot_desc}],
        )
    ])

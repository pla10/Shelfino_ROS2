#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(get_package_share_directory('shelfino_navigation'), 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='shelfino.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory(
        'shelfino_navigation'), 'rviz', 'shelfino2_slam.rviz')

    sim = LaunchConfiguration('sim', default='false')
    robot_id = LaunchConfiguration('robot_id', default='shelfino2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        
        DeclareLaunchArgument(name='sim', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            namespace=robot_id,
            parameters=[{'use_sim_time': sim}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            namespace=robot_id,            
            parameters=[{'use_sim_time': sim}],
            arguments=['-resolution', resolution,
                       '-publish_period_sec', publish_period_sec]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': sim}],
            output='screen'
        ),
    ])
#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gazebo_map   = LaunchConfiguration('gazebo_map', default='lab1')
    nav          = LaunchConfiguration('nav', default='true')
    headless     = LaunchConfiguration('headless', default='false')
    shelfino_id     = LaunchConfiguration('shelfino_id', default='404')

    # Name of the Shelfino
    shelfino_name   = PythonExpression(["'", 'shelfino', shelfino_id, "'"])
    # Path to the directory of the shelfino_description package
    shelfino_descr_pkg = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    sllidar_pkg = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')
    realsense_pkg = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    # Path to the directory of the shelfino_navigation package
    shelfino_nav2_pkg = os.path.join(
        get_package_share_directory('shelfino_navigation'), 'launch')

    odom_frame_name = PythonExpression(["'", shelfino_name, "/odom", "'"])
    base_link_name = PythonExpression(["'", shelfino_name, "/base_link", "'"])

    def launch_nav(context, *args, **kwargs):
        """
        Launch the navigation stack if the nav argument is true
        """
        nav_flag = LaunchConfiguration('nav').perform(context).capitalize() == "True"

        nav_instance_cmd = []
        if nav_flag:
            # print(f"Launching nav2 with\
            #     f"use_sim_time  : {context.LaunchConfiguration['use_sim_time']},"\
            #     f"remote_nav    : {'false'},"\
            #     f"headless      : {context.LaunchConfiguration['headless']},"\
            #     f"shelfino_id   : {context.LaunchConfiguration['shelfino_id']},"\
            #     f"shelfino_name : {context.LaunchConfiguration['shelfino_name']}"\
            # ")
            nav_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(shelfino_nav2_pkg, 'shelfino_nav.launch.py')),
                launch_arguments={
                    'use_sim_time'  : context.launch_configurations['use_sim_time'],
                    'remote_nav'    : 'false',
                    'headless'      : context.launch_configurations['headless'],
                    'shelfino_id'   : context.launch_configurations['shelfino_id'],
                    'shelfino_name' : shelfino_name
                }.items(),
            )
            nav_instance_cmd.append(nav_launch)

        return nav_instance_cmd

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),
        DeclareLaunchArgument(name='gazebo_map', default_value='lab1', choices=['lab1', 'povo', 'hexagon'],
                        description='World used in the gazebo simulation'),
        DeclareLaunchArgument(name='nav', default_value='true', choices=['true', 'false'],
                        description='Flag to start the navigation stack'),
        DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),
        DeclareLaunchArgument(name='shelfino_id', default_value='G',
                        description='ID of the robot'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([sllidar_pkg, '/sllidar_a2m8_launch.py']),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'frame_id': PythonExpression(["'", shelfino_name, "/base_laser", "'"])
        #     }.items()
        # ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            namespace=shelfino_name,
            parameters=[{
                'use_sim_time'      : use_sim_time,
                'channel_type'      : 'serial',
                'serial_port'       : '/dev/ttyUSB0', 
                'serial_baudrate'   : 115200, 
                'frame_id'          : PythonExpression(["'", shelfino_name, "/base_laser", "'"]),
                'inverted'          : False, 
                'angle_compensate'  : True
            }],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_pkg, '/rs_launch.py']),
            launch_arguments={
                'use_sim_time' : use_sim_time,
                'tf_ns' : shelfino_name,
                'odom_tf' : odom_frame_name,
                'node_namespace' : shelfino_name,
            }.items()
        ),
        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', odom_frame_name, base_link_name]
        ),

        Node(
            package='shelfino_node',
            namespace=shelfino_name,
            parameters=[
                {'shelfino_id': shelfino_id}
            ],
            output='screen',
            executable='shelfino_node'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([shelfino_descr_pkg, '/rsp.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'shelfino_id': shelfino_id
            }.items()
        ),

        OpaqueFunction(function=launch_nav),
    ])

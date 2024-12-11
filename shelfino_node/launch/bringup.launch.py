#!/usr/bin/env python3
#
# Authors: Placido Falqueto, Enrico Saccon

import os
import math 

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
    use_nav2     = LaunchConfiguration('use_nav2', default='true')
    headless     = LaunchConfiguration('headless', default='false')
    shelfino_id  = LaunchConfiguration('shelfino_id', default='404')

    shelfino_name = PythonExpression(["'shelfino", shelfino_id, "'"])
    # Path to the directory of the shelfino_description package    
    shelfino_descr_pkg = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    sllidar_pkg = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')
    realsense_pkg = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    # Path to the directory of the shelfino_navigation package
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')

    odom_frame_name = PythonExpression(["'", shelfino_name, "/odom'"])
    t265_frame_name = PythonExpression(["'", shelfino_name, "/t265/pose_frame'"])
    base_link_name = PythonExpression(["'", shelfino_name, "/base_footprint'"])

    def launch_nav(context, *args, **kwargs):
        """
        Launch the navigation stack if the nav argument is true
        """
        nav_flag = LaunchConfiguration('use_nav2').perform(context).capitalize() == "True"

        nav_instance_cmd = []
        if nav_flag:
            nav_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(shelfino_nav2_pkg, 'launch', 'shelfino_nav.launch.py')),
                launch_arguments={
                    'use_sim_time' : context.launch_configurations['use_sim_time'],
                    'remote_nav'   : 'false',
                    'headless'     : context.launch_configurations['headless'],
                    'robot_name'   : shelfino_name,
                    'map_file'     : os.path.join(shelfino_nav2_pkg, 'maps', 'lab1.yaml')
                }.items(),
            )
            nav_instance_cmd.append(nav_launch)

        return nav_instance_cmd

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value=use_sim_time, 
            choices=['true', 'false'],
            description='Flag to toggle between real robot and simulation'
        ),
        DeclareLaunchArgument(
            name='use_nav2', 
            default_value=use_nav2, 
            choices=['true', 'false'],
            description='Flag to start Nav2'
        ),
        DeclareLaunchArgument(
            name='headless', 
            default_value='false', 
            choices=['true', 'false'],
            description='Flag to toggle between navigation stack running on robot or locally'
        ),
        DeclareLaunchArgument(
            name='shelfino_name', 
            default_value=shelfino_name,
            description='Name of the robot'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            namespace=shelfino_name,
            parameters=[{
                'use_sim_time'      : use_sim_time,
                'channel_type'      : 'serial',
                'serial_port'       : '/dev/ttyUSB0', 
                'serial_baudrate'   : 256000, # 115200, 
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
                'camera_name' : 't265',
            }.items()
        ),

        Node(
            package='shelfino_node',
            namespace=shelfino_name,
            executable='shelfino_node',
            parameters=[
                {'shelfino_id': shelfino_id}
            ],
            output='screen'
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=f"--x -0.25 --y 0 --z -0.37 --roll {0} --pitch {0} --yaw {0} --frame-id".split(" ")+[t265_frame_name, '--child-frame-id', base_link_name]
        #     # arguments=['-0.25', '0', '-0.37', str(math.pi), str(math.pi), str(math.pi), t265_frame_name, base_link_name]
        #     # arguments=['-0.25', '-0.25', '-0.37', str(math.pi), '0', '0', t265_frame_name, base_link_name]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '-0.37', '0', '0', '0', odom_frame_name, PythonExpression(["'shelfino1/t265//base_camera_link'"])]
        #     # arguments=['-0.25', '-0.25', '-0.37', str(math.pi), '0', '0', t265_frame_name, base_link_name]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', t265_frame_name, base_link_name]
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([shelfino_descr_pkg, '/rsp.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'shelfino_name': shelfino_name
            }.items()
        ),

        # OpaqueFunction(function=launch_nav),
    ])

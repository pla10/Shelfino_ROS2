# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch_ros.actions import Node

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def check_exists(context):
    if not os.path.exists(context.launch_configurations['map_file']):
        raise Exception("[{}] Map file `{}` does not exist".format(__file__, context.launch_configurations['map_file']))
    
    if not os.path.exists(context.launch_configurations['nav2_params_file']):
        raise Exception("[{}] Nav2 parameters file `{}` does not exist".format(__file__, context.launch_configurations['nav2_params_file']))
    
    if context.launch_configurations['headless'] == "false" and \
        not os.path.exists(context.launch_configurations['rviz_config_file']):
        raise Exception("[{}] Rviz configuration `{}` does not exist".format(__file__, context.launch_configurations['rviz_config_file']))

    return

def generate_launch_description():
    shelfino_nav2_pkg = os.path.join(get_package_share_directory('shelfino_navigation'))

    use_sim_time     = LaunchConfiguration('use_sim_time', default='false')
    robot_id         = LaunchConfiguration('robot_id', default='G')
    map_file         = LaunchConfiguration('map_file', default=os.path.join(shelfino_nav2_pkg, 'map', 'hexagon.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=os.path.join(shelfino_nav2_pkg,'config', 'shelfino.yaml'))
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    nav2_autostart   = LaunchConfiguration('nav2_autostart', default='true')

    remote_nav       = LaunchConfiguration('remote_nav', default='false')
    headless         = LaunchConfiguration('headless', default='false')

    initial_x        = LaunchConfiguration('initial_x', default='0.0')
    initial_y        = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw      = LaunchConfiguration('initial_yaw', default='0.0')
    
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])
    
    # Remap lifecycle nodes to robot namespace
    map_node               = PythonExpression(["'/", robot_name, '/map_server', "'"])
    amcl_node              = PythonExpression(["'/", robot_name, '/amcl', "'"])
    bt_navigator_node      = PythonExpression(["'/", robot_name, '/bt_navigator', "'"])
    controller_node        = PythonExpression(["'/", robot_name, '/controller_server', "'"])
    planner_node           = PythonExpression(["'/", robot_name, '/planner_server', "'"])
    waypoint_follower_node = PythonExpression(["'/", robot_name, '/waypoint_follower', "'"])
    behavior_node          = PythonExpression(["'/", robot_name, '/behavior_server', "'"])
    smoother_node          = PythonExpression(["'/", robot_name, '/smoother_server', "'"])
    velocity_smoother_node = PythonExpression(["'/", robot_name, '/velocity_smoother', "'"])

    lifecycle_nodes_loc = [
        [map_node],
        [amcl_node]
    ]

    lifecycle_nodes_nav = [
        [bt_navigator_node], 
        [controller_node], 
        [planner_node], 
        [waypoint_follower_node], 
        [behavior_node], 
        [smoother_node], 
        [velocity_smoother_node]
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = { 
        'use_sim_time'     : use_sim_time,
        'base_frame_id'    : PythonExpression(["'", robot_name, '/base_link', "'"]),
        'odom_frame_id'    : PythonExpression(["'", robot_name, '/odom', "'"]),
        'robot_base_frame' : PythonExpression(["'", robot_name, '/base_link', "'"]),
        'global_frame'     : PythonExpression(["'", robot_name, '/odom', "'"]),
        'topic'            : PythonExpression(["'/", robot_name, '/scan', "'"]),
        'x'                : initial_x,
        'y'                : initial_y,
        'yaw'              : initial_yaw,
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    def evaluate_rviz(context, *args, **kwargs):
        rn = 'shelfino' + LaunchConfiguration('robot_id').perform(context)
        rviz_path = context.launch_configurations['rviz_config_file']
        cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino') + context.launch_configurations['robot_id'] + '_nav.rviz'
        
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
            choices=['true', 'false'],
            description='Flag to toggle between real robot and simulation'
        ),
        DeclareLaunchArgument(
            name='robot_id', 
            default_value=robot_id,
            description='ID of the robot'
        ),
        DeclareLaunchArgument(
            name='map_file', 
            default_value=map_file,
            # choices=['lab1', 'povo', 'hexagon'],
            description='Map to load for localization'
        ),
        DeclareLaunchArgument(
            name='nav2_params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RVIZ config file to use'
        ),
        DeclareLaunchArgument(
            name='nav2_autostart',
            default_value=nav2_autostart,
            choices=['true', 'false'],
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            name='remote_nav', 
            default_value=remote_nav,
            choices=['true', 'false'],
            description='Flag to toggle between navigation stack running on robot or locally'
        ),
        DeclareLaunchArgument(
            name='headless', 
            default_value=headless,
            choices=['true', 'false'],
            description='Flag to toggle between navigation stack running on robot or locally'
        ),
        DeclareLaunchArgument(
            name='initial_x',
            default_value=initial_x,
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_y',
            default_value=initial_y,
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_yaw',
            default_value=initial_yaw,
            description='Initial yaw position of the robot'
        ),
        
        OpaqueFunction(function=print_env),
        OpaqueFunction(function=check_exists),
        OpaqueFunction(function=evaluate_rviz),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'topic_name': "/map"},
                {'frame_id': "map"},
                {'yaml_filename': map_file}
            ],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[configured_params],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace= robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings = [
                ('cmd_vel', 'cmd_vel_nav'), 
                ('cmd_vel_smoothed', 'cmd_vel')
            ],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            namespace= robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': nav2_autostart},
                {'bond_timeout': 0.0},
                {'node_names': lifecycle_nodes_loc}
            ],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            namespace= robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': nav2_autostart},
                {'bond_timeout': 0.0},
                {'node_names': lifecycle_nodes_nav}
            ],
            condition=UnlessCondition(remote_nav),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace= robot_name,
            arguments=['-d', rviz_config_file],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            condition=UnlessCondition(headless),
            output='screen'
        ),
    ])

    
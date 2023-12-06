#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world', default='empty')
    world_path = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_gazebo'),'worlds', ''), world, '.world', "'"])
    n_robots = LaunchConfiguration('n_robots', default='3')

    declare_gui = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                            description='Flag to enable gazebo visualization')
    declare_rviz = DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
                            description='Flag to enable rviz visualization')
    declare_world = DeclareLaunchArgument(name='world', default_value='empty', choices=['empty', 'povo', 'hexagon'],
                            description='World used in the gazebo simulation')
    declare_n_robots = DeclareLaunchArgument(name='n_robots', default_value='3',
                            description='Number of robots in the simulation')

    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    xacro_model = os.path.join(gazebo_models_path, 'shelfino', 'model.sdf.xacro')
    
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Obstacles, borders and gates info
    start_obstacles = Node(
                        package='send_obstacles',
                        executable='send_obstacles'
                    )

    start_borders = Node(
                        package='send_borders',
                        executable='send_borders'
                    )

    start_gates = Node(
                        package='send_gates',
                        executable='send_gates'
                    )

    # Start Gazebo with plugin providing the robot spawning service
    start_gzserver = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
                            ),
                            launch_arguments={'world': world_path}.items(),
                        )

    start_gzclient = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                            ),
                            condition=IfCondition(gui)
                        )

    def evaluate_n_robots(context, *args, **kwargs):
        nr = LaunchConfiguration('n_robots').perform(context)
        # Define commands for launching the navigation instances
        nav_instances_cmds = []
        for i in range(int(nr)):
            model = os.path.join(gazebo_models_path,'shelfino','shelfino') + str(i+1) + '.sdf'
            rn = 'shelfino' + str(i+1)
            rviz_path = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino.rviz')
            rviz_config = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino' + str(i+1) + '.rviz')
            
            f = open(rviz_path,'r')
            filedata = f.read()
            f.close()

            newdata = filedata.replace("shelfinoX",rn)

            f = open(rviz_config,'w')
            f.write(newdata)
            f.close()

            group = GroupAction([
                ExecuteProcess(
                    cmd=[[
                        'xacro ',
                        xacro_model,
                        ' robot_id:=',
                        str(i+1),
                        ' > ',
                        model
                    ]],
                    shell=True
                ),

                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-file', model,
                            '-entity', 'shelfino'+str(i+1),
                            '-robot_namespace', 'shelfino'+str(i+1),
                            '-x', '0',
                            '-y', str(i-(int(int(nr)/2)))]
                ),


                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
                    launch_arguments={'use_sim_time': use_sim_time,
                                    'robot_id': str(i+1)}.items()
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    namespace='shelfino'+str(i+1),
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', rviz_config],
                    condition=IfCondition(rviz),
                ),


                # Node(
                #     package='get_positions',
                #     executable='get_positions',
                #     namespace='shelfino'+str(i+1),
                #     ),
            ])

            nav_instances_cmds.append(group)
        return nav_instances_cmds

    nav_instances_cmds = OpaqueFunction(function=evaluate_n_robots)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_gui)
    ld.add_action(declare_rviz)
    ld.add_action(declare_world)
    ld.add_action(declare_n_robots)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)

    ld.add_action(start_obstacles)
    ld.add_action(start_borders)
    ld.add_action(start_gates)

    ld.add_action(nav_instances_cmds)

    # for simulation_instance_cmd in nav_instances_cmds:
    #     ld.add_action(simulation_instance_cmd)

    return ld
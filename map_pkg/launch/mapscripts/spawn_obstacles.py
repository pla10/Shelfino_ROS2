import os, yaml, random
import numpy as np
from pathlib import Path

from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from geo_utility import *
from spawn_borders import get_borders_points


def spawn_obstacles(context):
    gen_map_params_file = context.launch_configurations['gen_map_params_file']
    with open(gen_map_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        obstacles = params["/**/send_obstacles"]['ros__parameters']

    box_model_path = os.path.join(context.launch_configurations['elements_models_path'], 'box', 'model.sdf')
    cylinder_model_path = os.path.join(context.launch_configurations['elements_models_path'], 'cylinder', 'model.sdf')
    
    # Check that all vectors have the same number of obstacles
    assert len(obstacles['vect_x']) == len(obstacles['vect_y']) == \
        len(obstacles['vect_yaw']) == len(obstacles["vect_type"]) == \
        len(obstacles['vect_dim_x']) == len(obstacles["vect_dim_y"]), \
        "The number of obstacles in the conf file is wrong. The script for generating the configuration made a mistake."

    nodes = []
    for obs in range(len(obstacles['vect_x'])):
        # If x and y are 0, then the gate is random

        if obstacles['vect_type'][obs] == "box":
            obs_polygon = rectangle(obstacles['vect_x'][obs], obstacles['vect_y'][obs], obstacles['vect_dim_x'][obs], obstacles['vect_dim_y'][obs], obstacles['vect_yaw'][obs])
            with open(box_model_path, 'r') as in_file:
                obs_model = in_file.read().replace("<size>1 1 1</size>", f"<size>{obstacles['vect_dim_x'][obs]} {obstacles['vect_dim_y'][obs]} 1</size>")
        
        elif obstacles['vect_type'][obs] == "cylinder":
            obs_polygon = circle(obstacles['vect_x'][obs], obstacles['vect_y'][obs], obstacles['vect_dim_x'][obs])
            with open(cylinder_model_path, 'r') as in_file:
                obs_model = in_file.read().replace("<radius>0.5</radius>", f"<radius>{obstacles['vect_dim_x'][obs]}</radius>")
        else:
            raise Exception(f"The obstacle type {obstacles['vect_type'][obs]} is not supported")

        center = [centroid(obs_polygon).coords[0][0], centroid(obs_polygon).coords[0][1]]
        print("Spawning obstacle in ", center, abs(center[0]), abs(center[1]))
        if abs(center[0]) < 1e-8:
            center[0] = 0
        if abs(center[1]) < 1e-8:
            center[1] = 0


        with open(f"obs{obs}_tmp.sdf", 'w') as out_file:
            out_file.write(obs_model)
            path = Path(out_file.name).resolve()
        

        nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                            '-file', PythonExpression(["'", str(path), "'"]),
                            '-entity', PythonExpression(["'", f"obstacles{obs}", "'"]),
                            '-x', PythonExpression(["'", str(center[0]), "'"]),
                            '-y', PythonExpression(["'", str(center[1]), "'"]),
                            '-z', PythonExpression(["'", str(0.0001), "'"]),
                            '-Y', PythonExpression(["'", str(obstacles["vect_yaw"][obs]), "'"])
                ]
            )
        )

    return nodes



#!/usr/bin/env python3

import os, math
from pathlib import Path
import yaml

from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def get_borders_points(yaml_file):
    try:
        if "hex" in yaml_file['/**']['ros__parameters']['map']:
            return gen_hex_points(float(yaml_file['/**']['ros__parameters']['dx']))
        elif "rect" in yaml_file['/**']['ros__parameters']['map']:
            return gen_rect_points(float(yaml_file['/**']['ros__parameters']['dx']), float(yaml_file['/**']['ros__parameters']['dy']))
        else:
            raise Exception("[{}] Map type `{}` not supported".format(__file__, yaml_file['/**']['ros__parameters']['map']))
        
    except Exception as e:
        print("The yaml file does not contain the map type")
        return []


def gen_hex_points(L):
    sqrt3o2 = math.sqrt(3) / 2

    x0, y0 = L, 0
    x1, y1 = L/2, L*sqrt3o2
    x2, y2 = -L/2, L*sqrt3o2
    x3, y3 = -L, 0
    x4, y4 = -L/2, -L*sqrt3o2
    x5, y5 = L/2, -L*sqrt3o2

    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

def gen_rect_points(L1, L2):
    return [(L1, L2), (-L1, L2), (-L1, -L2), (L1, -L2)]

def spawn_borders(context):
    map_env_params_file = context.launch_configurations['map_env_params_file']
    with open(map_env_params_file, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        map_type = params["/**"]['ros__parameters']['map']
        map_dx = params["/**"]['ros__parameters']['dx']
        map_dy = params["/**"]['ros__parameters']['dy']

    borders_model = ""

    if "hex" in map_type:
        L = map_dx
        sqrt3o2 = math.sqrt(3) / 2

        [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)] = gen_hex_points(L)

        c0 = ((x0+x1)/2, (y0+y1)/2)
        c1 = ((x1+x2)/2, (y1+y2)/2)
        c2 = ((x2+x3)/2, (y2+y3)/2)
        c3 = ((x3+x4)/2, (y3+y4)/2)
        c4 = ((x4+x5)/2, (y4+y5)/2)
        c5 = ((x5+x0)/2, (y5+y0)/2)

        with open(os.path.join(context.launch_configurations['gazebo_models_path'], 'hexagon_new', 'model.sdf'), 'r') as file:
            borders_model = file.read()

        borders_model = borders_model.replace("##L##", str(L))
        borders_model = borders_model.replace("##C0x##", str(c0[0])).replace("##C0y##", str(c0[1]))
        borders_model = borders_model.replace("##C1x##", str(c1[0])).replace("##C1y##", str(c1[1]))
        borders_model = borders_model.replace("##C2x##", str(c2[0])).replace("##C2y##", str(c2[1]))
        borders_model = borders_model.replace("##C3x##", str(c3[0])).replace("##C3y##", str(c3[1]))
        borders_model = borders_model.replace("##C4x##", str(c4[0])).replace("##C4y##", str(c4[1]))
        borders_model = borders_model.replace("##C5x##", str(c5[0])).replace("##C5y##", str(c5[1]))

        borders_points = [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

    elif "rect" in map_type:
        width = 0.15
        L1 = (map_dx + width) / 2
        L2 = (map_dy + width) / 2
        with open(os.path.join(context.launch_configurations['gazebo_models_path'], 'rectangle_world', 'model.sdf'), 'r') as file:
            borders_model = file.read().replace("dx", str(map_dx)).replace("dy", str(map_dy)).replace("width", str(width))
            borders_model = borders_model.replace("L1", str(L1)).replace("L2", str(L2))

    
    else:
        raise Exception("[{}] Map type `{}` not supported".format(__file__, map_type))
    
    with open('tmp.sdf', 'w') as file:
        file.write(borders_model)
        # Get full path of the file
        path = Path(file.name).resolve()

    nodes = [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                        '-file', PythonExpression(["'", str(path), "'"]),
                        '-entity', PythonExpression(["'borders1'"]),
                        '-x', PythonExpression(["'0.0'"]),
                        '-y', PythonExpression(["'0.0'"]),
                        '-z', PythonExpression(["'0.0'"])
            ]
        )]

    return nodes
#!/usr/bin/env python3

import yaml
import os
import math
import random
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

try:
    from launch.mapscripts.geo_utility import rectangle, circle, square, hexagon, is_in_map_polygons, center
except Exception as e:
    import sys
    sys.path.append(os.path.join(get_package_share_directory('map_pkg'), 'launch', 'mapscripts'))
    from geo_utility import rectangle, circle, square, hexagon, is_in_map_polygons, center, Polygon


SHELFINO_WIDTH = 0.5
SHELFINO_LENGTH = 0.4

class MapConfig:
    def __init__(self, type = None, dx = 0, dy = 0):
        assert type in ["hex", "rect", "hexagon", "rectangle"]
        self.type = type
        self.dx = dx
        self.dy = dy
        if "hex" in type:
            self.dy == self.dx

    def polygon(self):
        if "hex" in self.type:
            return hexagon(0, 0, self.dx)
        elif "rect" in self.type:
            return rectangle(0, 0, self.dx, self.dy)
        else:
            raise Exception("Map type `{}` not supported, how did you get here?".format(self.type))

    def gen_hex_points(self):
        L = self.dx
        sqrt3o2 = math.sqrt(3) / 2

        x0, y0 = L, 0
        x1, y1 = L/2, L*sqrt3o2
        x2, y2 = -L/2, L*sqrt3o2
        x3, y3 = -L, 0
        x4, y4 = -L/2, -L*sqrt3o2
        x5, y5 = L/2, -L*sqrt3o2

        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

    def gen_rect_points(self):
        L1, L2 = self.dx/2.0, self.dy/2.0
        return [(L1, L2), (-L1, L2), (-L1, -L2), (L1, -L2)]

    def get_points(self):
        if "hex" in self.type:
            return self.gen_hex_points()
        elif "rect" in self.type:
            return self.gen_rect_points()
        else:
            raise Exception("Map type `{}` not supported, how did you get here?".format(self.type))

    def __str__(self):
        return f"Map of type {self.type} with size ({self.dx}, {self.dy})"


class ShelfinoConfig:
    def __init__(self, name = None, x = 0, y = 0, yaw = 0):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw

    def polygon(self):
        polygon = rectangle(self.x, self.y, SHELFINO_WIDTH, SHELFINO_LENGTH, self.yaw)
        return polygon
    
    def check(self, map_config : MapConfig):
        return is_in_map_polygons(self.polygon(), map_config.polygon())
    
    def random(self, map_config : MapConfig):
        self.__random(map_config)
        while not self.check(map_config):
            self.__random(map_config)

    def __random(self, map_config : MapConfig):
        self.x = random.uniform(-map_config.dx/2.0, map_config.dx/2.0)
        self.y = random.uniform(-map_config.dy/2.0, map_config.dy/2.0)
        self.yaw = random.uniform(-np.pi, np.pi)

    def __str__(self):
        return f"Shelfino {self.name} at ({self.x}, {self.y}) with yaw {self.yaw}"

class GateConfig:
    def __init__(self, name = None, x = 0, y = 0, dx = 1.0, yaw = 0):
        self.name = name
        self.dx = dx
        self.yaw = yaw
        self.center = (x, y)
    
    def random(self, map_config : MapConfig, shelfini : list, gates : list) -> None:
        self.__random_config(map_config)
        while not self.check(map_config, shelfini, gates):
            self.__random_config(map_config)

    def polygon(self) -> Polygon:
        return square(self.x, self.y, self.dx, self.yaw)

    def check(self, map_config : MapConfig, shelfini : list, gates : list) -> bool:
        self_polygon = self.polygon()
        if not is_in_map_polygons(self_polygon, map_config.polygon()):
            return False
        for shelfino in shelfini:
            if self_polygon.intersects(shelfino.polygon()):
                return False
        for gate in gates:
            if self_polygon.intersects(gate.polygon()):
                return False
        return True

    def __random_config(self, map_config : MapConfig) -> None:
        borders_points = map_config.get_points()
        
        DELTA = self.dx/2.0 + 0.1 # 0.1 is half the border width + something

        edge = random.choice(range(len(borders_points)))
        P0 = borders_points[edge]
        P1 = borders_points[(edge+1)%len(borders_points)]

        cx, cy = center(borders_points)
        
        # Find the point along the edge
        if abs(P0[0] - P1[0]) < 1e-8:
            # Vertical edge
            x = P0[0] - DELTA if P0[0] >= 0 else P0[0] + DELTA
            y = random.uniform(min(P0[1], P1[1]), max(P0[1], P1[1]))
            yaw = 0 if P0[1] < P1[1] else np.pi
        elif abs(P0[1] - P1[1]) < 1e-8:
            # Horizontal edge
            y = P0[1] - DELTA if P0[1] >= 0 else P0[1] + DELTA
            x = random.uniform(min(P0[0], P1[0]), max(P0[0], P1[0]))
            yaw = -np.pi/2 if P0[0] < P1[0] else np.pi/2
        else:
            x = random.uniform(min(P0[0], P1[0]), max(P0[0], P1[0]))
            y = P0[1] + (P1[1] - P0[1])/(P1[0] - P0[0])*(x - P0[0])

            # Move the points away from the wall 
            v = np.array([P1[0] - P0[0], P1[1] - P0[1]])
            v = v/np.linalg.norm(v)
            v = np.array([-v[1], v[0]])*DELTA
            dist_before = np.linalg.norm(np.array([x, y]) - np.array([cx, cy]))
            (new_x, new_y) = (x + v[0], y + v[1])
            yaw = np.arctan2(v[1], v[0]) + np.pi
            if dist_before < np.linalg.norm(np.array([new_x, new_y]) - np.array([cx, cy])):
                (new_x, new_y) = (x - v[0], y - v[1])
            (x, y) = (new_x, new_y)

            # Find the yaw

        self.x = x
        self.y = y
        self.yaw = yaw

    def __str__(self):
        return f"Gate {self.name} at ({self.x}, {self.y}) with size {self.dx} and yaw {self.yaw}"


class ObstacleConfig:
    def __init__(self, name = None, type = "box", x = 0, y = 0, dx = 1.0, dy = 1.0, yaw = 0):
        assert type in ["box", "cylinder"], "Obstacle type must be either box or cylinder"
        self.name = name
        self.type = type
        self.dx = dx
        self.dy = dy
        self.yaw = yaw
        self.x, self.y = x, y

    def polygon(self):
        if "box" in self.type:
            return rectangle(self.x, self.y, self.dx, self.dy, self.yaw)
        elif "cylinder" in self.type:
            return circle(self.x, self.y, self.dx/2.0)
    
    def check(self, map_config : MapConfig, shelfini : list, gates : list, obstacles : list) -> bool:
        self_polygon = self.polygon()
        if not is_in_map_polygons(self_polygon, map_config.polygon()):
            return False
        for shelfino in shelfini:
            if self_polygon.intersects(shelfino.polygon()):
                return False
        for gate in gates:
            if self_polygon.intersects(gate.polygon()):
                return False
        for obstacle in obstacles:
            if self_polygon.intersects(obstacle.polygon()):
                return False
        return True

    def random(self, map_config : MapConfig, shelfini : list, gates : list, obstacles : list, no_boxes = False, no_cylinders = False, max_size = 1, min_size = 0.5) -> None:
        assert not (no_boxes and no_cylinders), "Cannot have no_boxes and no_cylinders at the same time"
        assert max_size >= min_size, "Max size must be greater than or equal to min size"
        assert min_size >= 0, "Min size must be greater than or equal to 0"
        
        self.__random_config(map_config, no_boxes, no_cylinders, max_size, min_size)
        while not self.check(map_config, shelfini, gates, obstacles):
            self.__random_config(map_config, no_boxes, no_cylinders, max_size, min_size)

    def __random_config(self, map_config : MapConfig, no_boxes = False, no_cylinders = False, max_size = 1, min_size = 0.5) -> None:
        if no_boxes:
            self.type = "cylinder"
        elif no_cylinders:
            self.type = "box"
        else:
            self.type = random.choice(["box", "cylinder"])

        if self.type == "box":
            self.__random_box_config(map_config, max_size, min_size)
        elif self.type == "cylinder":
            self.__random_cylinder_config(map_config, max_size, min_size)
        else:
            raise Exception("Obstacle type `{}` not supported, how did you get here?".format(self.type))


    def __random_box_config(self, map_config : MapConfig, max_size = 1, min_size = 0.5) -> None:
        x = random.uniform(-map_config.dx/2.0, map_config.dx/2.0)
        y = random.uniform(-map_config.dy/2.0, map_config.dy/2.0)
        dx = random.uniform(min_size, max_size)
        dy = random.uniform(min_size, max_size)
        yaw = random.uniform(-np.pi, np.pi)

        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.yaw = yaw

    def __random_cylinder_config(self, map_config : MapConfig, max_size = 1, min_size = 0.5) -> None:
        x = random.uniform(-map_config.dx/2.0, map_config.dx/2.0)
        y = random.uniform(-map_config.dy/2.0, map_config.dy/2.0)
        dx = random.uniform(min_size, max_size)
        yaw = random.uniform(-np.pi, np.pi)

        self.x = x
        self.y = y
        self.dx = dx
        self.yaw = yaw

    def __str__(self):
        return f"Obstacle {self.name} at ({self.x}, {self.y}) with size ({self.dx}, {self.dy})"


###############################################################################


class VictimConfig:
    def __init__(self, name = None, x = 0, y = 0, weight = 1.0):
        self.name = name
        self.x, self.y = x, y
        self.weight = weight

    def polygon(self):
        return circle(self.x, self.y, 1)
    
    def check(self, map_config : MapConfig, shelfini : list, gates : list, obstacles : list) -> bool:
        self_polygon = self.polygon()
        if not is_in_map_polygons(self_polygon, map_config.polygon()):
            return False
        for shelfino in shelfini:
            if self_polygon.intersects(shelfino.polygon()):
                print(f"Victim {self} intersects with shelfino {shelfino}")
                return False
        for gate in gates:
            if self_polygon.intersects(gate.polygon()):
                print(f"Victim {self} intersects with gate {gate}")
                return False
        for obstacle in obstacles:
            if self_polygon.intersects(obstacle.polygon()):
                print(f"Victim {self} intersects with obstacle {obstacle}")
                return False
        return True

    def random(self, map_config : MapConfig, shelfini : list, gates : list, obstacles : list, min_weight = 0.5, max_weight = 1) -> None:
        assert max_weight >= min_weight, "Max weight must be greater than or equal to min weight"
        assert min_weight >= 0, "Min weight must be greater than or equal to 0"

        self.__random_config(map_config, min_weight, max_weight)
        while not self.check(map_config, shelfini, gates, obstacles):
            self.__random_config(map_config, min_weight, max_weight)

    def __random_config(self, map_config : MapConfig, min_weight = 0, max_weight = 0) -> None:
        self.x = random.uniform(-map_config.dx/2.0, map_config.dx/2.0)
        self.y = random.uniform(-map_config.dy/2.0, map_config.dy/2.0)
        self.weight = int(random.uniform(min_weight, max_weight))

    def __str__(self):
        return f"Victim {self.name} at ({self.x}, {self.y}) with weight {self.weight}"

###############################################################################


def generate_config_file(map_env_params_file, generated_config_file):
    assert map_env_params_file != generated_config_file, "The input and output files must be different"
    if not os.path.exists(map_env_params_file):
        raise Exception(f"Map config file `{map_env_params_file}` does not exist")
    if not os.path.exists(Path(generated_config_file).parent):
        raise Exception(f"The parent directory of the output file `{Path(generated_config_file).parent}` does not exist, hence cannot create")

    with open(map_env_params_file, "r") as file:
        config_yaml = yaml.safe_load(file)

    # Generate map config
    map_config = MapConfig(
        config_yaml['/**']['ros__parameters']['map'], 
        config_yaml['/**']['ros__parameters']['dx'], 
        config_yaml['/**']['ros__parameters']['dy']
    )

    # Generate Shelfini config
    shelfini_yaml = config_yaml['/**']['ros__parameters']
    shelfini = []
    assert len(shelfini_yaml["init_names"]) == len(shelfini_yaml["init_rand"]) == len(shelfini_yaml["init_x"]) == len(shelfini_yaml["init_y"]) == len(shelfini_yaml["init_yaw"])
    for i in range(len(shelfini_yaml["init_names"])):
        shelfino = ShelfinoConfig(name=shelfini_yaml["init_names"][i])
        if shelfini_yaml["init_rand"][i]:
            shelfino.random(map_config)
        else:
            shelfino.x = shelfini_yaml["init_x"][i]
            shelfino.y = shelfini_yaml["init_y"][i]
            shelfino.yaw = shelfini_yaml["init_yaw"][i]
            assert shelfino.check(map_config)
        shelfini.append(shelfino)

    # Generate gates config
    gates_yaml = config_yaml['/**/send_gates']['ros__parameters']
    assert len(gates_yaml['x']) == len(gates_yaml['y']) == len(gates_yaml['yaw'])
    gates = []
    for i in range(len(gates_yaml['x'])):
        gate = GateConfig()
        gate.name = f"gate{i}"
        if gates_yaml['x'][i] == 0 and gates_yaml['y'][i] == 0:
            gate.random(map_config, shelfini, gates)
        else:
            gate.x = gates_yaml['x'][i]
            gate.y = gates_yaml['y'][i]
            gate.yaw = gates_yaml['yaw'][i]
            assert gate.check(map_config=map_config, shelfini=shelfini, gates=gates), f"{gate} is not in the map or intersects with another gate"
        gates.append(gate)
    
    # Generate obstacles config
    obstacles_yaml = config_yaml['/**/send_obstacles']['ros__parameters']
    obstacles = []
    assert len(obstacles_yaml['vect_x']) == len(obstacles_yaml['vect_y']) == len(obstacles_yaml['vect_dim_x']) == len(obstacles_yaml['vect_dim_y']) == len(obstacles_yaml['vect_yaw']) == len(obstacles_yaml['vect_type'])
    for i in range(len(obstacles_yaml['vect_x'])):
        obstacle = ObstacleConfig(
            f"obstacle{i}",
            obstacles_yaml['vect_type'][i],
            obstacles_yaml['vect_x'][i],
            obstacles_yaml['vect_y'][i],
            obstacles_yaml['vect_dim_x'][i],
            obstacles_yaml['vect_dim_y'][i],
            obstacles_yaml['vect_yaw'][i]
        )
        assert obstacle.check(map_config, shelfini, gates, obstacles), f"{obstacle} is not in the map or intersects with another gate or obstacle"
        obstacles.append(obstacle)
    max_size = obstacles_yaml['max_size']
    min_size = obstacles_yaml['min_size']
    no_boxes = obstacles_yaml['no_boxes']
    no_cylinders = obstacles_yaml['no_cylinders']
    for i in range(int(obstacles_yaml['n_obstacles'])):
        obstacle = ObstacleConfig()
        obstacle.name = f"obstacle{len(obstacles)}"
        obstacle.random(map_config, shelfini, gates, obstacles, no_boxes, no_cylinders, max_size, min_size)
        obstacles.append(obstacle)
    
    # Generate victims config
    victims = []
    if config_yaml['/**/send_victims']['ros__parameters']['victims_activated']:
        victims_yaml = config_yaml['/**/send_victims']['ros__parameters']
        assert len(victims_yaml['vect_x']) == len(victims_yaml['vect_y']) == len(victims_yaml['vect_weight'])
        for i in range(len(victims_yaml['vect_x'])):
            victim = VictimConfig(
                f"victim{i}",
                victims_yaml['vect_x'][i],
                victims_yaml['vect_y'][i],
                victims_yaml['vect_weight'][i]
            )
            assert victim.check(map_config, shelfini, gates, obstacles), f"{victim} is not in the map {map_config} or intersects with another gate or obstacle"
            victims.append(victim)
        max_weight = float(victims_yaml['max_weight'])
        min_weight = float(victims_yaml['min_weight'])
        for i in range(int(victims_yaml['n_victims'])):
            victim = VictimConfig()
            victim.name = f"victim{len(victims)}"
            victim.random(map_config, shelfini, gates, obstacles, min_weight, max_weight)
            victims.append(victim)

    print(map_config)
    for shelfino in shelfini:
        print(shelfino)
    for gate in gates:
        print(gate)
    for obstacle in obstacles:
        print(obstacle)
    for victim in victims:
        print(victim)    

    with open(generated_config_file, "w+") as file:
        file.write(f"# This file was generated by generate_config_file.py on the content of {map_env_params_file}\n")

    full_conf_file = {}
    full_conf_file["/**"] = {
        "ros__parameters": {}  
    }

    for key in config_yaml["/**"]["ros__parameters"].keys():
        if key != "init_rand":
            full_conf_file["/**"]["ros__parameters"][key] = config_yaml["/**"]["ros__parameters"][key]
        else:
            full_conf_file["/**"]["ros__parameters"][key] = [False for _ in range(len(shelfini))]

    full_conf_file["/**"]["ros__parameters"]["use_sim_time"] = True

    # Set gates info
    full_conf_file["/**/send_gates"] = {
        "ros__parameters": {}
    }
    full_conf_file["/**/send_gates"]["ros__parameters"]["x"] = [float(gate.x) for gate in gates]
    full_conf_file["/**/send_gates"]["ros__parameters"]["y"] = [float(gate.y) for gate in gates]
    full_conf_file["/**/send_gates"]["ros__parameters"]["yaw"] = [float(gate.yaw) for gate in gates]

    # Set obstacles info
    full_conf_file["/**/send_obstacles"] = {
        "ros__parameters": {}
    }
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_type"] = [str(obstacle.type) for obstacle in obstacles]
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_x"] = [float(obstacle.x) for obstacle in obstacles]
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_y"] = [float(obstacle.y) for obstacle in obstacles]
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_yaw"] = [float(obstacle.yaw) for obstacle in obstacles]
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_dim_x"] = [float(obstacle.dx) for obstacle in obstacles]
    full_conf_file["/**/send_obstacles"]["ros__parameters"]["vect_dim_y"] = [float(obstacle.dy) for obstacle in obstacles]

    # Set victims info
    full_conf_file["/**/send_victims"] = {
        "ros__parameters": {}
    }
    full_conf_file["/**/send_victims"]["ros__parameters"]["victims_activated"] = config_yaml['/**/send_victims']['ros__parameters']['victims_activated']
    if config_yaml['/**/send_victims']['ros__parameters']['victims_activated']:
        full_conf_file["/**/send_victims"]["ros__parameters"]["max_weight"] = max_weight
        full_conf_file["/**/send_victims"]["ros__parameters"]["min_weight"] = min_weight
        full_conf_file["/**/send_victims"]["ros__parameters"]["vect_x"] = [float(victim.x) for victim in victims]
        full_conf_file["/**/send_victims"]["ros__parameters"]["vect_y"] = [float(victim.y) for victim in victims]
        full_conf_file["/**/send_victims"]["ros__parameters"]["vect_weight"] = [float(victim.weight) for victim in victims]

    # Set timeout info
    full_conf_file["/**/send_timeout"] = {
        "ros__parameters": {
            "victims_timeout": config_yaml["/**/send_victims"]["ros__parameters"]["victims_timeout"]
        }
    }

    yaml.dump(full_conf_file, open(generated_config_file, "a"))


class Configurator(Node):
    def __init__(self):
        super().__init__('configurator')
        
        self.declare_parameter('map_env_params_file', os.path.join(get_package_share_directory('map_pkg'), 'config', 'map_config.yaml'))
        self.declare_parameter('gen_map_params_file', os.path.join(get_package_share_directory('map_pkg'), 'config', 'full_config.yaml'))
        self.declare_parameter('generate_new_config', True)

        map_env_params_file = self.get_parameter('map_env_params_file').get_parameter_value().string_value
        gen_map_params_file = self.get_parameter('gen_map_params_file').get_parameter_value().string_value
        generate_new_config = self.get_parameter('generate_new_config').get_parameter_value().bool_value

        # generate_new_config = True if generate_new_config.lower() == "true" else False
        # victims_activated = True if victims_activated.lower() == "true" else False

        print(f"map_env_params_file: {map_env_params_file}")
        print(f"gen_map_params_file: {gen_map_params_file}")
        print(f"generate_new_config: {generate_new_config}")

        self.get_logger().info(f"Reading conf template from {map_env_params_file} and writing to {gen_map_params_file}")

        if generate_new_config:
            generate_config_file(map_env_params_file, gen_map_params_file)
            self.get_logger().info("Config file generated")
        else:
            if not os.path.exists(gen_map_params_file):
                self.get_logger().warn("Config file does not exist, but was not requested to be generated")
            else:
                self.get_logger().info("Config file already generated")


def main(args=None):
    rclpy.init(args=args)
    configurator = Configurator()
    # rclpy.spin(configurator)
    configurator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
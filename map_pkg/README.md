# MAP PACKAGE

That's one small package for this repo, an exhausting one for me, semicit.

## Introduction

This package contains all the necessary nodes to create the map and to provide information on it.

The nodes are:

- `generate_config_file.py`: takes the config file from `config/map_config.yaml` and creates a file 
  containing the full configuration of the map, namely `full_config.yaml`, inside 
  `install/map_pkg/share/map_pkg/config`.

- `spawn_borders`: reads `full_config.yaml` and spawns the walls in Gazebo
- `spawn_gates`: reads `full_config.yaml` and spawns the gate(s) in Gazebo
- `spawn_obstacle`: reads `full_config.yaml` and spawns the obstacles in Gazebo

- `send_borders`: publishes the margin of the map 
- `send_gates`: publishes the position of one or more gates
- `send_obstacles`: publishes the position of the obstacles
- `send_victims`: publishes the position of the victims (if they are needed for the scenario)

These are the nodes from which you will take the information on the map, more on them later. 

There also other scripts and configuration files that are important:

- `scripts/create_map_pgm.py` allows for dynamically restructuring the borders of the map to basically any polygon (don't quote me on this)
- `models` contains all the Gazebo models for the simulation (not the one of the robot, which is in `shelfino_description` instead)

And finally the orchestrators:

- `launch/spawn_map.launch.py` allows for launching the nodes in the correct order with the right parameters

## Topics

The following topics can be used to get information on the map:

- `/borders`: the vertexes of the map. The message is of type `geometry_msgs::msg::Polygon`.
- `/gates`: the centers and orientations of the gates, which are rectangles of size 1x1. The message is of type `geometry_msgs::msg::PoseArray`.
- `/obstacles`: the obstacles of type `obstacles_msgs::msg::ObstacleArrayMsg`. The `ObstacleArrayMsg` contains an array of `ObstacleMsg`, which have the following fields:
  - `polygon` of type `geometry_msgs::msgs::Polygon`, which contains the vertexes of the obstacle if it is a polygon, or just the center of the cylinder otherwhise.
  - `radius` of type `float64`, which is set only if the obstacle is a cylinder.
- `/victims`: the victims of type `obstacles_msgs::msg::ObstacleArrayMsg`. The `ObstacleArrayMsg` contains an array of `ObstacleMsg`, which have the following fields:
  - `polygon` of type `geometry_msgs::msgs::Polygon`, which contains the center of the victim.
  - `radius` of type `float64`, which in victims reflects the weight of the victim.
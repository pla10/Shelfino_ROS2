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
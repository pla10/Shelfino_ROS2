# MAP PACKAGE

That's one small package for this repo, an exhausting one for me, semicit.

## Introduction

This package contains all the necessary nodes to create the map and to provide information on it.

The nodes are:

- `send_borders`: publish the margin of the map and spawns them in Gazebo
- `send_gates`: publish the position of one or more gates
- `send_obstacles`: publish the position of the obstacles
- `send_victims`: publish the position of the victims (if they are needed for the scenario)

These are the nodes from which you will take the information on the map, more on them later. 

There also other scripts and configuration files that are important:

- `scripts/create_map_pgm.py` allows for dynamically restructuring the borders of the map to basically any polygon (don't quote me on this)
- `config/map_config.yaml` contains the necessary configuration variables to run the simulation
- `models` contains all the Gazebo models for the simulation 

And finally the orchestrators:

- `launch/map_env.launch.py` allows for launching the nodes in the correct order with the right parameters
- `orchestrator` is a node that starts the other `send_*` nodes in the correct order.

## `send_*` nodes
# Shelfino ROS2

This repository contains the ROS 2 interface developed for the mobile robot Shelfino of the Department of Information Engineering and Computer Science of the University of Trento, Italy.

## Robot Planning and its Applications

Check `projects/README.md` for the description of the projects.

## Shelfino topics

The topics provided by the interface are:

|       TOPIC        | Description |
| ------------------ | ----------- |
| /scan              | Data from the Lidar |
| /odom              | Data of the odometry (sensor fusion of the RealSense and encoders data) |
| /t265              | Data from the RealSense camera |
| /joint_states      | Data from the encoders of the wheels |
| /cmd_vel           | Topic to control the movement of the robot acting on the velocities |
| /robot_description | The urdf description of the shelfino robot |
| /tf_static         | The fixed transform between the *base_link* frame and the *base_laser* frame |
| /tf                | The tranforms between the robot and the *odom* frame, and between the wheels and the robot body |


---
## Complete documentation

Check the [complete documentation](https://pla10.github.io/Shelfino_ROS2) of this repository.

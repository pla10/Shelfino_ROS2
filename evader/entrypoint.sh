#!/usr/bin/env sh

# Source the setup
. /opt/ros/humble/setup.sh
. /home/evader/evader/install/setup.sh

# Set domain id if specified
if [ $# -eq 2 ]; then
    export ROS_DOMAIN_ID=$2
fi

echo "ROS_DOMAIN_ID" $ROS_DOMAIN_ID

# Run the node
ros2 run evader evader --ros-args -p difficulty:=$1 -r __ns:=/evader
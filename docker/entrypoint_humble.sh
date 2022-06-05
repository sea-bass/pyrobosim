#!/bin/bash

# Source ROS and the Colcon workspace
source /opt/ros/humble/setup.bash
if [ ! -f /pyrobosim_ws/install/setup.bash ]
then
  colcon build
fi
. /pyrobosim_ws/install/setup.bash

# Execute the command passed into this entrypoint
exec "$@"

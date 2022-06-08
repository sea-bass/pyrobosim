#!/bin/bash

# Source ROS and the Colcon workspace
source /opt/ros/humble/setup.bash
if [ ! -f /pyrobosim_ws/install/setup.bash ]
then
  colcon build
fi
. /pyrobosim_ws/install/setup.bash

# Add dependencies to path
PDDLSTREAM_PATH=/pyrobosim_ws/src/pyrobosim/dependencies/pddlstream
if [ -d "$PDDLSTREAM_PATH" ]
then
    export PYTHONPATH=$PDDLSTREAM_PATH:$PYTHONPATH
fi

# Execute the command passed into this entrypoint
exec "$@"

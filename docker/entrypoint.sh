#!/bin/bash

# Source ROS and the pyrobosim workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ ! -f /pyrobosim_ws/install/setup.bash ]
then
  colcon build
fi
source /pyrobosim_ws/install/setup.bash

# Activate virtual environment
source ${VIRTUAL_ENV}/bin/activate

# Add dependencies to path
PDDLSTREAM_PATH=/pyrobosim_ws/src/dependencies/pddlstream
if [ -d "$PDDLSTREAM_PATH" ]
then
    export PYTHONPATH=$PDDLSTREAM_PATH:$PYTHONPATH
fi

# Execute the command passed into this entrypoint
exec "$@"

#!/bin/bash

# Source ROS
source /opt/ros/foxy/setup.bash

# Execute the command passed into this entrypoint
exec "$@"
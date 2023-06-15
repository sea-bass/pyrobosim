#!/bin/bash

# Sets up the pyrobosim environment for development.
#
# One recommendation is to make a bash function for this script in
# your ~/.bashrc file as follows:
#
# For non-ROS workflows:
#
#  pyrobosim() {
#    source /path/to/pyrobosim/setup/source_pyrobosim.bash
#  }
#
#  So you can then run this from your Terminal:
#    pyrobosim
#
# For ROS workflows, enter the ROS distro (humble/iron) as an argument:
#
#   pyrobosim_ros() {
#     source /path/to/pyrobosim/setup/source_pyrobosim.bash humble
#   }
#
#   So you can then call run from your Terminal:
#     pyrobosim_ros

# User variables -- change this to meet your needs
export VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim
export PYROBOSIM_WS=~/pyrobosim_ws

if [ -n "$VIRTUAL_ENV" ]
then
    deactivate
fi

# Parse ROS distro argument
ROS_DISTRO=$1
if [ "$ROS_DISTRO" == "" ]
then
    echo "Setting up pyrobosim with no ROS distro."
else
    echo "Setting up pyrobosim with ROS $ROS_DISTRO"
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd $PYROBOSIM_WS
    if [ ! -d "build" ]
    then
        echo "Building colcon workspace..."
        colcon build
    fi
    . install/local_setup.bash
fi

# Activate the Python virtual environment
source $VIRTUALENV_FOLDER/bin/activate

# Add dependencies to path
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PDDLSTREAM_PATH=$SCRIPT_DIR/../dependencies/pddlstream
if [ -d "$PDDLSTREAM_PATH" ]
then
    export PYTHONPATH=$PDDLSTREAM_PATH:$PYTHONPATH
fi

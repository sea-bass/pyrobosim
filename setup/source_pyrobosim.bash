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

# Set up the environment
ENV_FILE="pyrobosim.env"
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd "${SCRIPT_DIR}/.." > /dev/null
if [ ! -f "${ENV_FILE}" ]
then
    popd > /dev/null
    echo "Did not find a file named '${ENV_FILE}' in the root pyrobosim folder."
    echo "Please rerun the 'setup_pyrobosim.bash' script to set up your environment."
    return 1
fi
unset PYROBOSIM_VENV PYROBOSIM_ROS_WORKSPACE PYROBOSIM_ROS_DISTRO
source "${ENV_FILE}"
popd > /dev/null

if [ -n "${VIRTUAL_ENV}" ]
then
    deactivate
fi

# Activate the Python virtual environment.
echo "Activated virtual environment at ${PYROBOSIM_VENV}."
source ${PYROBOSIM_VENV}/bin/activate

# Parse ROS workspace and distro arguments.
if [ -z "${PYROBOSIM_ROS_WORKSPACE}" ]
then
    echo "Setting up pyrobosim with no ROS distro."
else
    echo "Setting up pyrobosim with ROS ${PYROBOSIM_ROS_DISTRO}."
    source /opt/ros/${PYROBOSIM_ROS_DISTRO}/setup.bash

    if [ ! -d "${PYROBOSIM_ROS_WORKSPACE}/src" ]
    then
        echo -e "\nFolder '${PYROBOSIM_ROS_WORKSPACE}/src' does not exist."
        echo "Please rerun the 'setup_pyrobosim.bash' script to set up your environment."
        return 1
    fi

    pushd "${PYROBOSIM_ROS_WORKSPACE}" > /dev/null
    if [ ! -f "install/setup.bash" ]
    then
        echo "Building ROS workspace at ${PYROBOSIM_ROS_WORKSPACE}..."
        colcon build
    fi
    echo "Sourcing ROS workspace at ${PYROBOSIM_ROS_WORKSPACE}."
    source install/setup.bash
    popd > /dev/null
fi

# Add dependencies to path
PDDLSTREAM_PATH=${SCRIPT_DIR}/../dependencies/pddlstream
if [ -d "${PDDLSTREAM_PATH}" ]
then
    echo "Added PDDLStream to Python path."
    export PYTHONPATH=${PDDLSTREAM_PATH}:$PYTHONPATH
fi

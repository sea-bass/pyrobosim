#!/bin/bash

# Sets up the pyrobosim virtual environment

# User variables
# Please modify these for your environment
VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim
ROS_WORKSPACE=~/pyrobosim_ws

# Create a Python virtual environment
[ ! -d "${VIRTUALENV_FOLDER}" ] && mkdir -p ${VIRTUALENV_FOLDER}
python3 -m venv ${VIRTUALENV_FOLDER}
echo -e "Created Python virtual environment in ${VIRTUALENV_FOLDER}\n"

# Install all the Python packages required
# Note that these overlay over whatever ROS 2 already contains
source "${VIRTUALENV_FOLDER}/bin/activate"
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd "${SCRIPT_DIR}/.." > /dev/null
pip3 install ./pyrobosim
pip3 install -r test/python_test_requirements.txt

# Write key variables to file
ENV_FILE="pyrobosim.env"
if [ -f "${ENV_FILE}" ]
then
  rm ${ENV_FILE}
fi
echo "# This is an autogenerated environment file for pyrobosim" >> ${ENV_FILE}
echo "PYROBOSIM_VENV=${VIRTUALENV_FOLDER}" >> ${ENV_FILE}

# If setting up with ROS, install extra packages
echo -e ""
read -p "Do you want to set up ROS? (y/n) : " USE_ROS
if [ "${USE_ROS,,}" == "y" ]
then
  # Validate that the ROS workspace specified exists.
  if [ ! -d "${ROS_WORKSPACE}" ]
  then
      echo -e "\nFolder '${ROS_WORKSPACE}' does not exist."
      echo "Please configure it to the ROS workspace that contains the 'pyrobosim' repository."
      return 1
  fi

  read -p "What ROS distro are you using? (humble, iron, jazzy, rolling) : " ROS_DISTRO
  echo ""
  echo "Installing additional packages for ROS ${ROS_DISTRO,,} setup"
  echo "PYROBOSIM_ROS_WORKSPACE=${ROS_WORKSPACE}" >> ${ENV_FILE}
  echo "PYROBOSIM_ROS_DISTRO=${ROS_DISTRO,,}" >> ${ENV_FILE}
  pip3 install colcon_common_extensions
fi

# Optionally configure PDDLStream for task and motion planning
echo -e ""
read -p "Do you want to set up PDDLStream for task and motion planning? (y/n) : " USE_PDDLSTREAM
if [ "${USE_PDDLSTREAM,,}" == "y" ]
then
  ./setup/configure_pddlstream.bash
fi

popd > /dev/null
deactivate

# Print confirmation and instructions at the end
echo -e "


Created Python virtual environment and installed packages at the following location:

  ${VIRTUALENV_FOLDER}

Environment data has been written to the file ${ENV_FILE}.

Source the environment using the following command:

  source setup/source_pyrobosim.bash
"

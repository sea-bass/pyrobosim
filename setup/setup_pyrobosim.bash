#!/bin/bash

# Sets up the pyrobosim virtual environment

# User variables
# Please modify these for your environment
VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim

# Create a Python virtual environment
[ ! -d "${VIRTUALENV_FOLDER}" ] && mkdir -p ${VIRTUALENV_FOLDER}
python3 -m venv ${VIRTUALENV_FOLDER}
echo -e "Created Python virtual environment in ${VIRTUALENV_FOLDER}\n"

# Install all the Python dependencies required
# Note that these overlay over whatever ROS 2 already contains
source "${VIRTUALENV_FOLDER}/bin/activate"
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd "${SCRIPT_DIR}/.." > /dev/null
pip3 install setuptools
python3 pyrobosim/setup.py egg_info
pip3 install -r pyrobosim.egg-info/requires.txt
rm -rf pyrobosim.egg-info/
pip3 install -r test/python_test_requirements.txt

# Write key variables to file
ENV_FILE="pyrobosim.env"
if [ -f "${ENV_FILE}" ]
then
  rm ${ENV_FILE}
fi
echo "# This is an autogenerated environment file for pyrobosim" >> ${ENV_FILE}
echo "PYROBOSIM_VENV=${VIRTUALENV_FOLDER}" >> ${ENV_FILE}

# If setting up with ROS, perform additional setup.
echo -e ""
read -p "Do you want to set up ROS? (y/n) : " USE_ROS
if [ "${USE_ROS,,}" == "y" ]
then
  # Attempt to auto-detect ROS workspace from script location
  POTENTIAL_WS=$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")
  echo "Checking if ${POTENTIAL_WS}/src/pyrobosim exists..."

  if [ -d "${POTENTIAL_WS}/src/pyrobosim" ]; then
    ROS_WORKSPACE="${POTENTIAL_WS}"
    echo "[INFO] Detected ROS workspace at: ${ROS_WORKSPACE}"
  else
    echo -e "\n[ERROR] Could not detect a valid ROS workspace."
    echo "[INFO] Expected: ${POTENTIAL_WS}/src/pyrobosim"
    echo "[INFO] Please place the repo inside a ROS 2 workspace (e.g. my_ws/src/pyrobosim)"
    type deactivate &> /dev/null && deactivate
    exit 1
  fi

  rm -rf ${ROS_WORKSPACE}/build ${ROS_WORKSPACE}/install ${ROS_WORKSPACE}/log

  read -p "What ROS distro are you using? (humble, jazzy, rolling) : " ROS_DISTRO
  echo ""
  echo "Installing additional packages for ROS ${ROS_DISTRO,,} setup"
  echo "PYROBOSIM_ROS_WORKSPACE=${ROS_WORKSPACE}" >> ${ENV_FILE}
  echo "PYROBOSIM_ROS_DISTRO=${ROS_DISTRO,,}" >> ${ENV_FILE}

  # Install packages needed to run colcon build and use rclpy from within our virtual environment.
  pip3 install colcon-common-extensions typing_extensions

  # Install any ROS package dependencies that may be missing.
  pushd ${ROS_WORKSPACE} > /dev/null
  rosdep install --from-paths src -y --ignore-src --rosdistro ${ROS_DISTRO}
  popd
else
  # Install pyrobosim using pip in the non-ROS case.
  pip3 install -e ./pyrobosim
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

# Sets up the pyrobosim environment for development
#
# One recommendation is to make a bash function for this script in 
# your ~/.bashrc file as follows:
#
# pyrobosim_ros() {
#   source /path/to/pyrobosim/setup/setup_pyrobosim.bash
# }
#
# So you can then simply call this from your Terminal:
#   pyrobosim_ros

# User variables -- change this to meet your needs
export ROS_DISTRO=foxy
export VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim
export PYROBOSIM_WS=~/pyrobosim_ws

# Do not modify below this line
source $VIRTUALENV_FOLDER/bin/activate
source /opt/ros/$ROS_DISTRO/setup.bash  
cd $PYROBOSIM_WS
. install/local_setup.bash

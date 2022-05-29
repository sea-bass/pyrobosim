# Sets up the pyrobosim environment for development
#
# One recommendation is to make a bash function for this script in 
# your ~/.bashrc file as follows:
#
# For non-ROS workflows:
#
#  pyrobosim() {
#    source /path/to/pyrobosim/setup/setup_pyrobosim.bash  
#  }
#
#  So you can then run this from your Terminal:
#    pyrobosim
#
# For ROS workflows, enter the ROS distro (foxy/humble) as an argument:
#
#   pyrobosim_ros() {
#     source /path/to/pyrobosim/setup/setup_pyrobosim.bash humble
#   }
#
#   So you can then call run from your Terminal:
#     pyrobosim_ros

# User variables -- change this to meet your needs
export VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim
export PYROBOSIM_WS=~/pyrobosim_ws

# Parse ROS distro argument
ROS_DISTRO=$1
if [ "$ROS_DISTRO" == "" ]
then
    echo "Setting up pyrobosim with no ROS distro."
else
    echo "Setting up pyrobosim with ROS $ROS_DISTRO"
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Do not modify below this line
source $VIRTUALENV_FOLDER/bin/activate
cd $PYROBOSIM_WS
. install/local_setup.bash

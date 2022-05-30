# Sets up the pyrobosim virtual environment

# User variables
VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim

# Create a Python virtual environment
[ ! -d "$VIRTUALENV_FOLDER" ] && mkdir -p $VIRTUALENV_FOLDER
python3 -m venv $VIRTUALENV_FOLDER
source $VIRTUALENV_FOLDER/bin/activate
echo "Created Python virtual environment in $VIRTUALENV_FOLDER"

# Install all the Python packages required
# Note that these overlay over whatever ROS2 already contains
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd $SCRIPT_DIR/../pyrobosim
pip3 install lark pytest pytest-html wheel
pip install .
popd
echo "Installed Python packages"

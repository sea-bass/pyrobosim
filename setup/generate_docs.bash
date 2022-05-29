# Generate Sphinx documentation
#
# Note that you may need some additional Python packages:
# pip3 install sphinx sphinx-rtd-theme

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source $SCRIPT_DIR/setup_pyrobosim.bash
cd $SCRIPT_DIR/../docs
make html

# Generate Sphinx documentation

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source $SCRIPT_DIR/setup_pyrobosim.bash
cd $SCRIPT_DIR/../docs
make html

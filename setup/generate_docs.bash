#!/bin/bash

# Generate Sphinx documentation
#
# Note that you may need some additional Python packages:
# pip3 install sphinx sphinx-rtd-theme

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "${SCRIPT_DIR}/source_pyrobosim.bash"
pushd "${SCRIPT_DIR}/../docs" || exit
rm -rf build/
rm -rf source/generated
make html
popd || exit

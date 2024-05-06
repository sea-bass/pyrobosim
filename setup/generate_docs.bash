#!/bin/bash

# Generate Sphinx documentation
#
# Note that you may need some additional Python packages:
# pip3 install -r docs/python_docs_requirements.txt

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "${SCRIPT_DIR}/source_pyrobosim.bash"
pushd "${SCRIPT_DIR}/../docs" > /dev/null || exit
rm -rf build/
rm -rf source/generated
make html
popd > /dev/null || exit

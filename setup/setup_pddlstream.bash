#!/bin/bash

# Set up PDDLStream

# Set up the dependencies folder
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DEPENDS_DIR=${SCRIPT_DIR}/../dependencies
if [ ! -d "${DEPENDS_DIR}" ]
then
    mkdir ${DEPENDS_DIR}
fi

# Clone and build PDDLStream
pushd ${SCRIPT_DIR}/../dependencies > /dev/null || exit
git clone https://github.com/caelan/pddlstream.git
pushd pddlstream > /dev/null || exit
touch COLCON_IGNORE
git submodule update --init --recursive
./downward/build.py
popd > /dev/null || exit
popd > /dev/null || exit

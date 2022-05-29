#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
pushd src/pyrobosim/pyrobosim
pip3 install .
pip3 install lark pytest pytest-html
popd
pushd src/pyrobosim/test
run_tests.bash
popd

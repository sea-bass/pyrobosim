#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
cd src/pyrobosim
pip3 install .
pip3 install pytest pytest-html
./test/run_tests.bash

#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
cd src/pyrobosim
pip3 install .
./test/run_tests.bash

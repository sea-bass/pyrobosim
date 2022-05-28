#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
cd src/pyrobosim
pip3 install -e pyrobosim
pip3 install lark pytest pytest-html
./test/run_tests.bash

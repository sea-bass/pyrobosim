#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
cd src/pyrobosim
pip3 install .
python3 -m pytest --junitxml=test_results/test_results.xml

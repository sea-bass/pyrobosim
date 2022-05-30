#!/bin/bash

# Runs tests from inside a Docker container
echo "Running Docker tests..."
colcon build
. install/setup.bash
pushd src/pyrobosim/test
./run_tests.bash
popd

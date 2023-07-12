#!/bin/bash

# Runs all unit tests
#
# If not using ROS, simply run
#   ./run_tests.bash
#
# If using ROS, additionally pass in your ROS_DISTRO argument (e.g., humble or rolling)
#   ./run_tests.bash ${ROS_DISTRO}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
TEST_RESULTS_DIR=$SCRIPT_DIR/results

# Run regular pytest tests
echo "Running Python package unit tests..."
python3 -m pytest "$SCRIPT_DIR/utils" \
 --junitxml="$TEST_RESULTS_DIR/test_results.xml" \
 --html="$TEST_RESULTS_DIR/test_results.html" --self-contained-html

# Run colcon tests, if using a ROS distro
ROS_DISTRO=$1
if [ "$ROS_DISTRO" == "" ]
then
    echo "Running ROS package unit tests..."
    pushd "${SCRIPT_DIR}/../../.." || exit
    colcon test --event-handlers console_direct+
    colcon test-result --verbose
    popd || exit
fi

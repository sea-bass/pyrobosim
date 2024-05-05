#!/bin/bash

# Runs all unit tests
#
# If not using ROS, simply run
#   ./run_tests.bash
#
# If using ROS, additionally pass in your ROS_DISTRO argument (e.g., humble or rolling)
#   ./run_tests.bash ${ROS_DISTRO}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
TEST_RESULTS_DIR="${SCRIPT_DIR}/results"
mkdir -p "${TEST_RESULTS_DIR}"

# Ensure we run everything for coverage purposes, but ensure failures are returned by the script
SUCCESS=0

# Do not swallow errors with tee
set -o pipefail

# Run regular pytest tests
echo "Running Python package unit tests..."
python3 -m pytest "${SCRIPT_DIR}" \
 --cov="${SCRIPT_DIR}/../pyrobosim/pyrobosim" --cov-branch \
 --cov-report term \
 --cov-report html:"${TEST_RESULTS_DIR}/test_results_coverage_html" \
 --cov-report xml:"${TEST_RESULTS_DIR}/test_results_coverage.xml" \
 --junitxml="${TEST_RESULTS_DIR}/test_results.xml" \
 --html="${TEST_RESULTS_DIR}/test_results.html" \
 --self-contained-html \
 | tee "${TEST_RESULTS_DIR}/pytest-coverage.txt" || SUCCESS=$?
echo ""

# Run ROS package tests, if using a ROS distro
ROS_DISTRO=$1
if [[ -n "${ROS_DISTRO}" && -n "${COLCON_PREFIX_PATH}" ]]
then
    WORKSPACE_DIR="${COLCON_PREFIX_PATH}/../"
    ROS_PKG_DIR="${WORKSPACE_DIR}/src/pyrobosim_ros"
    echo "Running ROS package unit tests from ${ROS_PKG_DIR} ..."
    python3 -m pytest "${ROS_PKG_DIR}" \
     --junitxml="${TEST_RESULTS_DIR}/test_results_ros.xml" \
     --html="${TEST_RESULTS_DIR}/test_results_ros.html" \
     --self-contained-html || SUCCESS=$?
    echo ""
fi

exit ${SUCCESS}

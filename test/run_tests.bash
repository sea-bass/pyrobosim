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
pushd "${SCRIPT_DIR}/../pyrobosim" || exit
python3 -m pytest . \
    --cov="pyrobosim" --cov-branch \
    --cov-report term \
    --cov-report html:"${TEST_RESULTS_DIR}/test_results_coverage_html" \
    --cov-report xml:"${TEST_RESULTS_DIR}/test_results_coverage.xml" \
    --junitxml="${TEST_RESULTS_DIR}/test_results.xml" \
    --html="${TEST_RESULTS_DIR}/test_results.html" \
    --self-contained-html \
    | tee "${TEST_RESULTS_DIR}/pytest-coverage.txt" || SUCCESS=$?
echo ""
popd || exit

# Run ROS package tests, if using a ROS distro.
ROS_DISTRO=$1
if [[ -n "${ROS_DISTRO}" && -n "${COLCON_PREFIX_PATH}" ]]
then
    WORKSPACE_DIR="${COLCON_PREFIX_PATH}/../"
    echo "Running ROS package unit tests from ${WORKSPACE_DIR}..."
    pushd "${WORKSPACE_DIR}" > /dev/null || exit
    colcon test \
        --packages-select pyrobosim_ros \
        --event-handlers console_cohesion+ \
        --pytest-with-coverage || SUCCESS=$?
    echo ""
    colcon test-result --verbose \
        | tee "${TEST_RESULTS_DIR}/test_results_ros.xml"
    popd > /dev/null || exit
fi

exit ${SUCCESS}

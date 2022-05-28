# Runs all unit tests

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
TEST_RESULTS_DIR=$SCRIPT_DIR/../test_results

pytest \
 --junitxml=$TEST_RESULTS_DIR/test-results.xml \
 --html=$TEST_RESULTS_DIR/test_results.html --self-contained-html

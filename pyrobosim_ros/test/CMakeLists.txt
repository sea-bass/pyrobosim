# CMakeLists.txt for pyrobosim_ros tests
find_package(ament_cmake_pytest REQUIRED)

set(AMENT_CMAKE_PYTEST_WITH_COVERAGE ON)

set(_pytest_tests
    test_ros_conversions.py
)
foreach(_test_path ${_pytest_tests})
get_filename_component(_test_name ${_test_path} NAME_WE)
ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
endforeach()

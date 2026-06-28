cmake_minimum_required(VERSION 3.22)
project(pyrobosim)

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the pyrobosim Python package.
ament_python_install_package(
  pyrobosim
  PACKAGE_DIR pyrobosim
)

# Also install the data folder into the package share directory.
install(DIRECTORY
  pyrobosim/data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

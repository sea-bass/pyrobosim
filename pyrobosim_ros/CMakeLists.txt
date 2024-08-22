cmake_minimum_required(VERSION 3.22)
project(pyrobosim_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pyrobosim_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install examples
install(PROGRAMS
  examples/demo.py
  examples/demo_commands.py
  examples/demo_pddl_world.py
  examples/demo_pddl_planner.py
  examples/demo_pddl_goal_publisher.py
  examples/demo_velocity_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install pyrobosim_ros Python package
ament_python_install_package(
  pyrobosim_ros
  PACKAGE_DIR pyrobosim_ros)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if enabled
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()

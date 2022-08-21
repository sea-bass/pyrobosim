cmake_minimum_required(VERSION 3.8)
project(pyrobosim_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(
    pyrobosim
    PACKAGE_DIR ../pyrobosim/pyrobosim)
install(PROGRAMS
  examples/demo.py
  examples/demo_commands.py
  examples/demo_pddl_world.py
  examples/demo_pddl_planner.py
  examples/demo_pddl_goal_publisher.py
  ../pyrobosim/examples/test_prm.py
  ../pyrobosim/examples/test_rrt.py
  ../pyrobosim/examples/demo_pddl.py
  ../pyrobosim/examples/demo_world_save.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data files
install(DIRECTORY
  ../pyrobosim/pyrobosim/data
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
